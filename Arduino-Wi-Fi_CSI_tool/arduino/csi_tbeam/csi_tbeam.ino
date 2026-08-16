/***************************************************************************
 * c ― T-Beam 2台で動く CSI 実証プログラム（1ファイル・役割切替式）
 ***************************************************************************
 *
 * ■ 構成
 *   送信機 t : ESP-NOW ブロードキャストを指定レートで送信（接続不要）
 *   受信機 r : 同一チャネルで t のフレームを受信し CSI を取得・表示
 *   → t×1 : r×N にそのまま拡張可能（r を増やしても t は無変更）
 *
 * ■ 動作条件
 *   - LilyGO T-Beam（ESP32 系）．LoRa/GPS は未使用（触らない）
 *   - Arduino IDE ＋ ESP32 ボードパッケージ 3.x，ボードは "T-Beam" か
 *     "ESP32 Dev Module"
 *   - シリアルモニタ 921600 baud，改行 LF または CR+LF
 *
 * ■ 最初の使い方（2台での実証手順）
 *   1. 両方の T-Beam に本スケッチを書き込む
 *   2. 1台目のシリアルモニタで  ROLE r  と入力 → 自動再起動して受信機に
 *   3. 2台目のシリアルモニタで  ROLE t  と入力 → 自動再起動して送信機に
 *   4. r 側に毎秒  [STAT] rx=10/s loss=0.0% rssi=-42 amp(sc44)=12.3
 *      のような行が出れば成立．t-r 間に手をかざす／人が歩くと amp が変動する
 *   5. 波形を見たい場合は r 側で  DISP CSV  に切り替え，PC で
 *      python serial_plot_csi_live.py COM5  を実行（同梱スクリプト）
 *
 * ■ コマンド一覧（シリアルモニタから入力．値は例）
 *   --- 役割 -----------------------------------------------------------
 *   ROLE t / ROLE r       役割を設定し保存→自動再起動
 *   --- 共通（t・r で一致させるもの） -----------------------------------
 *   CHANNEL 6             Wi-Fi チャネル 1-13（t と r は必ず同じ値に！）
 *   TXPOWER 20            送信電力 2-20 dBm
 *   --- t（送信機）用 ----------------------------------------------------
 *   RATE 10               送信レート 1-500 Hz（実証は 10-50 が見やすい）
 *   SIZE 32               ペイロード長 8-250 バイト（ESP-NOW 上限 250）
 *   --- r（受信機）用 ----------------------------------------------------
 *   DISP STAT             毎秒の要約表示（既定．実証向け）
 *   DISP CSV              全データ CSV 出力（解析・プロット向け）
 *   SC 44                 STAT で表示するサブキャリア番号 0-63
 *   FORMAT RAW            CSV の配列形式 RAW(生I/Q)|AMP(振幅)|PHASE(位相)
 *   LLTF 1                1: 配列を先頭128値(L-LTF)に制限 / 0: 全部
 *   FILTER AUTO           最初に受信した t の MAC に自動ロック（既定）
 *   FILTER 24:6F:28:xx:xx:xx   手動で MAC 指定
 *   FILTER OFF            フィルタ無し（周囲の全 Wi-Fi フレームの CSI も拾う）
 *   CSILLTF 1 / CSIHT 1 / CSISTBC 1 / CSIMERGE 1 /
 *   CSICHFILT 0 / CSISCALE 0 / CSISHIFT 0     CSI 取得詳細（通常は触らない）
 *   --- 時刻（r の CSV に実時刻を入れたい場合のみ） ----------------------
 *   TIME 1765346400.123   UNIX 秒を設定（PC の時計を手で写す）
 *   --- 管理 -------------------------------------------------------------
 *   SHOW / SAVE / RESET / REBOOT / HELP
 *
 * ■ 設定はこのすぐ下の『設定ブロック』に集中配置してある．
 *   コンパイル時の既定値は #define，実行時はコマンド（SAVE で永続化）．
 *
 * ■ 技術メモ
 *   - ESP-NOW の既定レート 1Mbps(11b/DSSS) では LTF が無く CSI が取れない
 *     ため，esp_wifi_config_espnow_rate() で MCS0(OFDM) に固定している
 *   - チャネル 14 は 11b 専用のため CSI 不可．日本は 1-13 が実用範囲で，
 *     esp_wifi_set_country() を JP に設定して 12,13 も使えるようにしている
 *   - t のペイロード先頭 4 バイトは連番（uint32）．r はこれでロス率を計算
 *   - CSI コールバックは Wi-Fi タスク内で走るためコピーのみ．整形・表示は
 *     loop() 側で行う（キュー渡し）
 ***************************************************************************/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <math.h>
#include <sys/time.h>

/* ======================================================================
 *  設定ブロック（★ここに全設定を集中配置★）
 * ====================================================================== */

/* --- コンパイル時デフォルト（RESET 時にもこの値に戻る） --- */
#define DEF_ROLE          'r'    // 't':送信機 / 'r':受信機 / '?':起動時に質問
#define DEF_CHANNEL       6      // Wi-Fi チャネル 1-13（t・r で一致させる）
#define DEF_TXPOWER_DBM   20     // 送信電力 2-20 dBm
#define DEF_RATE_HZ       10     // [t] 送信レート 1-500 Hz
#define DEF_PAYLOAD       32     // [t] ペイロード長 8-250 B（先頭4Bは連番）
#define DEF_DISP_CSV      0      // [r] 0:STAT 表示 / 1:CSV 出力
#define DEF_SUBCARRIER    44     // [r] STAT で表示するサブキャリア 0-63
#define DEF_FORMAT        0      // [r] CSV 配列 0:RAW 1:AMP 2:PHASE
#define DEF_LLTF_ONLY     1      // [r] 1:先頭128値のみ
#define SERIAL_BAUD       921600
#define CSI_QUEUE_LEN     24

/* --- 実行時設定（SHOW で全表示，SAVE で NVS 保存） --- */
struct Settings {
  char     role;          // 't' / 'r' / '?'
  uint8_t  channel;       // 共通：1-13
  uint8_t  txpower_dbm;   // 共通：2-20
  uint16_t rate_hz;       // t：1-500
  uint16_t payload_size;  // t：8-250
  uint8_t  disp_csv;      // r：0=STAT 1=CSV
  uint8_t  subcarrier;    // r：0-63
  uint8_t  format;        // r：0=RAW 1=AMP 2=PHASE
  uint8_t  lltf_only;     // r：0/1
  uint8_t  filter_mode;   // r：0=OFF 1=AUTO(未学習) 2=固定(学習済/手動)
  uint8_t  filter_mac[6];
  uint8_t  csi_lltf, csi_ht, csi_stbc, csi_merge,  // r：CSI 取得詳細
           csi_chfilt, csi_scale, csi_shift;
};
/* ====================== 設定ブロックここまで ========================== */

#define FMT_RAW 0
#define FMT_AMP 1
#define FMT_PHASE 2
#define FILT_OFF 0
#define FILT_AUTO 1
#define FILT_FIXED 2

Settings    g;
Preferences prefs;
volatile bool g_time_set = false;

/* --- t 用 --- */
uint8_t  bcastAddr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint32_t txSeq = 0, txCount1s = 0;
uint32_t nextSendUs = 0;
uint8_t  payload[250];

/* --- r 用 --- */
struct CsiMsg {
  wifi_pkt_rx_ctrl_t rx_ctrl;
  uint8_t  mac[6];
  uint16_t len;
  int8_t   buf[384];
};
QueueHandle_t csiQueue;
uint32_t g_dropped = 0;
volatile uint32_t rxCount1s = 0;          // 1秒間の CSI 受信数
volatile uint32_t seqFirst = 0, seqLast = 0, seqRecv = 0;  // ロス率計算用
volatile bool     seqValid = false;
volatile int      lastRssi = 0;
float             lastAmp = 0;            // 表示サブキャリアの最新振幅

/* ======== 設定の保存・読み出し ======== */
void settingsDefault() {
  memset(&g, 0, sizeof(g));
  g.role = DEF_ROLE;
  g.channel = DEF_CHANNEL;
  g.txpower_dbm = DEF_TXPOWER_DBM;
  g.rate_hz = DEF_RATE_HZ;
  g.payload_size = DEF_PAYLOAD;
  g.disp_csv = DEF_DISP_CSV;
  g.subcarrier = DEF_SUBCARRIER;
  g.format = DEF_FORMAT;
  g.lltf_only = DEF_LLTF_ONLY;
  g.filter_mode = FILT_AUTO;
  g.csi_lltf = 1; g.csi_ht = 1; g.csi_stbc = 1; g.csi_merge = 1;
  g.csi_chfilt = 0; g.csi_scale = 0; g.csi_shift = 0;
}
void settingsLoad() {
  settingsDefault();
  prefs.begin("csi", true);
  if (prefs.getBytesLength("set") == sizeof(g)) prefs.getBytes("set", &g, sizeof(g));
  prefs.end();
}
void settingsSave()  { prefs.begin("csi", false); prefs.putBytes("set", &g, sizeof(g)); prefs.end(); }
void settingsErase() { prefs.begin("csi", false); prefs.remove("set"); prefs.end(); }

void settingsValidate() {
  if (g.role != 't' && g.role != 'r' && g.role != '?') g.role = DEF_ROLE;
  if (g.channel < 1 || g.channel > 13) g.channel = DEF_CHANNEL;
  if (g.txpower_dbm < 2 || g.txpower_dbm > 20) g.txpower_dbm = DEF_TXPOWER_DBM;
  if (g.rate_hz < 1 || g.rate_hz > 500) g.rate_hz = DEF_RATE_HZ;
  if (g.payload_size < 8 || g.payload_size > 250) g.payload_size = DEF_PAYLOAD;
  if (g.disp_csv > 1) g.disp_csv = DEF_DISP_CSV;
  if (g.subcarrier > 63) g.subcarrier = DEF_SUBCARRIER;
  if (g.format > FMT_PHASE) g.format = DEF_FORMAT;
  if (g.lltf_only > 1) g.lltf_only = DEF_LLTF_ONLY;
  if (g.filter_mode > FILT_FIXED) g.filter_mode = FILT_AUTO;
  if (g.csi_lltf > 1) g.csi_lltf = 1;
  if (g.csi_ht > 1) g.csi_ht = 1;
  if (g.csi_stbc > 1) g.csi_stbc = 1;
  if (g.csi_merge > 1) g.csi_merge = 1;
  if (g.csi_chfilt > 1) g.csi_chfilt = 0;
  if (g.csi_scale > 1) g.csi_scale = 0;
  if (g.csi_shift > 15) g.csi_shift = 0;
}

/* ======== 無線共通設定 ======== */
void applyRadio() {
  // 日本の規制テーブルを設定（ch12,13 を使えるようにする）
  wifi_country_t jp = {};
  memcpy(jp.cc, "JP", 3);              // "JP\0"
  jp.schan  = 1;
  jp.nchan  = 13;
  jp.policy = WIFI_COUNTRY_POLICY_MANUAL;   // フィールド名で明示（順序ずれ防止）
  esp_wifi_set_country(&jp);
  esp_wifi_set_channel(g.channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(g.txpower_dbm * 4);   // 単位 0.25dBm
}

/* ======== CSI（r 専用） ======== */
void csiCallback(void *ctx, wifi_csi_info_t *info);

void applyCsiConfig() {
  esp_wifi_set_csi(0);
  wifi_csi_config_t c = {};
  c.lltf_en           = g.csi_lltf;
  c.htltf_en          = g.csi_ht;
  c.stbc_htltf2_en    = g.csi_stbc;
  c.ltf_merge_en      = g.csi_merge;
  c.channel_filter_en = g.csi_chfilt;
  c.manu_scale        = g.csi_scale;
  c.shift             = g.csi_shift;
  esp_wifi_set_csi_config(&c);
  esp_wifi_set_csi_rx_cb(&csiCallback, NULL);
  esp_wifi_set_csi(1);
}

// Wi-Fi タスク内で呼ばれる：コピーのみ（重い処理は厳禁）
void csiCallback(void *ctx, wifi_csi_info_t *info) {
  if (g.filter_mode == FILT_FIXED &&
      memcmp(info->mac, g.filter_mac, 6) != 0) return;
  if (g.filter_mode == FILT_AUTO) return;   // 相手 MAC 学習前は捨てる
  CsiMsg m;
  m.rx_ctrl = info->rx_ctrl;
  memcpy(m.mac, info->mac, 6);
  m.len = info->len > sizeof(m.buf) ? sizeof(m.buf) : info->len;
  memcpy(m.buf, info->buf, m.len);
  if (xQueueSend(csiQueue, &m, 0) != pdTRUE) g_dropped++;
}

// ESP-NOW 受信（r）：連番からロス率を計算，AUTO なら相手 MAC を学習
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (g.filter_mode == FILT_AUTO) {
    memcpy(g.filter_mac, info->src_addr, 6);
    g.filter_mode = FILT_FIXED;
    Serial.printf("[PAIRED] tx=%02X:%02X:%02X:%02X:%02X:%02X にロックしました"
                  "（変更は FILTER コマンド）\n",
                  g.filter_mac[0], g.filter_mac[1], g.filter_mac[2],
                  g.filter_mac[3], g.filter_mac[4], g.filter_mac[5]);
  }
  if (g.filter_mode == FILT_FIXED &&
      memcmp(info->src_addr, g.filter_mac, 6) != 0) return;
  if (len >= 4) {
    uint32_t seq; memcpy(&seq, data, 4);
    if (!seqValid) { seqFirst = seq; seqValid = true; }
    seqLast = seq;
    seqRecv++;
  }
}

/* ======== CSV 1 行の整形（Hernandez 互換 26 カラム） ======== */
void printCsvLine(CsiMsg &m) {
  static char line[3200];
  double ts;
  if (g_time_set) {
    struct timeval tv; gettimeofday(&tv, NULL);
    ts = tv.tv_sec + tv.tv_usec / 1e6;
  } else {
    ts = micros() / 1e6;
  }
  int n = snprintf(line, sizeof(line),
    "CSI_DATA,R,%02X:%02X:%02X:%02X:%02X:%02X,"
    "%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,"
    "%u,%.6f,%u,[",
    m.mac[0], m.mac[1], m.mac[2], m.mac[3], m.mac[4], m.mac[5],
    m.rx_ctrl.rssi, m.rx_ctrl.rate, m.rx_ctrl.sig_mode, m.rx_ctrl.mcs,
    m.rx_ctrl.cwb, m.rx_ctrl.smoothing, m.rx_ctrl.not_sounding,
    m.rx_ctrl.aggregation, m.rx_ctrl.stbc, m.rx_ctrl.fec_coding,
    m.rx_ctrl.sgi, m.rx_ctrl.noise_floor, m.rx_ctrl.ampdu_cnt,
    m.rx_ctrl.channel, m.rx_ctrl.secondary_channel, m.rx_ctrl.timestamp,
    m.rx_ctrl.ant, m.rx_ctrl.sig_len, m.rx_ctrl.rx_state,
    g_time_set ? 1 : 0, ts, m.len);

  int dataLen = g.lltf_only ? (m.len < 128 ? m.len : 128) : m.len;
  switch (g.format) {
    case FMT_AMP:    // 振幅．buf は [虚部, 実部] の交互
      for (int i = 0; i < dataLen / 2 && n < (int)sizeof(line) - 12; i++)
        n += snprintf(line + n, sizeof(line) - n, "%.1f ",
                      sqrtf((float)(m.buf[2*i]*m.buf[2*i] + m.buf[2*i+1]*m.buf[2*i+1])));
      break;
    case FMT_PHASE:  // 位相 [rad]
      for (int i = 0; i < dataLen / 2 && n < (int)sizeof(line) - 12; i++)
        n += snprintf(line + n, sizeof(line) - n, "%.3f ",
                      atan2f(m.buf[2*i], m.buf[2*i+1]));
      break;
    default:         // 生 I/Q
      for (int i = 0; i < dataLen && n < (int)sizeof(line) - 8; i++)
        n += snprintf(line + n, sizeof(line) - n, "%d ", m.buf[i]);
      break;
  }
  n += snprintf(line + n, sizeof(line) - n, "]\n");
  Serial.write((const uint8_t *)line, n);
}

/* ======== コマンド処理（シリアルから） ======== */
bool parseMac(const char *s, uint8_t mac[6]) {
  unsigned v[6];
  if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) != 6) return false;
  for (int i = 0; i < 6; i++) mac[i] = v[i];
  return true;
}

String processCommand(String cmd) {
  cmd.trim();
  String out;

  if (cmd.startsWith("ROLE ")) {
    char r = tolower(cmd.charAt(5));
    if (r == 't' || r == 'r') {
      g.role = r; settingsSave();
      Serial.printf("OK ROLE=%c → 保存して再起動します\n", r);
      delay(300); ESP.restart();
    } else out = "ERR ROLE t|r\n";
  } else if (cmd.startsWith("CHANNEL ")) {
    int v = cmd.substring(8).toInt();
    if (v >= 1 && v <= 13) {
      g.channel = v;
      settingsSave();
      Serial.printf("OK CHANNEL=%d → 保存して再起動します（t と r で一致させること）\n", v);
      delay(300);
      ESP.restart();
    }
    else out = "ERR CHANNEL 1-13（14 は 11b 専用で CSI 不可）\n";
  } else if (cmd.startsWith("TXPOWER ")) {
    int v = cmd.substring(8).toInt();
    if (v >= 2 && v <= 20) { g.txpower_dbm = v; applyRadio();
      out = "OK TXPOWER=" + String(v) + "dBm\n"; }
    else out = "ERR TXPOWER 2-20\n";
  } else if (cmd.startsWith("RATE ")) {
    int v = cmd.substring(5).toInt();
    if (v >= 1 && v <= 500) { g.rate_hz = v; out = "OK RATE=" + String(v) + "Hz\n"; }
    else out = "ERR RATE 1-500\n";
  } else if (cmd.startsWith("SIZE ")) {
    int v = cmd.substring(5).toInt();
    if (v >= 8 && v <= 250) { g.payload_size = v; out = "OK SIZE=" + String(v) + "B\n"; }
    else out = "ERR SIZE 8-250（ESP-NOW 上限 250）\n";
  } else if (cmd.startsWith("DISP ")) {
    String d = cmd.substring(5);
    if      (d == "STAT") g.disp_csv = 0;
    else if (d == "CSV")  g.disp_csv = 1;
    else return "ERR DISP STAT|CSV\n";
    out = "OK DISP=" + d + "\n";
  } else if (cmd.startsWith("SC ")) {
    int v = cmd.substring(3).toInt();
    if (v >= 0 && v <= 63) { g.subcarrier = v; out = "OK SC=" + String(v) + "\n"; }
    else out = "ERR SC 0-63\n";
  } else if (cmd.startsWith("FORMAT ")) {
    String f = cmd.substring(7);
    if      (f == "RAW")   g.format = FMT_RAW;
    else if (f == "AMP")   g.format = FMT_AMP;
    else if (f == "PHASE") g.format = FMT_PHASE;
    else return "ERR FORMAT RAW|AMP|PHASE\n";
    out = "OK FORMAT=" + f + "\n";
  } else if (cmd.startsWith("LLTF ")) {
    g.lltf_only = cmd.substring(5).toInt() ? 1 : 0;
    out = "OK LLTF=" + String(g.lltf_only) + "\n";
  } else if (cmd.startsWith("FILTER ")) {
    String f = cmd.substring(7);
    if (f == "OFF")       { g.filter_mode = FILT_OFF;  out = "OK FILTER=OFF（全フレームの CSI を拾う）\n"; }
    else if (f == "AUTO") { g.filter_mode = FILT_AUTO; out = "OK FILTER=AUTO（次に受信した t にロック）\n"; }
    else if (parseMac(f.c_str(), g.filter_mac)) {
      g.filter_mode = FILT_FIXED; out = "OK FILTER=" + f + "\n";
    } else out = "ERR FILTER <MAC|AUTO|OFF>\n";
  }
  /* --- CSI 取得詳細（通常は既定のまま） --- */
  else if (cmd.startsWith("CSILLTF "))  { g.csi_lltf  = cmd.substring(8).toInt()?1:0; applyCsiConfig(); out = "OK\n"; }
  else if (cmd.startsWith("CSIHT "))    { g.csi_ht    = cmd.substring(6).toInt()?1:0; applyCsiConfig(); out = "OK\n"; }
  else if (cmd.startsWith("CSISTBC "))  { g.csi_stbc  = cmd.substring(8).toInt()?1:0; applyCsiConfig(); out = "OK\n"; }
  else if (cmd.startsWith("CSIMERGE ")) { g.csi_merge = cmd.substring(9).toInt()?1:0; applyCsiConfig(); out = "OK\n"; }
  else if (cmd.startsWith("CSICHFILT ")){ g.csi_chfilt= cmd.substring(10).toInt()?1:0;applyCsiConfig(); out = "OK\n"; }
  else if (cmd.startsWith("CSISCALE ")) { g.csi_scale = cmd.substring(9).toInt()?1:0; applyCsiConfig(); out = "OK\n"; }
  else if (cmd.startsWith("CSISHIFT ")) {
    int v = cmd.substring(9).toInt();
    if (v >= 0 && v <= 15) { g.csi_shift = v; applyCsiConfig(); out = "OK\n"; }
    else out = "ERR CSISHIFT 0-15\n";
  }
  else if (cmd.startsWith("TIME ")) {
    double t = atof(cmd.substring(5).c_str());
    if (t > 1e9) {
      struct timeval tv;
      tv.tv_sec = (time_t)t; tv.tv_usec = (suseconds_t)((t - (time_t)t) * 1e6);
      settimeofday(&tv, NULL);
      g_time_set = true;
      out = "OK TIME " + String(t, 6) + "\n";
    } else out = "ERR TIME <unix_epoch_sec>\n";
  }
  else if (cmd == "SAVE")  { settingsSave(); out = "OK SAVE（次回起動も有効）\n"; }
  else if (cmd == "RESET") {
    settingsErase(); settingsDefault();
    out = "OK RESET（再起動で既定値に戻ります）\n";
  }
  else if (cmd == "REBOOT") { Serial.println("OK REBOOT"); delay(200); ESP.restart(); }
  else if (cmd == "HELP") {
    out = "ROLE t|r | CHANNEL/TXPOWER | RATE/SIZE(t用) | "
          "DISP STAT|CSV / SC / FORMAT / LLTF / FILTER(r用) | "
          "CSIxxx | TIME | SHOW/SAVE/RESET/REBOOT/HELP\n";
  }
  else if (cmd == "SHOW") {
    const char *fmts[]  = { "RAW", "AMP", "PHASE" };
    const char *filts[] = { "OFF", "AUTO", "FIXED" };
    char buf[420];
    snprintf(buf, sizeof(buf),
      "ROLE=%c CH=%u TXPWR=%udBm\n"
      "[t] RATE=%uHz SIZE=%uB SENT=%lu\n"
      "[r] DISP=%s SC=%u FORMAT=%s LLTF=%u FILTER=%s"
      " (%02X:%02X:%02X:%02X:%02X:%02X)\n"
      "[r] CSI: LLTF=%u HT=%u STBC=%u MERGE=%u CHFILT=%u SCALE=%u SHIFT=%u\n"
      "TIME_SET=%u DROPPED=%lu MyMAC=%s\n",
      g.role, g.channel, g.txpower_dbm,
      g.rate_hz, g.payload_size, (unsigned long)txSeq,
      g.disp_csv ? "CSV" : "STAT", g.subcarrier, fmts[g.format % 3],
      g.lltf_only, filts[g.filter_mode % 3],
      g.filter_mac[0], g.filter_mac[1], g.filter_mac[2],
      g.filter_mac[3], g.filter_mac[4], g.filter_mac[5],
      g.csi_lltf, g.csi_ht, g.csi_stbc, g.csi_merge,
      g.csi_chfilt, g.csi_scale, g.csi_shift,
      g_time_set ? 1 : 0, (unsigned long)g_dropped,
      WiFi.macAddress().c_str());
    out = buf;
  } else {
    out = "ERR unknown（HELP で一覧）\n";
  }
  return out;
}

/* ======== setup ======== */
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  settingsLoad();
  settingsValidate();

  Serial.println("\n=== csi_tbeam (T-Beam CSI demo) ===");

  // 役割未設定なら入力を待つ（書き込み直後の初回起動）
  if (g.role != 't' && g.role != 'r') {
    Serial.println("役割を入力してください: ROLE t（送信機）/ ROLE r（受信機）");
  }

  WiFi.mode(WIFI_STA);       // t・r とも STA モード（接続はしない）
  WiFi.disconnect();
  WiFi.setSleep(false);      // 省電力オフ（レート・CSI を安定させる）
  applyRadio();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERR: esp_now_init failed");
  }

  if (g.role == 't') {
    /* --- 送信機初期化 --- */
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, bcastAddr, 6);
    peer.channel = g.channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    // 既定の 1Mbps(11b) では CSI が取れないため OFDM(MCS0) に固定
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_MCS0_LGI);
    memset(payload, 'C', sizeof(payload));
    nextSendUs = micros();
    Serial.printf("[ROLE t] ch=%u rate=%uHz size=%uB → 送信開始\n",
                  g.channel, g.rate_hz, g.payload_size);
  } else if (g.role == 'r') {
    /* --- 受信機初期化 --- */
    csiQueue = xQueueCreate(CSI_QUEUE_LEN, sizeof(CsiMsg));
    esp_now_register_recv_cb(onEspNowRecv);
    applyCsiConfig();
    Serial.printf("[ROLE r] ch=%u disp=%s sc=%u → 受信待機（FILTER=%s）\n",
                  g.channel, g.disp_csv ? "CSV" : "STAT", g.subcarrier,
                  g.filter_mode == FILT_AUTO ? "AUTO" :
                  g.filter_mode == FILT_OFF  ? "OFF"  : "FIXED");
    if (g.disp_csv) {
      Serial.println("type,role,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,"
                     "not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,"
                     "ampdu_cnt,channel,secondary_channel,local_timestamp,ant,"
                     "sig_len,rx_state,real_time_set,real_timestamp,len,CSI_DATA");
    }
  }
}

/* ======== loop ======== */
void loop() {
  uint32_t nowMs = millis();
  static uint32_t lastStatMs = 0;

  /* ---------- t：等間隔送信 ---------- */
  if (g.role == 't') {
    uint32_t now = micros();
    if ((int32_t)(now - nextSendUs) >= 0) {
      memcpy(payload, &txSeq, 4);                     // 先頭 4B = 連番
      esp_now_send(bcastAddr, payload, g.payload_size);
      txSeq++; txCount1s++;
      nextSendUs += 1000000UL / g.rate_hz;
      if ((int32_t)(now - nextSendUs) > 100000) nextSendUs = now;  // 遅延蓄積防止
    }
    if (nowMs - lastStatMs >= 1000) {
      lastStatMs = nowMs;
      Serial.printf("[STAT t] tx=%lu/s total=%lu ch=%u\n",
                    (unsigned long)txCount1s, (unsigned long)txSeq, g.channel);
      txCount1s = 0;
    }
  }

  /* ---------- r：CSI キュー処理と表示 ---------- */
  if (g.role == 'r') {
    CsiMsg m;
    while (xQueueReceive(csiQueue, &m, 0) == pdTRUE) {
      rxCount1s++;
      lastRssi = m.rx_ctrl.rssi;
      // 表示サブキャリアの振幅（buf は [虚部, 実部] の交互）
      int sc = g.subcarrier;
      if (2 * sc + 1 < m.len)
        lastAmp = sqrtf((float)(m.buf[2*sc]*m.buf[2*sc] +
                                m.buf[2*sc+1]*m.buf[2*sc+1]));
      if (g.disp_csv) printCsvLine(m);
    }

    if (!g.disp_csv && nowMs - lastStatMs >= 1000) {
      lastStatMs = nowMs;
      // ロス率：連番の期待数に対する不足分
      float loss = 0;
      uint32_t expected = seqValid ? (seqLast - seqFirst + 1) : 0;
      if (expected > 0) loss = 100.0f * (1.0f - (float)seqRecv / expected);
      if (loss < 0) loss = 0;
      Serial.printf("[STAT r] rx=%lu/s loss=%.1f%% rssi=%d amp(sc%u)=%.1f drop=%lu\n",
                    (unsigned long)rxCount1s, loss, lastRssi,
                    g.subcarrier, lastAmp, (unsigned long)g_dropped);
      rxCount1s = 0;
      seqValid = false; seqRecv = 0;   // 1秒窓でリセット
    }
  }

  /* ---------- 共通：シリアルコマンド ---------- */
  static String serialLine;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (serialLine.length() > 0) Serial.print(processCommand(serialLine));
      serialLine = "";
    } else if (c != '\r') {
      serialLine += c;
    }
  }

  delay(1);
}
