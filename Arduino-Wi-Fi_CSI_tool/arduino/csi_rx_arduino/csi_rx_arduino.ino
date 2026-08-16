/***************************************************************************
 * csi_rx_arduino ― ESP32 CSI 受信機（softAP / CSI-RX）Arduino IDE 版
 ***************************************************************************
 *
 * ■ 動作条件
 *   - Arduino IDE ＋ ESP32 ボードパッケージ **3.x 以降**（必須）
 *     ツール > ボード > esp32 > "ESP32 Dev Module" を選択
 *     （コア 3.x のプリコンパイル済みライブラリは CSI 有効：
 *       CONFIG_ESP_WIFI_CSI_ENABLED=y．1.0.x 世代は無効なので不可）
 *   - シリアルモニタは 921600 baud に設定
 *
 * ■ 設定方法（2系統，同じコマンド．SAVE で次回起動も有効）
 *   a) UDP:    PC から python csi_ctl.py SHOW  （宛先 192.168.4.1:5006）
 *   b) シリアル: Arduino IDE のシリアルモニタにコマンドを入力して送信
 *              （改行コードは「LF」または「CR+LF」を選択）
 *
 * ■ コマンド一覧（値は例）
 *   --- Wi-Fi（AP）設定 -----------------------------------------------
 *   SSID mynetwork        SSID 変更（接続中の STA は再接続される）
 *   PASS mypassword       パスワード変更（8文字未満は認証なしAPになる）
 *   CHANNEL 6             チャネル 1-13
 *   BW 20                 帯域 20|40 MHz（40 にすると CSI 長が最大384に）
 *   TXPOWER 20            送信電力 2-20 dBm
 *   PROTOCOL BGN          11B|BG|BGN
 *   MAXCONN 16            最大接続 STA 数 1-16
 *   HIDDEN 0              SSID 隠蔽 0|1
 *   --- CSI 取得設定（esp_wifi_set_csi_config 全項目）-----------------
 *   CSILLTF 1 / CSIHT 1 / CSISTBC 1 / CSIMERGE 1
 *   CSICHFILT 0 / CSISCALE 0 / CSISHIFT 0
 *   --- 出力設定 -------------------------------------------------------
 *   FORMAT RAW            RAW(生I/Q)|AMP(振幅)|PHASE(位相)
 *   MODE FULL             FULL(26カラム Hernandez互換)|COMPACT(5カラム)
 *   OUTPUT UDP            UDP|SERIAL|BOTH
 *   TARGET 192.168.4.255  UDP 送出先（255 終わりでブロードキャスト）
 *   PORT 5005             UDP 送出先ポート
 *   LLTF 1                1: 配列を先頭128値に制限 / 0: 全部
 *   FILTER 2C:BC:BB:83:1E:D8   指定MACのみ出力（解除は FILTER OFF）
 *   --- 時刻 -----------------------------------------------------------
 *   TIME 1765346400.123   UNIX 秒で実時刻を設定
 *                         （PC 時刻を送るなら python csi_ctl.py time）
 *   --- 管理 -----------------------------------------------------------
 *   SHOW / SAVE / RESET / REBOOT / HELP
 *
 * ■ 使用例（典型手順）
 *   1. 書き込み → 2. PC を本機 AP に接続 → 3. csi_ctl.py time（時刻同期）
 *   → 4. csi_ctl.py SHOW（確認）→ 5. udp_logging.py（収集）
 *   ※既存 PC スクリプト互換は FORMAT RAW ＋ MODE FULL（既定値）のとき
 ***************************************************************************/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <math.h>
#include <sys/time.h>
#include "esp_wifi.h"          // esp_wifi_set_csi_* / set_bandwidth / set_protocol

/* ======== コンパイル時デフォルト（RESET 時にもこの値に戻る） ======== */
#define DEF_AP_SSID        "myssid"
#define DEF_AP_PASS        "mypassword"
#define DEF_WIFI_CHANNEL   6
#define DEF_TARGET_IP      "192.168.4.255"   // ブロードキャスト：PC の IP 変更に強い
#define DEF_TARGET_PORT    5005
#define CMD_PORT           5006              // コマンド受付ポート（固定）
#define CSI_QUEUE_LEN      24
#define SERIAL_BAUD        921600

#define FMT_RAW    0
#define FMT_AMP    1
#define FMT_PHASE  2
#define OUT_UDP    0x01
#define OUT_SERIAL 0x02

/* ======== 実行時設定（SHOW で表示される項目はすべてここ） ======== */
struct Settings {
  char     ssid[33];
  char     pass[65];
  uint8_t  channel, bw40, txpower_dbm, protocol, maxconn, hidden;
  uint8_t  csi_lltf, csi_ht, csi_stbc, csi_merge, csi_chfilt, csi_scale, csi_shift;
  uint8_t  format, compact, out;
  char     target_ip[16];
  uint16_t target_port;
  uint8_t  lltf_only;
  uint8_t  filter_on;
  uint8_t  filter_mac[6];
};

Settings    g;
Preferences prefs;
WiFiUDP     udpData;     // CSI データ送出用
WiFiUDP     udpCmd;      // コマンド受付用
uint32_t    g_dropped = 0;
volatile bool g_time_set = false;

void settingsDefault() {
  memset(&g, 0, sizeof(g));
  strlcpy(g.ssid, DEF_AP_SSID, sizeof(g.ssid));
  strlcpy(g.pass, DEF_AP_PASS, sizeof(g.pass));
  g.channel = DEF_WIFI_CHANNEL;
  g.bw40 = 0; g.txpower_dbm = 20;
  g.protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
  g.maxconn = 16; g.hidden = 0;
  g.csi_lltf = 1; g.csi_ht = 1; g.csi_stbc = 1; g.csi_merge = 1;
  g.csi_chfilt = 0; g.csi_scale = 0; g.csi_shift = 0;
  g.format = FMT_RAW; g.compact = 0; g.out = OUT_UDP;
  strlcpy(g.target_ip, DEF_TARGET_IP, sizeof(g.target_ip));
  g.target_port = DEF_TARGET_PORT;
  g.lltf_only = 1;
}

/* NVS（Preferences）に構造体丸ごと保存／読み出し */
void settingsLoad() {
  settingsDefault();
  prefs.begin("csi", true);
  if (prefs.getBytesLength("set") == sizeof(g)) prefs.getBytes("set", &g, sizeof(g));
  prefs.end();
}
void settingsSave()  { prefs.begin("csi", false); prefs.putBytes("set", &g, sizeof(g)); prefs.end(); }
void settingsErase() { prefs.begin("csi", false); prefs.remove("set"); prefs.end(); }

/* ======== Wi-Fi（AP）設定の適用 ======== */
void applyApConfig() {
  // softAP(ssid, pass, channel, hidden, max_connection)
  // パスワード 8 文字未満は WPA2 にできないため NULL（認証なし）にする
  WiFi.softAP(g.ssid, strlen(g.pass) >= 8 ? g.pass : NULL,
              g.channel, g.hidden, g.maxconn);
}
void applyRadioConfig() {
  esp_wifi_set_protocol(WIFI_IF_AP, g.protocol);
  esp_wifi_set_bandwidth(WIFI_IF_AP, g.bw40 ? WIFI_BW_HT40 : WIFI_BW_HT20);
  esp_wifi_set_max_tx_power(g.txpower_dbm * 4);   // 単位 0.25dBm
}

/* ======== CSI 取得設定の適用 ======== */
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

/* ======== CSI: コールバック → キュー → loop() で整形送出 ======== */
struct CsiMsg {
  wifi_pkt_rx_ctrl_t rx_ctrl;
  uint8_t  mac[6];
  uint16_t len;
  int8_t   buf[384];
};
QueueHandle_t csiQueue;

// Wi-Fi タスク内で呼ばれるためコピーのみ（重い処理は厳禁）
void csiCallback(void *ctx, wifi_csi_info_t *info) {
  if (g.filter_on && memcmp(info->mac, g.filter_mac, 6) != 0) return;
  CsiMsg m;
  m.rx_ctrl = info->rx_ctrl;
  memcpy(m.mac, info->mac, 6);
  m.len = info->len > sizeof(m.buf) ? sizeof(m.buf) : info->len;
  memcpy(m.buf, info->buf, m.len);
  if (xQueueSend(csiQueue, &m, 0) != pdTRUE) g_dropped++;
}

/* CSV 1 行を組み立てて UDP/シリアルへ */
void formatAndSend(CsiMsg &m) {
  static char line[3200];

  // 実時刻：TIME 設定済みなら UNIX エポック，未設定なら起動からの秒
  double ts;
  if (g_time_set) {
    struct timeval tv; gettimeofday(&tv, NULL);
    ts = tv.tv_sec + tv.tv_usec / 1e6;
  } else {
    ts = micros() / 1e6;
  }

  int n;
  if (g.compact) {
    // COMPACT: CSI_DATA,MAC,RSSI,時刻,len,[...]
    n = snprintf(line, sizeof(line),
      "CSI_DATA,%02X:%02X:%02X:%02X:%02X:%02X,%d,%.6f,%u,[",
      m.mac[0], m.mac[1], m.mac[2], m.mac[3], m.mac[4], m.mac[5],
      m.rx_ctrl.rssi, ts, m.len);
  } else {
    // FULL: Hernandez 互換 26 カラム（udp_logging.py 等がそのまま動く）
    n = snprintf(line, sizeof(line),
      "CSI_DATA,AP,%02X:%02X:%02X:%02X:%02X:%02X,"
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
  }

  int dataLen = g.lltf_only ? (m.len < 128 ? m.len : 128) : m.len;

  switch (g.format) {
    case FMT_AMP:    // 振幅 = sqrt(I^2+Q^2)．buf は [imag, real] 順
      for (int i = 0; i < dataLen / 2 && n < (int)sizeof(line) - 12; i++)
        n += snprintf(line + n, sizeof(line) - n, "%.1f ",
                      sqrtf((float)(m.buf[2*i]*m.buf[2*i] + m.buf[2*i+1]*m.buf[2*i+1])));
      break;
    case FMT_PHASE:  // 位相 = atan2(imag, real) [rad]
      for (int i = 0; i < dataLen / 2 && n < (int)sizeof(line) - 12; i++)
        n += snprintf(line + n, sizeof(line) - n, "%.3f ",
                      atan2f(m.buf[2*i], m.buf[2*i+1]));
      break;
    default:         // RAW: 生 I/Q（Hernandez 互換）
      for (int i = 0; i < dataLen && n < (int)sizeof(line) - 8; i++)
        n += snprintf(line + n, sizeof(line) - n, "%d ", m.buf[i]);
      break;
  }
  n += snprintf(line + n, sizeof(line) - n, "]\n");

  if (g.out & OUT_UDP) {
    udpData.beginPacket(g.target_ip, g.target_port);
    udpData.write((const uint8_t *)line, n);
    udpData.endPacket();
  }
  if (g.out & OUT_SERIAL) Serial.write((const uint8_t *)line, n);
}

/* ======== コマンド処理（UDP・シリアル共通） ======== */
bool parseMac(const char *s, uint8_t mac[6]) {
  unsigned v[6];
  if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) != 6) return false;
  for (int i = 0; i < 6; i++) mac[i] = v[i];
  return true;
}

String processCommand(String cmd) {
  cmd.trim();
  String out;

  /* --- Wi-Fi --- */
  if (cmd.startsWith("SSID ")) {
    strlcpy(g.ssid, cmd.substring(5).c_str(), sizeof(g.ssid));
    applyApConfig();
    out = "OK SSID=" + String(g.ssid) + " (STA再接続が必要)\n";
  } else if (cmd.startsWith("PASS ")) {
    strlcpy(g.pass, cmd.substring(5).c_str(), sizeof(g.pass));
    applyApConfig();
    out = strlen(g.pass) < 8 ? "OK PASS (8文字未満→認証なしAP)\n" : "OK PASS\n";
  } else if (cmd.startsWith("CHANNEL ")) {
    int v = cmd.substring(8).toInt();
    if (v >= 1 && v <= 13) { g.channel = v; applyApConfig(); out = "OK CHANNEL=" + String(v) + "\n"; }
    else out = "ERR CHANNEL 1-13\n";
  } else if (cmd.startsWith("BW ")) {
    int v = cmd.substring(3).toInt();
    if (v == 20 || v == 40) { g.bw40 = (v == 40); applyRadioConfig();
      out = "OK BW=" + String(v) + " (CSI長が変わる点に注意)\n"; }
    else out = "ERR BW 20|40\n";
  } else if (cmd.startsWith("TXPOWER ")) {
    int v = cmd.substring(8).toInt();
    if (v >= 2 && v <= 20) { g.txpower_dbm = v; applyRadioConfig(); out = "OK TXPOWER=" + String(v) + "dBm\n"; }
    else out = "ERR TXPOWER 2-20\n";
  } else if (cmd.startsWith("PROTOCOL ")) {
    String p = cmd.substring(9);
    if      (p == "B")   g.protocol = WIFI_PROTOCOL_11B;
    else if (p == "BG")  g.protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G;
    else if (p == "BGN") g.protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
    else return "ERR PROTOCOL B|BG|BGN\n";
    applyRadioConfig();
    out = "OK PROTOCOL=" + p + "\n";
  } else if (cmd.startsWith("MAXCONN ")) {
    int v = cmd.substring(8).toInt();
    if (v >= 1 && v <= 16) { g.maxconn = v; applyApConfig(); out = "OK MAXCONN=" + String(v) + "\n"; }
    else out = "ERR MAXCONN 1-16\n";
  } else if (cmd.startsWith("HIDDEN ")) {
    g.hidden = cmd.substring(7).toInt() ? 1 : 0;
    applyApConfig();
    out = "OK HIDDEN=" + String(g.hidden) + "\n";

  /* --- CSI 取得設定 --- */
  } else if (cmd.startsWith("CSILLTF "))  { g.csi_lltf  = cmd.substring(8).toInt()?1:0; applyCsiConfig(); out = "OK CSILLTF="  + String(g.csi_lltf)  + "\n"; }
  else if (cmd.startsWith("CSIHT "))      { g.csi_ht    = cmd.substring(6).toInt()?1:0; applyCsiConfig(); out = "OK CSIHT="    + String(g.csi_ht)    + "\n"; }
  else if (cmd.startsWith("CSISTBC "))    { g.csi_stbc  = cmd.substring(8).toInt()?1:0; applyCsiConfig(); out = "OK CSISTBC="  + String(g.csi_stbc)  + "\n"; }
  else if (cmd.startsWith("CSIMERGE "))   { g.csi_merge = cmd.substring(9).toInt()?1:0; applyCsiConfig(); out = "OK CSIMERGE=" + String(g.csi_merge) + "\n"; }
  else if (cmd.startsWith("CSICHFILT "))  { g.csi_chfilt= cmd.substring(10).toInt()?1:0;applyCsiConfig(); out = "OK CSICHFILT="+ String(g.csi_chfilt)+ "\n"; }
  else if (cmd.startsWith("CSISCALE "))   { g.csi_scale = cmd.substring(9).toInt()?1:0; applyCsiConfig(); out = "OK CSISCALE=" + String(g.csi_scale) + "\n"; }
  else if (cmd.startsWith("CSISHIFT ")) {
    int v = cmd.substring(9).toInt();
    if (v >= 0 && v <= 15) { g.csi_shift = v; applyCsiConfig(); out = "OK CSISHIFT=" + String(v) + "\n"; }
    else out = "ERR CSISHIFT 0-15\n";

  /* --- 出力 --- */
  } else if (cmd.startsWith("FORMAT ")) {
    String f = cmd.substring(7);
    if      (f == "RAW")   g.format = FMT_RAW;
    else if (f == "AMP")   g.format = FMT_AMP;
    else if (f == "PHASE") g.format = FMT_PHASE;
    else return "ERR FORMAT RAW|AMP|PHASE\n";
    out = "OK FORMAT=" + f + "\n";
  } else if (cmd.startsWith("MODE ")) {
    String mo = cmd.substring(5);
    if      (mo == "FULL")    g.compact = 0;
    else if (mo == "COMPACT") g.compact = 1;
    else return "ERR MODE FULL|COMPACT\n";
    out = "OK MODE=" + mo + "\n";
  } else if (cmd.startsWith("OUTPUT ")) {
    String o = cmd.substring(7);
    if      (o == "UDP")    g.out = OUT_UDP;
    else if (o == "SERIAL") g.out = OUT_SERIAL;
    else if (o == "BOTH")   g.out = OUT_UDP | OUT_SERIAL;
    else return "ERR OUTPUT UDP|SERIAL|BOTH\n";
    out = "OK OUTPUT=" + o + "\n";
  } else if (cmd.startsWith("TARGET ")) {
    strlcpy(g.target_ip, cmd.substring(7).c_str(), sizeof(g.target_ip));
    out = "OK TARGET=" + String(g.target_ip) + "\n";
  } else if (cmd.startsWith("PORT ")) {
    g.target_port = cmd.substring(5).toInt();
    out = "OK PORT=" + String(g.target_port) + "\n";
  } else if (cmd.startsWith("LLTF ")) {
    g.lltf_only = cmd.substring(5).toInt() ? 1 : 0;
    out = "OK LLTF=" + String(g.lltf_only) + "\n";
  } else if (cmd.startsWith("FILTER ")) {
    String f = cmd.substring(7);
    if (f == "OFF") { g.filter_on = 0; out = "OK FILTER=OFF\n"; }
    else if (parseMac(f.c_str(), g.filter_mac)) { g.filter_on = 1; out = "OK FILTER=" + f + "\n"; }
    else out = "ERR FILTER <AA:BB:CC:DD:EE:FF|OFF>\n";

  /* --- 時刻 --- */
  } else if (cmd.startsWith("TIME ")) {
    double t = atof(cmd.substring(5).c_str());
    if (t > 1e9) {                        // 2001 年以降ならエポックとみなす
      struct timeval tv;
      tv.tv_sec  = (time_t)t;
      tv.tv_usec = (suseconds_t)((t - (time_t)t) * 1e6);
      settimeofday(&tv, NULL);
      g_time_set = true;
      out = "OK TIME " + String(t, 6) + "\n";
    } else out = "ERR TIME <unix_epoch_sec>\n";

  /* --- 管理 --- */
  } else if (cmd == "SAVE")  { settingsSave();  out = "OK SAVE (次回起動も有効)\n"; }
  else if (cmd == "RESET") {
    settingsErase(); settingsDefault();
    applyApConfig(); applyRadioConfig(); applyCsiConfig();
    out = "OK RESET (デフォルトに復帰)\n";
  } else if (cmd == "REBOOT") { Serial.println("OK REBOOT"); delay(200); ESP.restart(); }
  else if (cmd == "HELP") {
    out = "SSID/PASS/CHANNEL/BW/TXPOWER/PROTOCOL/MAXCONN/HIDDEN | "
          "CSILLTF/CSIHT/CSISTBC/CSIMERGE/CSICHFILT/CSISCALE/CSISHIFT | "
          "FORMAT/MODE/OUTPUT/TARGET/PORT/LLTF/FILTER | "
          "TIME/SHOW/SAVE/RESET/REBOOT/HELP\n";
  } else if (cmd == "SHOW") {
    const char *fmts[] = { "RAW", "AMP", "PHASE" };
    char buf[420];
    snprintf(buf, sizeof(buf),
      "SSID=%s PASS=%s CH=%u BW=%u TXPWR=%udBm PROTO=0x%02X MAXCONN=%u HIDDEN=%u\n"
      "CSI: LLTF=%u HT=%u STBC=%u MERGE=%u CHFILT=%u SCALE=%u SHIFT=%u\n"
      "OUT: FORMAT=%s MODE=%s OUTPUT=%s%s TARGET=%s:%u LLTF_ONLY=%u FILTER=%s\n"
      "TIME_SET=%u DROPPED=%lu STA_NUM=%d\n",
      g.ssid, g.pass, g.channel, g.bw40 ? 40 : 20, g.txpower_dbm, g.protocol,
      g.maxconn, g.hidden,
      g.csi_lltf, g.csi_ht, g.csi_stbc, g.csi_merge, g.csi_chfilt, g.csi_scale, g.csi_shift,
      fmts[g.format % 3], g.compact ? "COMPACT" : "FULL",
      (g.out & OUT_UDP) ? "UDP" : "", (g.out & OUT_SERIAL) ? "+SERIAL" : "",
      g.target_ip, g.target_port, g.lltf_only, g.filter_on ? "ON" : "OFF",
      g_time_set ? 1 : 0, (unsigned long)g_dropped, WiFi.softAPgetStationNum());
    out = buf;
  } else {
    out = "ERR unknown (HELP で一覧)\n";
  }
  return out;
}

/* ======== setup / loop ======== */
void setup() {
  Serial.begin(SERIAL_BAUD);    // シリアルモニタも 921600 に合わせること
  settingsLoad();

  WiFi.mode(WIFI_AP);
  applyApConfig();
  applyRadioConfig();
  WiFi.setSleep(false);         // 省電力オフ（CSI 取得を安定させる）

  csiQueue = xQueueCreate(CSI_QUEUE_LEN, sizeof(CsiMsg));
  applyCsiConfig();             // CSI 取得開始

  udpCmd.begin(CMD_PORT);       // コマンド受付

  // FULL モードのカラム見出し（PC 側でヘッダが要る場合に利用）
  Serial.println("type,role,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,"
                 "not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,"
                 "ampdu_cnt,channel,secondary_channel,local_timestamp,ant,"
                 "sig_len,rx_state,real_time_set,real_timestamp,len,CSI_DATA");
  Serial.printf("csi_rx ready: SSID=%s ch=%d target=%s:%u\n",
                g.ssid, g.channel, g.target_ip, g.target_port);
}

void loop() {
  // (1) CSI キューを掃き出して送出
  CsiMsg m;
  while (xQueueReceive(csiQueue, &m, 0) == pdTRUE) formatAndSend(m);

  // (2) UDP コマンド
  int sz = udpCmd.parsePacket();
  if (sz > 0) {
    char buf[96];
    int n = udpCmd.read(buf, sizeof(buf) - 1);
    buf[n > 0 ? n : 0] = 0;
    String reply = processCommand(String(buf));
    udpCmd.beginPacket(udpCmd.remoteIP(), udpCmd.remotePort());
    udpCmd.print(reply);
    udpCmd.endPacket();
  }

  // (3) シリアルコマンド（シリアルモニタから入力，改行 LF/CRLF で実行）
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

  delay(1);   // 他タスク（Wi-Fi）に CPU を譲る
}
