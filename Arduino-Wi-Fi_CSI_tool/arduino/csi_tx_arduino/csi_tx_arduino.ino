/***************************************************************************
 * csi_tx_arduino ― ESP32 CSI 送信機（STA / CSI-TX）Arduino IDE 版
 ***************************************************************************
 *
 * AP（csi_rx）に接続し，一定レートで UDP パケットを送るだけの装置．
 * CSI は AP 側で取得されるため本機では収集しない．
 *
 * ■ 動作条件
 *   - Arduino IDE ＋ ESP32 ボードパッケージ 3.x 以降，"ESP32 Dev Module"
 *   - シリアルモニタは 921600 baud
 *
 * ■ コマンドの送り方（2系統）
 *   a) UDP:    AP 接続後，PC から本機 IP 宛て（SHOW で確認）
 *              python csi_ctl.py -i 192.168.4.2 RATE 50
 *              ブロードキャストで全 TX 一斉変更も可:
 *              python csi_ctl.py -i 192.168.4.255 RATE 50
 *   b) シリアル: シリアルモニタにコマンドを入力して送信（改行 LF/CRLF）
 *              ★SSID を間違えて接続できない時はシリアルが復旧路★
 *
 * ■ コマンド一覧（値は例）
 *   SSID mynetwork      接続先 AP の SSID（即再接続）
 *   PASS mypassword     接続先 AP のパスワード（即再接続）
 *   TXPOWER 20          送信電力 2-20 dBm（距離・電力依存性実験用）
 *   PROTOCOL BGN        11B|BG|BGN（B にすると CSI は L-LTF 由来のみに）
 *   RATE 100            送信レート 1-1000 Hz
 *   SIZE 8              ペイロード長 1-1400 バイト
 *   DSTIP 192.168.4.1   送信先 IP（通常は AP のまま）
 *   DSTPORT 5010        送信先ポート（受け手不要．CSI 発生が目的）
 *   SHOW / SAVE / RESET / REBOOT / HELP
 *
 * ■ 使用例
 *   接続先変更:   SSID fieldnet → PASS fieldpass → SAVE
 *   レート実験:   RATE 10 → 計測 → RATE 100 → 計測
 *   電力実験:     TXPOWER 2 → 計測 → TXPOWER 20 → 計測
 ***************************************************************************/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include "esp_wifi.h"   // esp_wifi_set_protocol / set_max_tx_power

/* ======== コンパイル時デフォルト（RESET 時にもこの値に戻る） ======== */
#define DEF_WIFI_SSID    "myssid"
#define DEF_WIFI_PASS    "mypassword"
#define DEF_RATE_HZ      100
#define DEF_PAYLOAD      8
#define DEF_DST_IP       "192.168.4.1"   // softAP の既定ゲートウェイ
#define DEF_DST_PORT     5010
#define CMD_PORT         5006
#define SERIAL_BAUD      921600

struct Settings {
  char     ssid[33];
  char     pass[65];
  uint8_t  txpower_dbm;
  uint8_t  protocol;
  uint16_t rate_hz;
  uint16_t payload_size;
  char     dst_ip[16];
  uint16_t dst_port;
};

Settings    g;
Preferences prefs;
WiFiUDP     udpPing;    // CSI 発生用パケット送出
WiFiUDP     udpCmd;     // コマンド受付
uint32_t    g_sent = 0;
uint32_t    nextSendUs = 0;
char        payload[1400];

void settingsDefault() {
  memset(&g, 0, sizeof(g));
  strlcpy(g.ssid, DEF_WIFI_SSID, sizeof(g.ssid));
  strlcpy(g.pass, DEF_WIFI_PASS, sizeof(g.pass));
  g.txpower_dbm = 20;
  g.protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
  g.rate_hz = DEF_RATE_HZ;
  g.payload_size = DEF_PAYLOAD;
  strlcpy(g.dst_ip, DEF_DST_IP, sizeof(g.dst_ip));
  g.dst_port = DEF_DST_PORT;
}
void settingsLoad() {
  settingsDefault();
  prefs.begin("csi", true);
  if (prefs.getBytesLength("set") == sizeof(g)) prefs.getBytes("set", &g, sizeof(g));
  prefs.end();
}
void settingsSave()  { prefs.begin("csi", false); prefs.putBytes("set", &g, sizeof(g)); prefs.end(); }
void settingsErase() { prefs.begin("csi", false); prefs.remove("set"); prefs.end(); }

void applyRadioConfig() {
  esp_wifi_set_protocol(WIFI_IF_STA, g.protocol);
  esp_wifi_set_max_tx_power(g.txpower_dbm * 4);   // 単位 0.25dBm
}

void connectWifi() {
  WiFi.disconnect();
  WiFi.begin(g.ssid, g.pass);
  Serial.printf("connecting to %s ...\n", g.ssid);
}

String processCommand(String cmd) {
  cmd.trim();
  String out;

  if (cmd.startsWith("SSID ")) {
    strlcpy(g.ssid, cmd.substring(5).c_str(), sizeof(g.ssid));
    connectWifi();
    out = "OK SSID=" + String(g.ssid) + " (再接続中)\n";
  } else if (cmd.startsWith("PASS ")) {
    strlcpy(g.pass, cmd.substring(5).c_str(), sizeof(g.pass));
    connectWifi();
    out = "OK PASS (再接続中)\n";
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
  } else if (cmd.startsWith("RATE ")) {
    int v = cmd.substring(5).toInt();
    if (v >= 1 && v <= 1000) { g.rate_hz = v; out = "OK RATE=" + String(v) + "Hz\n"; }
    else out = "ERR RATE 1-1000\n";
  } else if (cmd.startsWith("SIZE ")) {
    int v = cmd.substring(5).toInt();
    if (v >= 1 && v <= 1400) { g.payload_size = v; out = "OK SIZE=" + String(v) + "B\n"; }
    else out = "ERR SIZE 1-1400\n";
  } else if (cmd.startsWith("DSTIP ")) {
    strlcpy(g.dst_ip, cmd.substring(6).c_str(), sizeof(g.dst_ip));
    out = "OK DSTIP=" + String(g.dst_ip) + "\n";
  } else if (cmd.startsWith("DSTPORT ")) {
    g.dst_port = cmd.substring(8).toInt();
    out = "OK DSTPORT=" + String(g.dst_port) + "\n";
  } else if (cmd == "SAVE")  { settingsSave();  out = "OK SAVE (次回起動も有効)\n"; }
  else if (cmd == "RESET") {
    settingsErase(); settingsDefault(); connectWifi();
    out = "OK RESET (デフォルトに復帰)\n";
  } else if (cmd == "REBOOT") { Serial.println("OK REBOOT"); delay(200); ESP.restart(); }
  else if (cmd == "HELP") {
    out = "SSID/PASS | TXPOWER/PROTOCOL | RATE/SIZE/DSTIP/DSTPORT | "
          "SHOW/SAVE/RESET/REBOOT/HELP\n";
  } else if (cmd == "SHOW") {
    char buf[240];
    snprintf(buf, sizeof(buf),
      "SSID=%s PASS=%s TXPWR=%udBm PROTO=0x%02X\n"
      "RATE=%uHz SIZE=%uB DST=%s:%u SENT=%lu CONNECTED=%d IP=%s\n",
      g.ssid, g.pass, g.txpower_dbm, g.protocol,
      g.rate_hz, g.payload_size, g.dst_ip, g.dst_port,
      (unsigned long)g_sent, WiFi.status() == WL_CONNECTED ? 1 : 0,
      WiFi.localIP().toString().c_str());
    out = buf;
  } else {
    out = "ERR unknown (HELP で一覧)\n";
  }
  return out;
}

void setup() {
  Serial.begin(SERIAL_BAUD);   // シリアルモニタも 921600 に合わせること
  settingsLoad();
  memset(payload, 'C', sizeof(payload));   // 中身は CSI 発生用ダミー

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);        // 省電力オフ：送信間隔を安定させる
  WiFi.setAutoReconnect(true); // 切断時は自動で再接続
  connectWifi();

  udpCmd.begin(CMD_PORT);
  nextSendUs = micros();
  Serial.println("csi_tx ready (HELP でコマンド一覧)");
}

void loop() {
  // (1) 接続イベント表示と無線設定の再適用
  static bool wasConnected = false;
  bool conn = (WiFi.status() == WL_CONNECTED);
  if (conn && !wasConnected) {
    Serial.printf("connected, IP=%s\n", WiFi.localIP().toString().c_str());
    applyRadioConfig();        // 接続のたびに電力・プロトコルを反映
    nextSendUs = micros();
  }
  wasConnected = conn;

  // (2) 一定レートで CSI 発生用パケットを送出（micros() ベースで等間隔）
  if (conn) {
    uint32_t now = micros();
    if ((int32_t)(now - nextSendUs) >= 0) {
      udpPing.beginPacket(g.dst_ip, g.dst_port);
      udpPing.write((const uint8_t *)payload, g.payload_size);
      udpPing.endPacket();
      g_sent++;
      nextSendUs += 1000000UL / g.rate_hz;
      // 遅延が溜まった場合は現在時刻へリセット（バースト送信を防ぐ）
      if ((int32_t)(now - nextSendUs) > 100000) nextSendUs = now;
      if (g_sent % (g.rate_hz * 10UL) == 0)
        Serial.printf("sent %lu packets (rate=%u Hz)\n",
                      (unsigned long)g_sent, g.rate_hz);
    }
  }

  // (3) UDP コマンド
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

  // (4) シリアルコマンド
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
}
