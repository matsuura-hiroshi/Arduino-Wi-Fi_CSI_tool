/***************************************************************************
 * csi_tx ― 自作 ESP32 CSI 送信機（STA / CSI-TX）  全設定対応版
 ***************************************************************************
 *
 * AP（csi_rx）に接続し，一定レートで UDP パケットを送るだけの装置．
 * CSI は AP 側で取得されるため本機では収集しない．
 *
 * ■ コマンドの送り方（2通り）
 *   a) UDP:    AP 接続後，PC から本機の IP（SHOW か AP のログで確認）宛て
 *              python csi_ctl.py -i 192.168.4.2 RATE 50
 *              ブロードキャストでも届く（全 TX 一斉変更に便利）:
 *              python csi_ctl.py -i 192.168.4.255 RATE 50
 *   b) シリアル: idf.py monitor からコマンドを打ち Enter．
 *              ★SSID を変えて接続できなくなった時はシリアルが復旧路★
 *
 * ■ コマンド一覧（値は例）
 *   --- 接続先 Wi-Fi ----------------------------------------------------
 *   SSID mynetwork        接続先 AP の SSID（即再接続）
 *   PASS mypassword       接続先 AP のパスワード（即再接続）
 *   --- 無線 -------------------------------------------------------------
 *   TXPOWER 20            送信電力 2-20 dBm（距離実験・電力依存性の確認用）
 *   PROTOCOL BGN          11B|BG|BGN（B にすると 11b のみ＝CSI は L-LTF のみ）
 *   --- 送信パケット ------------------------------------------------------
 *   RATE 100              送信レート 1-1000 Hz
 *   SIZE 8                ペイロード長 1-1400 バイト
 *   DSTIP 192.168.4.1     送信先 IP（通常は AP のまま）
 *   DSTPORT 5010          送信先ポート（受け手不要．CSI 発生が目的）
 *   --- 管理 --------------------------------------------------------------
 *   SHOW / SAVE / RESET / REBOOT / HELP
 *
 * ■ 使用例
 *   接続先を変える:    SSID fieldnet → PASS fieldpass → SAVE
 *   レート実験:        RATE 10 → 計測 → RATE 100 → 計測
 *   電力依存性の実験:  TXPOWER 2 → 計測 → TXPOWER 20 → 計測
 *
 * 対象: ESP32 / ESP-IDF 5.x
 ***************************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lwip/sockets.h"
#include "driver/uart.h"

/* ======== コンパイル時デフォルト（RESET 時にもこの値に戻る） ======== */
#define DEF_WIFI_SSID      "myssid"
#define DEF_WIFI_PASS      "mypassword"
#define DEF_TXPOWER_DBM    20
#define DEF_PROTOCOL       (WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N)
#define DEF_RATE_HZ        100
#define DEF_PAYLOAD_SIZE   8
#define DEF_DST_IP         "192.168.4.1"   /* softAP の既定ゲートウェイ */
#define DEF_DST_PORT       5010
#define CMD_PORT           5006

static const char *TAG = "csi_tx";
static EventGroupHandle_t s_ev;
#define CONNECTED_BIT BIT0

/* ======== 実行時設定 ======== */
typedef struct {
    char     ssid[33];
    char     pass[65];
    uint8_t  txpower_dbm;
    uint8_t  protocol;
    uint16_t rate_hz;       /* 1-1000 */
    uint16_t payload_size;  /* 1-1400 */
    char     dst_ip[16];
    uint16_t dst_port;
} settings_t;

static settings_t g_set;
static volatile uint32_t g_sent = 0;

static void settings_default(void) {
    memset(&g_set, 0, sizeof(g_set));
    strlcpy(g_set.ssid, DEF_WIFI_SSID, sizeof(g_set.ssid));
    strlcpy(g_set.pass, DEF_WIFI_PASS, sizeof(g_set.pass));
    g_set.txpower_dbm = DEF_TXPOWER_DBM;
    g_set.protocol = DEF_PROTOCOL;
    g_set.rate_hz = DEF_RATE_HZ;
    g_set.payload_size = DEF_PAYLOAD_SIZE;
    strlcpy(g_set.dst_ip, DEF_DST_IP, sizeof(g_set.dst_ip));
    g_set.dst_port = DEF_DST_PORT;
}

/* ======== NVS（設定構造体を blob で丸ごと保存） ======== */
static void settings_load(void) {
    settings_default();
    nvs_handle_t h;
    if (nvs_open("csi", NVS_READONLY, &h) != ESP_OK) return;
    size_t sz = sizeof(g_set);
    settings_t tmp;
    if (nvs_get_blob(h, "set", &tmp, &sz) == ESP_OK && sz == sizeof(g_set))
        g_set = tmp;
    nvs_close(h);
}

static void settings_save(void) {
    nvs_handle_t h;
    if (nvs_open("csi", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "set", &g_set, sizeof(g_set));
    nvs_commit(h);
    nvs_close(h);
}

static void settings_erase(void) {
    nvs_handle_t h;
    if (nvs_open("csi", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, "set");
    nvs_commit(h);
    nvs_close(h);
}

/* ======== Wi-Fi（STA） ======== */
static void apply_sta_config_and_reconnect(void) {
    wifi_config_t wc = { 0 };
    strlcpy((char *)wc.sta.ssid, g_set.ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, g_set.pass, sizeof(wc.sta.password));
    esp_wifi_disconnect();
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_connect();
}

static void apply_radio_config(void) {
    esp_wifi_set_protocol(WIFI_IF_STA, g_set.protocol);
    /* esp_wifi_set_max_tx_power の単位は 0.25 dBm */
    esp_wifi_set_max_tx_power(g_set.txpower_dbm * 4);
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_ev, CONNECTED_BIT);
        ESP_LOGW(TAG, "disconnected, retrying");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        ESP_LOGI(TAG, "got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_ev, CONNECTED_BIT);
        apply_radio_config();   /* 接続のたびに反映（電力・プロトコル） */
    }
}

static void sta_init(void) {
    s_ev = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wc = { 0 };
    strlcpy((char *)wc.sta.ssid, g_set.ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, g_set.pass, sizeof(wc.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);   /* 省電力オフ：送信間隔を安定させる */
}

/* ======== コマンド処理（UDP・シリアル共通） ======== */
static const char *HELP_TEXT =
    "SSID/PASS | TXPOWER/PROTOCOL | RATE/SIZE/DSTIP/DSTPORT | "
    "SHOW/SAVE/RESET/REBOOT/HELP\n";

static void process_command(char *buf, char *out, size_t outlen) {
    char *nl = strpbrk(buf, "\r\n");
    if (nl) *nl = 0;
    out[0] = 0;

    if (!strncmp(buf, "SSID ", 5)) {
        strlcpy(g_set.ssid, buf + 5, sizeof(g_set.ssid));
        apply_sta_config_and_reconnect();
        snprintf(out, outlen, "OK SSID=%s (再接続中)\n", g_set.ssid);
    } else if (!strncmp(buf, "PASS ", 5)) {
        strlcpy(g_set.pass, buf + 5, sizeof(g_set.pass));
        apply_sta_config_and_reconnect();
        snprintf(out, outlen, "OK PASS (再接続中)\n");
    } else if (!strncmp(buf, "TXPOWER ", 8)) {
        int v = atoi(buf + 8);
        if (v >= 2 && v <= 20) { g_set.txpower_dbm = v; apply_radio_config();
            snprintf(out, outlen, "OK TXPOWER=%ddBm\n", v); }
        else snprintf(out, outlen, "ERR TXPOWER 2-20\n");
    } else if (!strncmp(buf, "PROTOCOL ", 9)) {
        if      (!strcmp(buf + 9, "B"))   g_set.protocol = WIFI_PROTOCOL_11B;
        else if (!strcmp(buf + 9, "BG"))  g_set.protocol = WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G;
        else if (!strcmp(buf + 9, "BGN")) g_set.protocol = WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N;
        else { snprintf(out, outlen, "ERR PROTOCOL B|BG|BGN\n"); return; }
        apply_radio_config();
        snprintf(out, outlen, "OK PROTOCOL=%s\n", buf + 9);
    } else if (!strncmp(buf, "RATE ", 5)) {
        int v = atoi(buf + 5);
        if (v >= 1 && v <= 1000) { g_set.rate_hz = v;
            snprintf(out, outlen, "OK RATE=%dHz\n", v); }
        else snprintf(out, outlen, "ERR RATE 1-1000\n");
    } else if (!strncmp(buf, "SIZE ", 5)) {
        int v = atoi(buf + 5);
        if (v >= 1 && v <= 1400) { g_set.payload_size = v;
            snprintf(out, outlen, "OK SIZE=%dB\n", v); }
        else snprintf(out, outlen, "ERR SIZE 1-1400\n");
    } else if (!strncmp(buf, "DSTIP ", 6)) {
        strlcpy(g_set.dst_ip, buf + 6, sizeof(g_set.dst_ip));
        snprintf(out, outlen, "OK DSTIP=%s\n", g_set.dst_ip);
    } else if (!strncmp(buf, "DSTPORT ", 8)) {
        g_set.dst_port = atoi(buf + 8);
        snprintf(out, outlen, "OK DSTPORT=%u\n", g_set.dst_port);
    } else if (!strncmp(buf, "SAVE", 4)) {
        settings_save();
        snprintf(out, outlen, "OK SAVE (次回起動も有効)\n");
    } else if (!strncmp(buf, "RESET", 5)) {
        settings_erase();
        settings_default();
        apply_sta_config_and_reconnect();
        snprintf(out, outlen, "OK RESET (デフォルトに復帰)\n");
    } else if (!strncmp(buf, "REBOOT", 6)) {
        snprintf(out, outlen, "OK REBOOT\n");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else if (!strncmp(buf, "HELP", 4)) {
        strlcpy(out, HELP_TEXT, outlen);
    } else if (!strncmp(buf, "SHOW", 4)) {
        snprintf(out, outlen,
            "SSID=%s PASS=%s TXPWR=%udBm PROTO=0x%02X\n"
            "RATE=%uHz SIZE=%uB DST=%s:%u SENT=%lu CONNECTED=%d\n",
            g_set.ssid, g_set.pass, g_set.txpower_dbm, g_set.protocol,
            g_set.rate_hz, g_set.payload_size, g_set.dst_ip, g_set.dst_port,
            (unsigned long)g_sent,
            (xEventGroupGetBits(s_ev) & CONNECTED_BIT) ? 1 : 0);
    } else {
        snprintf(out, outlen, "ERR unknown (HELP で一覧)\n");
    }
}

/* ======== UDP コマンド受信タスク（ポート 5006，AP 接続後に有効） ======== */
static void cmd_udp_task(void *arg) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in a = { .sin_family = AF_INET,
                             .sin_port = htons(CMD_PORT),
                             .sin_addr.s_addr = htonl(INADDR_ANY) };
    bind(sock, (struct sockaddr *)&a, sizeof(a));

    char buf[96], out[320];
    struct sockaddr_in from;
    socklen_t flen = sizeof(from);
    while (1) {
        int n = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                         (struct sockaddr *)&from, &flen);
        if (n <= 0) continue;
        buf[n] = 0;
        process_command(buf, out, sizeof(out));
        sendto(sock, out, strlen(out), 0, (struct sockaddr *)&from, sizeof(from));
    }
}

/* ======== シリアルコマンド受信タスク ====================================
 * idf.py monitor からコマンドを打ち Enter．ローカルエコーされないので
 * 打った文字は見えないが，応答は表示される．
 * ★SSID/PASS を誤って AP に繋がらなくなった時の復旧はここから行う★      */
static void cmd_serial_task(void *arg) {
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    char line[96], out[320];
    int pos = 0;
    uint8_t ch;
    while (1) {
        if (uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY) < 1) continue;
        if (ch == '\r') continue;
        if (ch == '\n') {
            line[pos] = 0;
            if (pos > 0) {
                process_command(line, out, sizeof(out));
                printf("%s", out);
            }
            pos = 0;
        } else if (pos < (int)sizeof(line) - 1) {
            line[pos++] = ch;
        }
    }
}

/* ======== main：接続を待って指定レートで送信し続ける ======== */
void app_main(void) {
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    settings_load();
    sta_init();
    xTaskCreate(cmd_udp_task,    "cmd_udp", 4096, NULL, 4, NULL);
    xTaskCreate(cmd_serial_task, "cmd_ser", 4096, NULL, 4, NULL);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    static char payload[1400];
    memset(payload, 'C', sizeof(payload));   /* 中身は CSI 発生用ダミー */

    while (1) {
        /* 切断中は送信を止めて接続回復を待つ */
        xEventGroupWaitBits(s_ev, CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

        struct sockaddr_in to = { .sin_family = AF_INET,
                                  .sin_port = htons(g_set.dst_port),
                                  .sin_addr.s_addr = inet_addr(g_set.dst_ip) };
        sendto(sock, payload, g_set.payload_size, 0,
               (struct sockaddr *)&to, sizeof(to));
        g_sent++;

        /* 10 秒ごとに進捗ログ */
        if ((g_sent % (g_set.rate_hz * 10)) == 0)
            ESP_LOGI(TAG, "sent %lu packets (rate=%u Hz, size=%uB)",
                     (unsigned long)g_sent, g_set.rate_hz, g_set.payload_size);

        uint32_t delay_ms = 1000 / g_set.rate_hz;
        vTaskDelay(pdMS_TO_TICKS(delay_ms ? delay_ms : 1));
    }
}
