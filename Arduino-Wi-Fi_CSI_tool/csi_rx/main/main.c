/***************************************************************************
 * csi_rx ― 自作 ESP32 CSI 受信機（softAP / CSI-RX）  全設定対応版
 ***************************************************************************
 *
 * ■ 設定の三層構造
 *   (1) sdkconfig.defaults   … ビルド時に自動取込（menuconfig 不要）
 *   (2) 本ファイル冒頭の DEF_xxx … コンパイル時デフォルト
 *   (3) 実行時コマンド        … UDP（ポート5006）またはシリアルから変更．
 *                               SAVE で NVS に保存され，次回起動も有効．
 *
 * ■ コマンドの送り方（2通り，どちらも同じコマンドが使える）
 *   a) UDP:    python csi_ctl.py SHOW
 *              python csi_ctl.py CHANNEL 11
 *      （csi_ctl.py は 192.168.4.1:5006 宛てに送信し，応答を表示する）
 *   b) シリアル: idf.py monitor を開いてコマンドを打ち Enter
 *      （Wi-Fi 設定を壊して UDP が届かなくなった時の復旧路としても使える）
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
 *   CSILLTF 1             L-LTF からの CSI 取得      0|1
 *   CSIHT 1               HT-LTF からの CSI 取得     0|1
 *   CSISTBC 1             STBC HT-LTF2 からの取得    0|1
 *   CSIMERGE 1            隣接サブキャリア平均(LTF統合) 0|1
 *   CSICHFILT 0           チャネルフィルタ           0|1
 *   CSISCALE 0            手動スケーリング           0|1
 *   CSISHIFT 0            手動スケール時のシフト量   0-15
 *   --- 出力設定 -------------------------------------------------------
 *   FORMAT RAW            RAW(生I/Q)|AMP(振幅)|PHASE(位相)
 *   MODE FULL             FULL(26カラム Hernandez互換)|COMPACT(5カラム)
 *   OUTPUT UDP            UDP|SERIAL|BOTH
 *   TARGET 192.168.4.255  UDP 送出先 IP（255 終わりでブロードキャスト）
 *   PORT 5005             UDP 送出先ポート
 *   LLTF 1                1: 配列を先頭128値(L-LTF分)に制限 / 0: 全部
 *   FILTER 2C:BC:BB:83:1E:D8   この MAC の CSI だけ出力（解除は FILTER OFF）
 *   --- 時刻 -----------------------------------------------------------
 *   TIME 1765346400.123   UNIX 秒で実時刻を設定．
 *                         PC 時刻を送るなら → python csi_ctl.py time
 *                         設定後 real_time_set=1，real_timestamp がエポックに．
 *   --- 管理 -----------------------------------------------------------
 *   SHOW                  現在の全設定とドロップ数を表示
 *   SAVE                  NVS へ保存（保存しないと再起動で消える）
 *   RESET                 コンパイル時デフォルトへ戻す（NVS も消去）
 *   REBOOT                再起動
 *   HELP                  コマンド一覧
 *
 * ■ 使用例（典型的な実験開始手順）
 *   1. idf.py build flash      … 書き込み
 *   2. PC を本機の AP に接続
 *   3. python csi_ctl.py time  … 時刻同期
 *   4. python csi_ctl.py SHOW  … 設定確認
 *   5. python udp_logging.py   … 収集開始（FORMAT RAW + MODE FULL なら
 *                                 既存の Hernandez 用スクリプトがそのまま動く）
 *
 * 対象: ESP32 / ESP-IDF 5.x
 ***************************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lwip/sockets.h"
#include "driver/uart.h"

/* ======== コンパイル時デフォルト（RESET 時にもこの値に戻る） ======== */
#define DEF_AP_SSID        "myssid"
#define DEF_AP_PASS        "mypassword"
#define DEF_WIFI_CHANNEL   6
#define DEF_BW40           0        /* 0:HT20 1:HT40 */
#define DEF_TXPOWER_DBM    20       /* 2-20 dBm */
#define DEF_PROTOCOL       (WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N)
#define DEF_MAXCONN        16
#define DEF_HIDDEN         0
#define DEF_TARGET_IP      "192.168.4.255"
#define DEF_TARGET_PORT    5005
#define DEF_LLTF_ONLY      1
#define CMD_PORT           5006     /* コマンド受付ポート（固定） */
#define CSI_QUEUE_LEN      24

/* 列挙値 */
#define FMT_RAW    0
#define FMT_AMP    1
#define FMT_PHASE  2
#define OUT_UDP    0x01
#define OUT_SERIAL 0x02

static const char *TAG = "csi_rx";

/* ======== 実行時設定（SHOW で表示される項目はすべてここ） ======== */
typedef struct {
    /* Wi-Fi */
    char     ssid[33];
    char     pass[65];
    uint8_t  channel;
    uint8_t  bw40;        /* 0:20MHz 1:40MHz */
    uint8_t  txpower_dbm; /* 2-20 */
    uint8_t  protocol;    /* WIFI_PROTOCOL_* のビット和 */
    uint8_t  maxconn;
    uint8_t  hidden;
    /* CSI 取得（wifi_csi_config_t 対応） */
    uint8_t  csi_lltf, csi_ht, csi_stbc, csi_merge, csi_chfilt, csi_scale, csi_shift;
    /* 出力 */
    uint8_t  format;      /* FMT_* */
    uint8_t  compact;     /* 0:FULL 1:COMPACT */
    uint8_t  out;         /* OUT_* ビット和 */
    char     target_ip[16];
    uint16_t target_port;
    uint8_t  lltf_only;
    uint8_t  filter_on;
    uint8_t  filter_mac[6];
} settings_t;

static settings_t g_set;
static int g_udp_sock = -1;
static struct sockaddr_in g_udp_addr;
static uint32_t g_dropped = 0;
static volatile uint8_t g_time_set = 0;

/* ======== デフォルト値 ======== */
static void settings_default(void) {
    memset(&g_set, 0, sizeof(g_set));
    strlcpy(g_set.ssid, DEF_AP_SSID, sizeof(g_set.ssid));
    strlcpy(g_set.pass, DEF_AP_PASS, sizeof(g_set.pass));
    g_set.channel = DEF_WIFI_CHANNEL;
    g_set.bw40 = DEF_BW40;
    g_set.txpower_dbm = DEF_TXPOWER_DBM;
    g_set.protocol = DEF_PROTOCOL;
    g_set.maxconn = DEF_MAXCONN;
    g_set.hidden = DEF_HIDDEN;
    g_set.csi_lltf = 1; g_set.csi_ht = 1; g_set.csi_stbc = 1;
    g_set.csi_merge = 1; g_set.csi_chfilt = 0; g_set.csi_scale = 0; g_set.csi_shift = 0;
    g_set.format = FMT_RAW;
    g_set.compact = 0;
    g_set.out = OUT_UDP;
    strlcpy(g_set.target_ip, DEF_TARGET_IP, sizeof(g_set.target_ip));
    g_set.target_port = DEF_TARGET_PORT;
    g_set.lltf_only = DEF_LLTF_ONLY;
}

/* ======== NVS 入出力（設定構造体を blob で丸ごと保存） ======== */
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

static void udp_target_update(void) {
    memset(&g_udp_addr, 0, sizeof(g_udp_addr));
    g_udp_addr.sin_family = AF_INET;
    g_udp_addr.sin_port = htons(g_set.target_port);
    g_udp_addr.sin_addr.s_addr = inet_addr(g_set.target_ip);
}

/* ======== Wi-Fi 設定の適用（SSID/PASS/CHANNEL 等の変更時に呼ぶ） ======== */
static void apply_ap_config(void) {
    wifi_config_t wc = { 0 };
    strlcpy((char *)wc.ap.ssid, g_set.ssid, sizeof(wc.ap.ssid));
    strlcpy((char *)wc.ap.password, g_set.pass, sizeof(wc.ap.password));
    wc.ap.ssid_len = strlen(g_set.ssid);
    wc.ap.channel = g_set.channel;
    wc.ap.max_connection = g_set.maxconn;
    wc.ap.ssid_hidden = g_set.hidden;
    /* パスワード 8 文字未満は WPA2 にできないため OPEN にする */
    wc.ap.authmode = (strlen(g_set.pass) >= 8) ? WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;
    esp_wifi_set_config(WIFI_IF_AP, &wc);
}

static void apply_radio_config(void) {
    esp_wifi_set_protocol(WIFI_IF_AP, g_set.protocol);
    esp_wifi_set_bandwidth(WIFI_IF_AP, g_set.bw40 ? WIFI_BW_HT40 : WIFI_BW_HT20);
    /* esp_wifi_set_max_tx_power の単位は 0.25 dBm（範囲 8-84 = 2-21 dBm） */
    esp_wifi_set_max_tx_power(g_set.txpower_dbm * 4);
}

/* ======== CSI 取得設定の適用（CSIxxx コマンド変更時に呼ぶ） ======== */
static void apply_csi_config(void) {
    esp_wifi_set_csi(0);    /* いったん停止してから設定し直すのが安全 */
    wifi_csi_config_t c = {
        .lltf_en           = g_set.csi_lltf,
        .htltf_en          = g_set.csi_ht,
        .stbc_htltf2_en    = g_set.csi_stbc,
        .ltf_merge_en      = g_set.csi_merge,
        .channel_filter_en = g_set.csi_chfilt,
        .manu_scale        = g_set.csi_scale,
        .shift             = g_set.csi_shift,
    };
    esp_wifi_set_csi_config(&c);
    esp_wifi_set_csi(1);
}

/* ======== CSI: コールバック → キュー → 送信タスク ======== */
typedef struct {
    wifi_pkt_rx_ctrl_t rx_ctrl;
    uint8_t  mac[6];
    uint16_t len;
    int8_t   buf[384];
} csi_msg_t;

static QueueHandle_t g_csi_q;

/* Wi-Fi タスク内で呼ばれるため最小限のコピーのみ行う */
static void csi_cb(void *ctx, wifi_csi_info_t *info) {
    if (g_set.filter_on && memcmp(info->mac, g_set.filter_mac, 6) != 0)
        return;                                  /* MAC フィルタ */
    csi_msg_t m;
    m.rx_ctrl = info->rx_ctrl;
    memcpy(m.mac, info->mac, 6);
    m.len = info->len > sizeof(m.buf) ? sizeof(m.buf) : info->len;
    memcpy(m.buf, info->buf, m.len);
    if (xQueueSend(g_csi_q, &m, 0) != pdTRUE) g_dropped++;   /* 満杯時は破棄 */
}

/* CSV 行の整形と UDP/シリアルへの送出を担う */
static void csi_sender_task(void *arg) {
    static char line[3200];
    csi_msg_t m;

    while (1) {
        if (xQueueReceive(g_csi_q, &m, portMAX_DELAY) != pdTRUE) continue;

        /* 実時刻：TIME 設定済みなら UNIX エポック，未設定なら起動からの秒 */
        double ts;
        if (g_time_set) {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            ts = tv.tv_sec + tv.tv_usec / 1e6;
        } else {
            ts = esp_timer_get_time() / 1e6;
        }

        int n;
        if (g_set.compact) {
            /* COMPACT: CSI_DATA,MAC,RSSI,時刻,len,[...] の 5+1 カラム */
            n = snprintf(line, sizeof(line),
                "CSI_DATA,%02X:%02X:%02X:%02X:%02X:%02X,%d,%.6f,%u,[",
                m.mac[0], m.mac[1], m.mac[2], m.mac[3], m.mac[4], m.mac[5],
                m.rx_ctrl.rssi, ts, m.len);
        } else {
            /* FULL: Hernandez 互換 26 カラム（既存 PC スクリプト用） */
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
                g_time_set, ts, m.len);
        }

        int data_len = g_set.lltf_only ? (m.len < 128 ? m.len : 128) : m.len;

        switch (g_set.format) {
        case FMT_AMP:      /* 振幅 = sqrt(I^2+Q^2)，buf は [imag, real] 順 */
            for (int i = 0; i < data_len / 2 && n < (int)sizeof(line) - 12; i++)
                n += snprintf(line + n, sizeof(line) - n, "%.1f ",
                              sqrtf((float)(m.buf[2*i]*m.buf[2*i] +
                                            m.buf[2*i+1]*m.buf[2*i+1])));
            break;
        case FMT_PHASE:    /* 位相 = atan2(imag, real) [rad] */
            for (int i = 0; i < data_len / 2 && n < (int)sizeof(line) - 12; i++)
                n += snprintf(line + n, sizeof(line) - n, "%.3f ",
                              atan2f(m.buf[2*i], m.buf[2*i+1]));
            break;
        default:           /* RAW: I/Q 生値（Hernandez 互換） */
            for (int i = 0; i < data_len && n < (int)sizeof(line) - 8; i++)
                n += snprintf(line + n, sizeof(line) - n, "%d ", m.buf[i]);
            break;
        }
        n += snprintf(line + n, sizeof(line) - n, "]\n");

        if (g_set.out & OUT_UDP)
            sendto(g_udp_sock, line, n, 0,
                   (struct sockaddr *)&g_udp_addr, sizeof(g_udp_addr));
        if (g_set.out & OUT_SERIAL)
            fwrite(line, 1, n, stdout);
    }
}

/* ======== softAP ======== */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data) {
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "STA " MACSTR " joined", MAC2STR(e->mac));
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGI(TAG, "STA " MACSTR " left", MAC2STR(e->mac));
    }
}

static void softap_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    apply_ap_config();
    ESP_ERROR_CHECK(esp_wifi_start());
    apply_radio_config();           /* start 後でないと反映されない項目がある */
    esp_wifi_set_ps(WIFI_PS_NONE);  /* 省電力オフ（CSI 取得を安定させる） */
    ESP_LOGI(TAG, "softAP up: SSID=%s ch=%d", g_set.ssid, g_set.channel);
}

/* ======== コマンド処理（UDP・シリアル共通） ======== */
static int parse_mac(const char *s, uint8_t mac[6]) {
    unsigned v[6];
    if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) != 6)
        return -1;
    for (int i = 0; i < 6; i++) mac[i] = v[i];
    return 0;
}

static const char *HELP_TEXT =
    "SSID/PASS/CHANNEL/BW/TXPOWER/PROTOCOL/MAXCONN/HIDDEN | "
    "CSILLTF/CSIHT/CSISTBC/CSIMERGE/CSICHFILT/CSISCALE/CSISHIFT | "
    "FORMAT/MODE/OUTPUT/TARGET/PORT/LLTF/FILTER | "
    "TIME/SHOW/SAVE/RESET/REBOOT/HELP\n";

/* 1 行のコマンドを処理し out に応答文字列を返す */
static void process_command(char *buf, char *out, size_t outlen) {
    char *nl = strpbrk(buf, "\r\n");
    if (nl) *nl = 0;
    out[0] = 0;

    /* --- Wi-Fi --- */
    if (!strncmp(buf, "SSID ", 5)) {
        strlcpy(g_set.ssid, buf + 5, sizeof(g_set.ssid));
        apply_ap_config();
        snprintf(out, outlen, "OK SSID=%s (STA再接続が必要)\n", g_set.ssid);
    } else if (!strncmp(buf, "PASS ", 5)) {
        strlcpy(g_set.pass, buf + 5, sizeof(g_set.pass));
        apply_ap_config();
        snprintf(out, outlen, "OK PASS%s\n",
                 strlen(g_set.pass) < 8 ? " (8文字未満→認証なしAP)" : "");
    } else if (!strncmp(buf, "CHANNEL ", 8)) {
        int v = atoi(buf + 8);
        if (v >= 1 && v <= 13) { g_set.channel = v; apply_ap_config();
            snprintf(out, outlen, "OK CHANNEL=%d\n", v); }
        else snprintf(out, outlen, "ERR CHANNEL 1-13\n");
    } else if (!strncmp(buf, "BW ", 3)) {
        int v = atoi(buf + 3);
        if (v == 20 || v == 40) { g_set.bw40 = (v == 40); apply_radio_config();
            snprintf(out, outlen, "OK BW=%d (CSI長が変わる点に注意)\n", v); }
        else snprintf(out, outlen, "ERR BW 20|40\n");
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
    } else if (!strncmp(buf, "MAXCONN ", 8)) {
        int v = atoi(buf + 8);
        if (v >= 1 && v <= 16) { g_set.maxconn = v; apply_ap_config();
            snprintf(out, outlen, "OK MAXCONN=%d\n", v); }
        else snprintf(out, outlen, "ERR MAXCONN 1-16\n");
    } else if (!strncmp(buf, "HIDDEN ", 7)) {
        g_set.hidden = atoi(buf + 7) ? 1 : 0;
        apply_ap_config();
        snprintf(out, outlen, "OK HIDDEN=%d\n", g_set.hidden);

    /* --- CSI 取得設定 --- */
    } else if (!strncmp(buf, "CSILLTF ", 8))  { g_set.csi_lltf  = atoi(buf+8)?1:0; apply_csi_config(); snprintf(out, outlen, "OK CSILLTF=%d\n",  g_set.csi_lltf); }
    else if (!strncmp(buf, "CSIHT ", 6))      { g_set.csi_ht    = atoi(buf+6)?1:0; apply_csi_config(); snprintf(out, outlen, "OK CSIHT=%d\n",    g_set.csi_ht); }
    else if (!strncmp(buf, "CSISTBC ", 8))    { g_set.csi_stbc  = atoi(buf+8)?1:0; apply_csi_config(); snprintf(out, outlen, "OK CSISTBC=%d\n",  g_set.csi_stbc); }
    else if (!strncmp(buf, "CSIMERGE ", 9))   { g_set.csi_merge = atoi(buf+9)?1:0; apply_csi_config(); snprintf(out, outlen, "OK CSIMERGE=%d\n", g_set.csi_merge); }
    else if (!strncmp(buf, "CSICHFILT ", 10)) { g_set.csi_chfilt= atoi(buf+10)?1:0;apply_csi_config(); snprintf(out, outlen, "OK CSICHFILT=%d\n",g_set.csi_chfilt); }
    else if (!strncmp(buf, "CSISCALE ", 9))   { g_set.csi_scale = atoi(buf+9)?1:0; apply_csi_config(); snprintf(out, outlen, "OK CSISCALE=%d\n", g_set.csi_scale); }
    else if (!strncmp(buf, "CSISHIFT ", 9)) {
        int v = atoi(buf + 9);
        if (v >= 0 && v <= 15) { g_set.csi_shift = v; apply_csi_config();
            snprintf(out, outlen, "OK CSISHIFT=%d\n", v); }
        else snprintf(out, outlen, "ERR CSISHIFT 0-15\n");

    /* --- 出力 --- */
    } else if (!strncmp(buf, "FORMAT ", 7)) {
        if      (!strcmp(buf + 7, "RAW"))   g_set.format = FMT_RAW;
        else if (!strcmp(buf + 7, "AMP"))   g_set.format = FMT_AMP;
        else if (!strcmp(buf + 7, "PHASE")) g_set.format = FMT_PHASE;
        else { snprintf(out, outlen, "ERR FORMAT RAW|AMP|PHASE\n"); return; }
        snprintf(out, outlen, "OK FORMAT=%s\n", buf + 7);
    } else if (!strncmp(buf, "MODE ", 5)) {
        if      (!strcmp(buf + 5, "FULL"))    g_set.compact = 0;
        else if (!strcmp(buf + 5, "COMPACT")) g_set.compact = 1;
        else { snprintf(out, outlen, "ERR MODE FULL|COMPACT\n"); return; }
        snprintf(out, outlen, "OK MODE=%s\n", buf + 5);
    } else if (!strncmp(buf, "OUTPUT ", 7)) {
        if      (!strcmp(buf + 7, "UDP"))    g_set.out = OUT_UDP;
        else if (!strcmp(buf + 7, "SERIAL")) g_set.out = OUT_SERIAL;
        else if (!strcmp(buf + 7, "BOTH"))   g_set.out = OUT_UDP | OUT_SERIAL;
        else { snprintf(out, outlen, "ERR OUTPUT UDP|SERIAL|BOTH\n"); return; }
        snprintf(out, outlen, "OK OUTPUT=%s\n", buf + 7);
    } else if (!strncmp(buf, "TARGET ", 7)) {
        strlcpy(g_set.target_ip, buf + 7, sizeof(g_set.target_ip));
        udp_target_update();
        snprintf(out, outlen, "OK TARGET=%s\n", g_set.target_ip);
    } else if (!strncmp(buf, "PORT ", 5)) {
        g_set.target_port = atoi(buf + 5);
        udp_target_update();
        snprintf(out, outlen, "OK PORT=%u\n", g_set.target_port);
    } else if (!strncmp(buf, "LLTF ", 5)) {
        g_set.lltf_only = atoi(buf + 5) ? 1 : 0;
        snprintf(out, outlen, "OK LLTF=%d\n", g_set.lltf_only);
    } else if (!strncmp(buf, "FILTER ", 7)) {
        if (!strcmp(buf + 7, "OFF")) {
            g_set.filter_on = 0;
            snprintf(out, outlen, "OK FILTER=OFF\n");
        } else if (parse_mac(buf + 7, g_set.filter_mac) == 0) {
            g_set.filter_on = 1;
            snprintf(out, outlen, "OK FILTER=%s\n", buf + 7);
        } else snprintf(out, outlen, "ERR FILTER <AA:BB:CC:DD:EE:FF|OFF>\n");

    /* --- 時刻 --- */
    } else if (!strncmp(buf, "TIME ", 5)) {
        double t = atof(buf + 5);
        if (t > 1e9) {              /* 2001年以降の値ならエポックとみなす */
            struct timeval tv = {
                .tv_sec  = (time_t)t,
                .tv_usec = (suseconds_t)((t - (time_t)t) * 1e6),
            };
            settimeofday(&tv, NULL);
            g_time_set = 1;
            snprintf(out, outlen, "OK TIME %.6f\n", t);
        } else snprintf(out, outlen, "ERR TIME <unix_epoch_sec>\n");

    /* --- 管理 --- */
    } else if (!strncmp(buf, "SAVE", 4)) {
        settings_save();
        snprintf(out, outlen, "OK SAVE (次回起動も有効)\n");
    } else if (!strncmp(buf, "RESET", 5)) {
        settings_erase();
        settings_default();
        udp_target_update();
        apply_ap_config();
        apply_radio_config();
        apply_csi_config();
        snprintf(out, outlen, "OK RESET (デフォルトに復帰)\n");
    } else if (!strncmp(buf, "REBOOT", 6)) {
        snprintf(out, outlen, "OK REBOOT\n");
        /* 応答は呼び出し元で送られないため即時再起動でよい場面のみ使用 */
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else if (!strncmp(buf, "HELP", 4)) {
        strlcpy(out, HELP_TEXT, outlen);
    } else if (!strncmp(buf, "SHOW", 4)) {
        static const char *fmts[] = { "RAW", "AMP", "PHASE" };
        snprintf(out, outlen,
            "SSID=%s PASS=%s CH=%u BW=%u TXPWR=%udBm PROTO=0x%02X MAXCONN=%u HIDDEN=%u\n"
            "CSI: LLTF=%u HT=%u STBC=%u MERGE=%u CHFILT=%u SCALE=%u SHIFT=%u\n"
            "OUT: FORMAT=%s MODE=%s OUTPUT=%s%s TARGET=%s:%u LLTF_ONLY=%u FILTER=%s\n"
            "TIME_SET=%u DROPPED=%lu\n",
            g_set.ssid, g_set.pass, g_set.channel, g_set.bw40 ? 40 : 20,
            g_set.txpower_dbm, g_set.protocol, g_set.maxconn, g_set.hidden,
            g_set.csi_lltf, g_set.csi_ht, g_set.csi_stbc, g_set.csi_merge,
            g_set.csi_chfilt, g_set.csi_scale, g_set.csi_shift,
            fmts[g_set.format % 3], g_set.compact ? "COMPACT" : "FULL",
            (g_set.out & OUT_UDP) ? "UDP" : "",
            (g_set.out & OUT_SERIAL) ? "+SERIAL" : "",
            g_set.target_ip, g_set.target_port, g_set.lltf_only,
            g_set.filter_on ? "ON" : "OFF",
            g_time_set, (unsigned long)g_dropped);
    } else {
        snprintf(out, outlen, "ERR unknown (HELP で一覧)\n");
    }
}

/* ======== UDP コマンド受信タスク（ポート 5006） ======== */
static void cmd_udp_task(void *arg) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in a = { .sin_family = AF_INET,
                             .sin_port = htons(CMD_PORT),
                             .sin_addr.s_addr = htonl(INADDR_ANY) };
    bind(sock, (struct sockaddr *)&a, sizeof(a));
    ESP_LOGI(TAG, "command port %d ready (UDP)", CMD_PORT);

    char buf[96], out[512];
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

/* ======== シリアルコマンド受信タスク（idf.py monitor から直接打てる） ====
 * 注意: monitor はローカルエコーしないため，打った文字は画面に出ない．
 *       Enter で実行され応答だけが表示される．                            */
static void cmd_serial_task(void *arg) {
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    char line[96], out[512];
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

/* ======== main ======== */
void app_main(void) {
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    settings_load();      /* NVS に保存があればそれを，なければデフォルト */

    g_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int bc = 1;
    setsockopt(g_udp_sock, SOL_SOCKET, SO_BROADCAST, &bc, sizeof(bc));
    udp_target_update();

    softap_init();

    g_csi_q = xQueueCreate(CSI_QUEUE_LEN, sizeof(csi_msg_t));
    xTaskCreatePinnedToCore(csi_sender_task, "csi_send", 4096, NULL, 5, NULL, 1);
    xTaskCreate(cmd_udp_task,    "cmd_udp", 4096, NULL, 4, NULL);
    xTaskCreate(cmd_serial_task, "cmd_ser", 4096, NULL, 4, NULL);

    apply_csi_config();   /* CSI 取得開始 */

    /* FULL モードのカラム見出し（PC 側でヘッダが要る場合に利用） */
    printf("type,role,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,"
           "not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,"
           "ampdu_cnt,channel,secondary_channel,local_timestamp,ant,"
           "sig_len,rx_state,real_time_set,real_timestamp,len,CSI_DATA\n");
    ESP_LOGI(TAG, "csi_rx ready: target=%s:%u", g_set.target_ip, g_set.target_port);
}
