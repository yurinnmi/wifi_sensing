#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#include <string.h>

extern "C" {
#include "esp_wifi.h"
#include "esp_err.h"
#include "ping/ping_sock.h"
#include "lwip/ip_addr.h"
#include "lwip/ip4_addr.h"
}

/*
 * Wi-Fi設定
 * ご自身の環境に合わせて書き換えてください。
 *
 * ESP32-C6は2.4GHz Wi-Fiを使用します。
 */
//static const char* WIFI_SSID = "YOUR_WIFI_SSID";
//static const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

static const char* WIFI_SSID = "aterm-6e0352-5p";
static const char* WIFI_PASS = "071cb9f218f5c";

/*
 * シリアル設定
 */
static constexpr uint32_t SERIAL_BAUD = 921600;

/*
 * CSI設定
 *
 * ESP32-C6のCSI長は受信パケット条件により変わります。
 * ESP-IDF資料ではESP32-C6のCSI total bytesとして 128/256/384/612 などが出ます。
 * 余裕を持って768バイト確保します。
 */
static constexpr uint16_t CSI_BUF_MAX = 768;

/*
 * CSVへ出力するI/Qペア数の最大値
 *
 * ESP32-C6のCSIが612バイト程度なら pair_count=306 なので、
 * 320列あればまず足ります。
 */
static constexpr uint16_t CSI_PAIR_OUTPUT_MAX = 320;

/*
 * CSV出力周期
 *
 * 出しすぎるとシリアル帯域を圧迫するため、まずは5Hz程度。
 */
static constexpr uint32_t CSV_OUTPUT_INTERVAL_MS = 200;

/*
 * Ping周期
 *
 * ルータからの応答パケットを受けることでCSI更新のきっかけにします。
 */
static constexpr uint32_t PING_INTERVAL_MS = 200;

/*
 * CSI最新データ
 *
 * CSIコールバック内では、この構造体にコピーするだけにします。
 */
struct CsiSnapshot
{
    bool valid;
    bool firstWordInvalid;
    int8_t rssi;
    uint16_t len;
    uint32_t count;
    int8_t buf[CSI_BUF_MAX];
};

/*
 * CSI解析結果
 */
struct CsiAnalysis
{
    uint16_t pairCount;
    uint16_t outputPairCount;
    float ampAvg;
    float ampMax;
    float amp[CSI_PAIR_OUTPUT_MAX];
};

static CsiSnapshot g_csi;
static portMUX_TYPE g_csiMux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t g_lastCsvMs = 0;
static uint32_t g_lastOutputCount = 0;

static esp_ping_handle_t g_pingHandle = nullptr;

/*
 * CSVヘッダ出力
 */
static void printCsvHeader()
{
    Serial.print("ms,count,rssi,len,pair_count,amp_avg,amp_max");

    for (uint16_t i = 0; i < CSI_PAIR_OUTPUT_MAX; i++)
    {
        Serial.print(",amp");
        Serial.print(i);
    }

    Serial.println();
}

/*
 * CSI受信コールバック
 *
 * 重要:
 * - ここでSerial出力しない
 * - ここでCSV化しない
 * - ここで重い解析をしない
 * - 最新CSIをコピーするだけ
 */
static void csiReceiveCallback(void* ctx, wifi_csi_info_t* info)
{
    (void)ctx;

    if (info == nullptr)
    {
        return;
    }

    if ((info->buf == nullptr) || (info->len == 0))
    {
        return;
    }

    const uint16_t copyLen = (info->len > CSI_BUF_MAX)
                               ? CSI_BUF_MAX
                               : static_cast<uint16_t>(info->len);

    portENTER_CRITICAL(&g_csiMux);

    g_csi.valid = true;
    g_csi.firstWordInvalid = info->first_word_invalid;
    g_csi.rssi = info->rx_ctrl.rssi;
    g_csi.len = copyLen;
    g_csi.count++;
    memcpy(g_csi.buf, info->buf, copyLen);

    portEXIT_CRITICAL(&g_csiMux);
}

/*
 * CSI 1フレームを解析する
 *
 * ESP32-C6 CSI buffer:
 *   buf[0] = imaginary
 *   buf[1] = real
 *   buf[2] = imaginary
 *   buf[3] = real
 *   ...
 *
 * first_word_invalid が true の場合、先頭4バイトは無効なので、
 * amp0, amp1は空欄扱いにします。
 */
static CsiAnalysis analyzeCsi(const CsiSnapshot& s)
{
    CsiAnalysis a = {};

    for (uint16_t i = 0; i < CSI_PAIR_OUTPUT_MAX; i++)
    {
        a.amp[i] = -1.0f;  // CSVでは空欄として出す
    }

    if ((!s.valid) || (s.len < 2))
    {
        return a;
    }

    a.pairCount = s.len / 2;
    a.outputPairCount = a.pairCount;

    if (a.outputPairCount > CSI_PAIR_OUTPUT_MAX)
    {
        a.outputPairCount = CSI_PAIR_OUTPUT_MAX;
    }

    float ampSum = 0.0f;
    uint16_t validAmpCount = 0;
    float ampMax = 0.0f;

    for (uint16_t pair = 0; pair < a.outputPairCount; pair++)
    {
        /*
         * ESP32-C6では first_word_invalid=true の場合、
         * 先頭4バイト、つまり最初のI/Qペア2個分を無効扱いにする。
         */
        if (s.firstWordInvalid && (pair < 2))
        {
            continue;
        }

        const uint16_t index = pair * 2;

        if ((index + 1) >= s.len)
        {
            break;
        }

        const int8_t q = s.buf[index + 0];  // imaginary
        const int8_t i = s.buf[index + 1];  // real

        const int power =
            static_cast<int>(i) * static_cast<int>(i) +
            static_cast<int>(q) * static_cast<int>(q);

        const float amp = sqrtf(static_cast<float>(power));

        a.amp[pair] = amp;

        /*
         * 平均値はゼロ成分を少し避ける。
         * ガードバンド等のゼロ成分で平均が薄まるのを避ける目的。
         */
        if (power > 0)
        {
            ampSum += amp;
            validAmpCount++;

            if (amp > ampMax)
            {
                ampMax = amp;
            }
        }
    }

    if (validAmpCount > 0)
    {
        a.ampAvg = ampSum / static_cast<float>(validAmpCount);
        a.ampMax = ampMax;
    }

    return a;
}

/*
 * CSI解析結果をCSV 1行として出力
 */
static void printCsvRow(const CsiSnapshot& s, const CsiAnalysis& a)
{
    Serial.print(millis());
    Serial.print(',');
    Serial.print(s.count);
    Serial.print(',');
    Serial.print(static_cast<int>(s.rssi));
    Serial.print(',');
    Serial.print(s.len);
    Serial.print(',');
    Serial.print(a.pairCount);
    Serial.print(',');
    Serial.print(a.ampAvg, 3);
    Serial.print(',');
    Serial.print(a.ampMax, 3);

    for (uint16_t i = 0; i < CSI_PAIR_OUTPUT_MAX; i++)
    {
        Serial.print(',');

        if ((i < a.outputPairCount) && (a.amp[i] >= 0.0f))
        {
            Serial.print(a.amp[i], 3);
        }
        /*
         * 無効データや範囲外は空欄
         */
    }

    Serial.println();
}

/*
 * Wi-Fi接続
 */
static void connectWifi()
{
    Serial.println("# Wi-Fi connecting...");
    Serial.print("# SSID: ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print("# .");
    }

    Serial.println();
    Serial.println("# Wi-Fi connected");
    Serial.print("# IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("# Gateway: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("# RSSI: ");
    Serial.println(WiFi.RSSI());

    /*
     * CSI観察では省電力モードを切った方が安定しやすいです。
     */
    esp_err_t err = esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.print("# esp_wifi_set_ps: ");
    Serial.println(static_cast<int>(err));

    /*
     * まずはHT20に固定します。
     * ESP32-C6はHT20/HT40をサポートしますが、実験では20MHzの方が扱いやすいです。
     */
    err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
    Serial.print("# esp_wifi_set_bandwidth HT20: ");
    Serial.println(static_cast<int>(err));
}

/*
 * Pingセッション開始
 *
 * ESP-IDFのping APIで、ゲートウェイへ定期的にICMP Echoを送ります。
 * これにより、ルータからの応答パケットが入り、CSI更新のきっかけになります。
 */
static bool startPingToGateway()
{
    if (g_pingHandle != nullptr)
    {
        return true;
    }

    IPAddress gateway = WiFi.gatewayIP();

    if (gateway == IPAddress(0, 0, 0, 0))
    {
        Serial.println("# Gateway IP invalid. Ping not started.");
        return false;
    }

    ip_addr_t targetAddr;
    memset(&targetAddr, 0, sizeof(targetAddr));

    targetAddr.type = IPADDR_TYPE_V4;
    IP4_ADDR(&targetAddr.u_addr.ip4,
             gateway[0],
             gateway[1],
             gateway[2],
             gateway[3]);

    esp_ping_config_t pingConfig = ESP_PING_DEFAULT_CONFIG();
    pingConfig.target_addr = targetAddr;
    pingConfig.count = ESP_PING_COUNT_INFINITE;
    pingConfig.interval_ms = PING_INTERVAL_MS;
    pingConfig.timeout_ms = 1000;
    pingConfig.data_size = 32;

    esp_ping_callbacks_t callbacks = {};
    callbacks.on_ping_success = nullptr;
    callbacks.on_ping_timeout = nullptr;
    callbacks.on_ping_end = nullptr;
    callbacks.cb_args = nullptr;

    esp_err_t err = esp_ping_new_session(&pingConfig, &callbacks, &g_pingHandle);
    if (err != ESP_OK)
    {
        Serial.print("# esp_ping_new_session failed: ");
        Serial.println(static_cast<int>(err));
        return false;
    }

    err = esp_ping_start(g_pingHandle);
    if (err != ESP_OK)
    {
        Serial.print("# esp_ping_start failed: ");
        Serial.println(static_cast<int>(err));
        return false;
    }

    Serial.println("# Ping started.");
    return true;
}

/*
 * CSI開始
 */
static bool startCsi()
{
    wifi_csi_config_t csiConfig = {};

#if defined(CONFIG_IDF_TARGET_ESP32C6)
    /*
     * ESP32-C6 / Wi-Fi 6世代のCSI設定
     *
     * Arduino core 3.x / ESP-IDF 5.x系では wifi_csi_config_t が
     * wifi_csi_acquire_config_t 系のフィールドになります。
     */
    csiConfig.enable = 1;
    csiConfig.acquire_csi_legacy = 1;
    csiConfig.acquire_csi_ht20 = 1;
    csiConfig.acquire_csi_ht40 = 1;
    csiConfig.acquire_csi_su = 1;
    csiConfig.acquire_csi_mu = 1;
    csiConfig.acquire_csi_dcm = 1;
    csiConfig.acquire_csi_beamformed = 1;
    csiConfig.acquire_csi_he_stbc = 2;
    csiConfig.val_scale_cfg = 0;
#else
    /*
     * 従来ESP32系向け。
     * 今回のターゲットはESP32-C6なので通常こちらには入りません。
     */
    csiConfig.lltf_en = true;
    csiConfig.htltf_en = true;
    csiConfig.stbc_htltf2_en = true;
    csiConfig.ltf_merge_en = true;
    csiConfig.channel_filter_en = false;
    csiConfig.manu_scale = false;
    csiConfig.shift = 0;
#endif

    esp_err_t err;

    err = esp_wifi_set_csi_rx_cb(csiReceiveCallback, nullptr);
    if (err != ESP_OK)
    {
        Serial.print("# esp_wifi_set_csi_rx_cb failed: ");
        Serial.println(static_cast<int>(err));
        return false;
    }

    err = esp_wifi_set_csi_config(&csiConfig);
    if (err != ESP_OK)
    {
        Serial.print("# esp_wifi_set_csi_config failed: ");
        Serial.println(static_cast<int>(err));
        return false;
    }

    err = esp_wifi_set_csi(true);
    if (err != ESP_OK)
    {
        Serial.print("# esp_wifi_set_csi failed: ");
        Serial.println(static_cast<int>(err));
        return false;
    }

    Serial.println("# CSI started.");
    return true;
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(1500);

    Serial.println("# XIAO ESP32-C6 Wi-Fi CSI CSV Logger");
    Serial.println("# Build: PlatformIO + Arduino");

    connectWifi();

    if (!startCsi())
    {
        Serial.println("# CSI start failed. Stop.");
        while (true)
        {
            delay(1000);
        }
    }

    startPingToGateway();

    /*
     * CSVヘッダを出力。
     * PC側loggerは、#で始まる行をコメント扱いにし、
     * このヘッダ行からCSV保存します。
     */
    printCsvHeader();
}

void loop()
{
    CsiSnapshot local = {};

    portENTER_CRITICAL(&g_csiMux);
    local = g_csi;
    portEXIT_CRITICAL(&g_csiMux);

    const uint32_t now = millis();

    if (!local.valid)
    {
        delay(10);
        return;
    }

    /*
     * 新しいCSIがあり、出力周期を満たした場合だけCSV出力します。
     */
    if ((local.count != g_lastOutputCount) &&
        ((now - g_lastCsvMs) >= CSV_OUTPUT_INTERVAL_MS))
    {
        g_lastOutputCount = local.count;
        g_lastCsvMs = now;

        CsiAnalysis analysis = analyzeCsi(local);
        printCsvRow(local, analysis);
    }

    delay(2);
}