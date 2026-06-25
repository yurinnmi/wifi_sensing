#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Ping.h>
#include <M5Unified.h>
#include <math.h>

extern "C" {
#include "esp_wifi.h"
}

/*
 * Wi-Fi設定
 * ご自身の環境に合わせて書き換えてください。
 * M5Stack Basic / ESP32 は 2.4GHz Wi-Fi を使用します。
 */
static const char* WIFI_SSID = "YOUR_WIFI_SSID";
static const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

/*
 * CSIバッファ最大長
 */
static constexpr size_t CSI_BUF_MAX = 384;
static constexpr uint16_t CSI_PAIR_MAX = CSI_BUF_MAX / 2;

/*
 * LCDグラフ設定
 */
static constexpr int GRAPH_X = 8;
static constexpr int GRAPH_Y = 120;
static constexpr int GRAPH_W = 304;
static constexpr int GRAPH_H = 100;

static constexpr int HISTORY_LEN = GRAPH_W;

/*
 * 無人状態の基準CSIを作るためのフレーム数
 */
static constexpr uint32_t BASELINE_FRAME_COUNT = 100;

/*
 * CSI最新データ保持用
 *
 * 注意:
 * CSIコールバック内では、この構造体へコピーするだけにします。
 * 解析やLCD描画は loop() 側で行います。
 */
struct CsiSnapshot
{
    bool valid;
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
    bool valid;

    int8_t real;
    int8_t imag;

    float ampMax;
    float ampAvg;

    uint16_t maxPairIndex;
    uint16_t pairCount;
};

static CsiSnapshot g_csi;
static portMUX_TYPE g_csiMux = portMUX_INITIALIZER_UNLOCKED;

/*
 * グラフ履歴
 */
static float g_scoreHistory[HISTORY_LEN];
static int g_rssiHistory[HISTORY_LEN];
static size_t g_histPos = 0;

/*
 * 基準CSI用
 */
static float g_ampVector[CSI_PAIR_MAX];
static float g_baselineSum[CSI_PAIR_MAX];
static float g_baseline[CSI_PAIR_MAX];

static bool g_baselineValid = false;
static bool g_calibrating = false;
static uint32_t g_baselineCount = 0;
static uint16_t g_baselinePairCount = 0;

/*
 * 表示更新用
 */
static uint32_t g_lastDrawMs = 0;
static uint32_t g_lastPingMs = 0;
static uint32_t g_lastCsiCount = 0;

/*
 * CSIバッファから振幅配列を作る
 *
 * ESP32のCSIバッファは基本的に
 *   imaginary, real, imaginary, real, ...
 * の順です。
 */
static uint16_t makeAmpVector(const int8_t* buf,
                              uint16_t len,
                              float* ampVector,
                              uint16_t maxPairs)
{
    if ((buf == nullptr) || (ampVector == nullptr))
    {
        return 0;
    }

    uint16_t pairCount = len / 2;
    if (pairCount > maxPairs)
    {
        pairCount = maxPairs;
    }

    for (uint16_t pair = 0; pair < pairCount; pair++)
    {
        const uint16_t index = pair * 2;

        const int8_t q = buf[index + 0];  // imaginary
        const int8_t i = buf[index + 1];  // real

        const int power =
            static_cast<int>(i) * static_cast<int>(i) +
            static_cast<int>(q) * static_cast<int>(q);

        ampVector[pair] = sqrtf(static_cast<float>(power));
    }

    return pairCount;
}

/*
 * CSI 1フレームを解析する
 *
 * LCD表示用に、
 * - 最大振幅
 * - 平均振幅
 * - 最大振幅のI/Q
 * を求めます。
 */
static CsiAnalysis analyzeCsiFrame(const CsiSnapshot& s)
{
    CsiAnalysis result = {};

    if (!s.valid || (s.len < 2))
    {
        result.valid = false;
        return result;
    }

    int bestPower = -1;
    int8_t bestReal = 0;
    int8_t bestImag = 0;
    uint16_t bestPairIndex = 0;

    float ampSum = 0.0f;
    uint16_t validPairs = 0;

    for (uint16_t byteIndex = 0; (byteIndex + 1) < s.len; byteIndex += 2)
    {
        const int8_t q = s.buf[byteIndex + 0];  // imaginary
        const int8_t i = s.buf[byteIndex + 1];  // real

        const int power =
            static_cast<int>(i) * static_cast<int>(i) +
            static_cast<int>(q) * static_cast<int>(q);

        if (power <= 0)
        {
            continue;
        }

        const uint16_t pairIndex = byteIndex / 2;

        if (power > bestPower)
        {
            bestPower = power;
            bestReal = i;
            bestImag = q;
            bestPairIndex = pairIndex;
        }

        ampSum += sqrtf(static_cast<float>(power));
        validPairs++;
    }

    if ((validPairs == 0) || (bestPower <= 0))
    {
        result.valid = false;
        return result;
    }

    result.valid = true;
    result.real = bestReal;
    result.imag = bestImag;
    result.ampMax = sqrtf(static_cast<float>(bestPower));
    result.ampAvg = ampSum / static_cast<float>(validPairs);
    result.maxPairIndex = bestPairIndex;
    result.pairCount = validPairs;

    return result;
}

/*
 * 基準CSI取得開始
 *
 * 人がいない状態でBtnBを押して実行します。
 */
static void startBaselineCalibration()
{
    for (uint16_t i = 0; i < CSI_PAIR_MAX; i++)
    {
        g_baselineSum[i] = 0.0f;
        g_baseline[i] = 0.0f;
    }

    g_baselineValid = false;
    g_calibrating = true;
    g_baselineCount = 0;
    g_baselinePairCount = 0;

    Serial.println("Baseline calibration started.");
}

/*
 * 基準CSIを更新する
 */
static void updateBaselineCalibration(const CsiSnapshot& s)
{
    uint16_t pairCount = makeAmpVector(s.buf, s.len, g_ampVector, CSI_PAIR_MAX);
    if (pairCount == 0)
    {
        return;
    }

    if (g_baselinePairCount == 0)
    {
        g_baselinePairCount = pairCount;
    }

    uint16_t usePairs = pairCount;
    if (usePairs > g_baselinePairCount)
    {
        usePairs = g_baselinePairCount;
    }

    for (uint16_t i = 0; i < usePairs; i++)
    {
        g_baselineSum[i] += g_ampVector[i];
    }

    g_baselineCount++;

    if (g_baselineCount >= BASELINE_FRAME_COUNT)
    {
        for (uint16_t i = 0; i < usePairs; i++)
        {
            g_baseline[i] = g_baselineSum[i] / static_cast<float>(g_baselineCount);
        }

        g_baselinePairCount = usePairs;
        g_baselineValid = true;
        g_calibrating = false;

        Serial.println("Baseline calibration completed.");
        /*
         * キャリブレーション完了音
         */
        M5.Speaker.tone(2000, 120);  // 2000Hzを120ms鳴らす
    }
}

/*
 * 現在CSIと基準CSIとの差分スコアを計算する
 */
static float calculateDeltaScore(const CsiSnapshot& s)
{
    if (!g_baselineValid)
    {
        return 0.0f;
    }

    uint16_t pairCount = makeAmpVector(s.buf, s.len, g_ampVector, CSI_PAIR_MAX);
    if (pairCount == 0)
    {
        return 0.0f;
    }

    uint16_t usePairs = pairCount;
    if (usePairs > g_baselinePairCount)
    {
        usePairs = g_baselinePairCount;
    }

    float diffSum = 0.0f;
    float baseSum = 0.0f;

    for (uint16_t i = 0; i < usePairs; i++)
    {
        diffSum += fabsf(g_ampVector[i] - g_baseline[i]);
        baseSum += g_baseline[i] + 1.0f;
    }

    if (baseSum <= 0.0f)
    {
        return 0.0f;
    }

    /*
     * 基準状態からの変化率っぽい値。
     * 見やすいように100倍しています。
     */
    return 100.0f * diffSum / baseSum;
}

/*
 * CSI受信コールバック
 *
 * 重要:
 * - LCD描画しない
 * - Serial大量出力しない
 * - 解析しない
 * - 最新データをコピーするだけ
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
    g_csi.rssi = info->rx_ctrl.rssi;
    g_csi.len = copyLen;
    g_csi.count++;
    memcpy(g_csi.buf, info->buf, copyLen);

    portEXIT_CRITICAL(&g_csiMux);
}

/*
 * CSIを有効化する
 */
static bool startCsi()
{
    wifi_csi_config_t csiConfig = {};

    csiConfig.lltf_en = true;
    csiConfig.htltf_en = true;
    csiConfig.stbc_htltf2_en = true;
    csiConfig.ltf_merge_en = true;
    csiConfig.channel_filter_en = false;
    csiConfig.manu_scale = false;
    csiConfig.shift = 0;

    esp_err_t err;

    err = esp_wifi_set_csi_rx_cb(csiReceiveCallback, nullptr);
    if (err != ESP_OK)
    {
        Serial.printf("esp_wifi_set_csi_rx_cb failed: %d\n", err);
        return false;
    }

    err = esp_wifi_set_csi_config(&csiConfig);
    if (err != ESP_OK)
    {
        Serial.printf("esp_wifi_set_csi_config failed: %d\n", err);
        return false;
    }

    err = esp_wifi_set_csi(true);
    if (err != ESP_OK)
    {
        Serial.printf("esp_wifi_set_csi failed: %d\n", err);
        return false;
    }

    Serial.println("CSI started.");
    return true;
}

/*
 * Wi-Fi接続
 */
static void connectWifi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    M5.Display.clear();
    M5.Display.setCursor(0, 0);
    M5.Display.println("Wi-Fi connecting...");

    Serial.printf("Connecting to %s\n", WIFI_SSID);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
        M5.Display.print(".");
    }

    Serial.println();
    Serial.println("Wi-Fi connected.");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());

    /*
     * 省電力モードを切る。
     * CSI観察では受信タイミングを安定させたいので重要。
     */
    esp_wifi_set_ps(WIFI_PS_NONE);

    /*
     * まずは20MHz幅に固定。
     */
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);

    M5.Display.clear();
    M5.Display.setCursor(0, 0);
    M5.Display.println("Wi-Fi connected");
    M5.Display.print("IP: ");
    M5.Display.println(WiFi.localIP());
    delay(1000);
}

/*
 * 履歴クリア
 */
static void clearHistory()
{
    for (size_t i = 0; i < HISTORY_LEN; i++)
    {
        g_scoreHistory[i] = 0.0f;
        g_rssiHistory[i] = -90;
    }

    g_histPos = 0;
}

/*
 * グラフ履歴へ追加
 */
static void pushHistory(float score, int rssi)
{
    g_scoreHistory[g_histPos] = score;
    g_rssiHistory[g_histPos] = rssi;

    g_histPos++;
    if (g_histPos >= HISTORY_LEN)
    {
        g_histPos = 0;
    }
}

/*
 * float折れ線グラフ描画
 */
static void drawLineGraphFloat(int x,
                               int y,
                               int w,
                               int h,
                               const float* data,
                               size_t len,
                               size_t pos,
                               float minValue,
                               float maxValue,
                               uint16_t color)
{
    if ((data == nullptr) || (len < 2))
    {
        return;
    }

    if (maxValue <= minValue)
    {
        maxValue = minValue + 1.0f;
    }

    int prevX = x;
    int prevY = y + h / 2;

    for (size_t n = 0; n < len; n++)
    {
        const size_t index = (pos + n) % len;
        float v = data[index];

        if (v < minValue)
        {
            v = minValue;
        }
        if (v > maxValue)
        {
            v = maxValue;
        }

        const float norm = (v - minValue) / (maxValue - minValue);

        const int px = x + static_cast<int>(n);
        const int py = y + h - 1 - static_cast<int>(norm * static_cast<float>(h - 1));

        if (n > 0)
        {
            M5.Display.drawLine(prevX, prevY, px, py, color);
        }

        prevX = px;
        prevY = py;
    }
}

/*
 * int折れ線グラフ描画
 */
static void drawLineGraphInt(int x,
                             int y,
                             int w,
                             int h,
                             const int* data,
                             size_t len,
                             size_t pos,
                             int minValue,
                             int maxValue,
                             uint16_t color)
{
    if ((data == nullptr) || (len < 2))
    {
        return;
    }

    if (maxValue <= minValue)
    {
        maxValue = minValue + 1;
    }

    int prevX = x;
    int prevY = y + h / 2;

    for (size_t n = 0; n < len; n++)
    {
        const size_t index = (pos + n) % len;
        int v = data[index];

        if (v < minValue)
        {
            v = minValue;
        }
        if (v > maxValue)
        {
            v = maxValue;
        }

        const float norm =
            static_cast<float>(v - minValue) /
            static_cast<float>(maxValue - minValue);

        const int px = x + static_cast<int>(n);
        const int py = y + h - 1 - static_cast<int>(norm * static_cast<float>(h - 1));

        if (n > 0)
        {
            M5.Display.drawLine(prevX, prevY, px, py, color);
        }

        prevX = px;
        prevY = py;
    }
}

/*
 * LCD画面描画
 */
static void drawScreen(const CsiSnapshot& s,
                       const CsiAnalysis& a,
                       float deltaScore)
{
    M5.Display.fillScreen(TFT_BLACK);

    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);

    M5.Display.setCursor(0, 0);
    M5.Display.println("M5Stack Wi-Fi CSI Monitor");

    M5.Display.setCursor(0, 16);
    M5.Display.printf("RSSI:%d dBm  len:%u  cnt:%lu\n",
                       static_cast<int>(s.rssi),
                       static_cast<unsigned int>(s.len),
                       static_cast<unsigned long>(s.count));

    if (a.valid)
    {
        M5.Display.setCursor(0, 34);
        M5.Display.printf("I:%4d Q:%4d Max:%5.1f\n",
                           static_cast<int>(a.real),
                           static_cast<int>(a.imag),
                           a.ampMax);

        M5.Display.setCursor(0, 50);
        M5.Display.printf("Avg:%5.1f Pair:%u/%u\n",
                           a.ampAvg,
                           static_cast<unsigned int>(a.maxPairIndex),
                           static_cast<unsigned int>(a.pairCount));
    }
    else
    {
        M5.Display.setCursor(0, 34);
        M5.Display.println("CSI analysis invalid");
    }

    M5.Display.setCursor(0, 70);

    if (g_calibrating)
    {
        M5.Display.printf("Calibrating... %lu/%lu\n",
                           static_cast<unsigned long>(g_baselineCount),
                           static_cast<unsigned long>(BASELINE_FRAME_COUNT));
    }
    else if (g_baselineValid)
    {
        M5.Display.printf("DeltaScore:%6.2f\n", deltaScore);
    }
    else
    {
        M5.Display.println("Press BtnB: baseline");
    }

    M5.Display.setCursor(0, 92);
    M5.Display.println("Graph: DeltaScore / RSSI");

    /*
     * グラフ枠
     */
    M5.Display.drawRect(GRAPH_X - 1,
                        GRAPH_Y - 1,
                        GRAPH_W + 2,
                        GRAPH_H + 2,
                        TFT_DARKGREY);

    /*
     * 中央目安線
     */
    M5.Display.drawLine(GRAPH_X,
                        GRAPH_Y + GRAPH_H / 2,
                        GRAPH_X + GRAPH_W - 1,
                        GRAPH_Y + GRAPH_H / 2,
                        TFT_DARKGREY);

    /*
     * DeltaScoreグラフ
     * 環境により値が変わるため、まずは0～30で表示。
     * 変化が大きすぎる場合は最大値を50や100に変更してください。
     */
    drawLineGraphFloat(GRAPH_X,
                       GRAPH_Y,
                       GRAPH_W,
                       GRAPH_H,
                       g_scoreHistory,
                       HISTORY_LEN,
                       g_histPos,
                       0.0f,
                       30.0f,
                       TFT_GREEN);

    /*
     * RSSIグラフ
     */
    drawLineGraphInt(GRAPH_X,
                     GRAPH_Y,
                     GRAPH_W,
                     GRAPH_H,
                     g_rssiHistory,
                     HISTORY_LEN,
                     g_histPos,
                     -90,
                     -30,
                     TFT_CYAN);

    M5.Display.setCursor(8, 224);

    M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
    M5.Display.print("Delta ");

    M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Display.print("RSSI ");

    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.print("A:Clear B:Base");
}

/*
 * CSI更新きっかけ用ping
 *
 * Wi-Fi通信が少ないとCSI更新が少ないため、
 * ゲートウェイへ定期的にpingします。
 */
static void sendPingPeriodically()
{
    const uint32_t now = millis();

    /*
     * 必要なら300を1000などに戻してもOKです。
     */
    if ((now - g_lastPingMs) < 300)
    {
        return;
    }

    g_lastPingMs = now;

    IPAddress gateway = WiFi.gatewayIP();
    if (gateway == IPAddress(0, 0, 0, 0))
    {
        return;
    }

    Ping.ping(gateway, 1);
}

void setup()
{
    auto cfg = M5.config();
    M5.begin(cfg);

    M5.Display.setRotation(1);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.fillScreen(TFT_BLACK);

    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("M5Stack Wi-Fi CSI Monitor");

    clearHistory();
    connectWifi();

    if (!startCsi())
    {
        M5.Display.clear();
        M5.Display.setCursor(0, 0);
        M5.Display.println("CSI start failed");
        M5.Display.println("Check ESP32 Arduino core");
        Serial.println("CSI start failed.");
    }
}

void loop()
{
    M5.update();

    if (M5.BtnA.wasPressed())
    {
        clearHistory();
        Serial.println("History cleared.");
    }

    if (M5.BtnB.wasPressed())
    {
        clearHistory();
        startBaselineCalibration();
    }

    sendPingPeriodically();

    CsiSnapshot local = {};

    portENTER_CRITICAL(&g_csiMux);
    local = g_csi;
    portEXIT_CRITICAL(&g_csiMux);

    if (!local.valid)
    {
        M5.Display.fillScreen(TFT_BLACK);
        M5.Display.setCursor(0, 0);
        M5.Display.println("Waiting CSI...");
        M5.Display.println();
        M5.Display.print("Wi-Fi: ");
        M5.Display.println(WiFi.isConnected() ? "connected" : "disconnected");
        M5.Display.print("RSSI: ");
        M5.Display.println(WiFi.RSSI());
        delay(300);
        return;
    }

    CsiAnalysis analysis = analyzeCsiFrame(local);

    float deltaScore = 0.0f;

    if (local.count != g_lastCsiCount)
    {
        g_lastCsiCount = local.count;

        if (g_calibrating)
        {
            updateBaselineCalibration(local);
        }
        else
        {
            deltaScore = calculateDeltaScore(local);
        }

        pushHistory(deltaScore, local.rssi);

        if ((local.count % 20) == 0)
        {
            Serial.printf("cnt=%lu rssi=%d len=%u ",
                          static_cast<unsigned long>(local.count),
                          static_cast<int>(local.rssi),
                          static_cast<unsigned int>(local.len));

            if (analysis.valid)
            {
                Serial.printf("I=%d Q=%d max=%.1f avg=%.1f score=%.2f\n",
                              static_cast<int>(analysis.real),
                              static_cast<int>(analysis.imag),
                              analysis.ampMax,
                              analysis.ampAvg,
                              deltaScore);
            }
            else
            {
                Serial.println("analysis invalid");
            }
        }
    }
    else
    {
        deltaScore = calculateDeltaScore(local);
    }

    const uint32_t now = millis();

    /*
     * LCD更新は10Hz程度
     */
    if ((now - g_lastDrawMs) >= 100)
    {
        g_lastDrawMs = now;
        drawScreen(local, analysis, deltaScore);
    }

    delay(5);
}