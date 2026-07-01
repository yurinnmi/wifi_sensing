# WiFi受信状態のゆらぎ測定　ESP32C6使用版

XIAO ESP32C6を使ってWi-Fi CSIという情報を取得します。


詳細はNOTEに記載しています。  

https://note.com/yuzu_monaka_/n/ne140b89c1651


# 仕様

CSIの各I/Qペアから振幅を計算し、時系列データとしてCSV保存します。  

そしてPC側のPythonで、  

- RSSIの変化

- CSI平均振幅の変化

- 基準状態との差分スコア

- サブキャリア相当の振幅ヒートマップ

を見られるようにします。


# ハードウェア

ハードウェア：XIAO ESP32C6  
開発環境：PlatformIO  
フレームワーク：Arduino  

# ライブラリインストール

pip install pandas matplotlib numpy  


# 実行

## ログ保存

python csi_serial_logger.py --port COM5 --baud 921600

ファイル名指定する場合：
python csi_serial_logger.py --port COM5 --baud 921600 --out test_csi.csv

## 保存ログ解析

python analyze_csi_csv.py test_csi.csv --baseline-sec 5 --show  

PNG保存もする場合：  

python analyze_csi_csv.py csi_log_20260630_120000.csv --baseline-sec 5 --save-plots  

# Python解析で見えるもの

## RSSIグラフ

電波全体の強さの変化を見ます。

amp_avg / amp_max では、CSI振幅が時間方向にどう変化したかを見ます。

DeltaScore では、先頭数秒の無人状態を基準にして、現在のCSI振幅配列がどれだけ変わったかを見ます。

##ヒートマップ


横軸：時間
縦軸：I/Qペア番号、サブキャリア相当
色　：振幅、または基準との差分

を表示します。  
どのI/Qペアが、いつ変化したか、を見ることができます。



# CSV各列の意味

| 列            | 意味               |
| ------------ | ---------------- |
| `ms`         | ESP32-C6起動後のミリ秒  |
| `count`      | CSI受信カウント        |
| `rssi`       | 受信パケットのRSSI      |
| `len`        | CSIバッファ長 byte    |
| `pair_count` | `len / 2`。I/Qペア数 |
| `amp_avg`    | 有効I/Qペアの振幅平均     |
| `amp_max`    | 1フレーム内の最大振幅      |
| `amp0...`    | 各I/Qペアの振幅        |



# 使い方

1. ESP32-C6とWi-Fiルータの位置を固定する  
2. Python loggerを開始する  
3. 最初の5秒は無人状態にする  
4. 人がWi-FiルータとESP32-C6の間を横切る  
5. その場で静止する  
6. 退出する  
7. Ctrl+CでCSV保存を止める  
8. analyze_csi_csv.pyで解析する  


## WiFi 

### SSDIとPassword

以下を書き換えてください。  
static const char* WIFI_SSID = "YOUR_WIFI_SSID";  
static const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";  

### 接続先

2.4GHz Wi-Fi に接続してください。  
5GHz専用SSIDには接続できません。



[注]  

ESP32-C6のCSIでは first_word_invalid がtrueの場合、先頭4バイトが無効というハードウェア制約があります。　　
コードではその場合、最初の2ペアを空欄扱いにしています。

また、ESP32-C6ではHT20/HT40の帯域幅設定があり、公式ドキュメントでも特殊な環境ではHT20固定が推奨されるケースがあると説明されています。　　
今回のコードでは実験の再現性を優先してHT20に固定しています。

CSIは環境依存が大きいです。　　　　
ルータの位置、ESP32-C6の向き、距離、壁、机、人体の位置で結果が大きく変わります。　　
今回の目的は「人検出の確定判定」ではなく、人が動いたときにWi-Fi CSIがどう変化するかを可視化することです。


CSIは環境依存が強いので、最初はM5StackとWi-Fiルータの距離を1〜3m程度にして、人がその間を横切るようにすると変化が見えやすいです。
