# ESP32 GPS Tracker with MQTT

基於 ESP32 的 GPS 追蹤器，透過 MQTT 協定將位置資料發布到 OwnTracks 伺服器。支援多衛星系統（GPS、GLONASS、Galileo、BeiDou）快速定位，具備智慧移動偵測、批次上傳與網頁設定介面。

## 主要功能

- 🛰️ **多衛星系統支援**：同時支援 GPS、GLONASS、Galileo、北斗四大導航系統
- 📍 **智慧移動偵測**：基於距離、時間、方向角變化觸發上傳
- 📦 **雙緩衝批次上傳**：上傳與資料累積同步進行，不互相阻塞
- 💤 **自動休眠**：靜止超過設定時間自動暫停上傳，節省流量
- 🔄 **自動重連**：WiFi/MQTT 斷線自動重連
- 🌐 **網頁設定介面**：透過瀏覽器即可修改所有參數，無需重新燒錄
- 📡 **AP 模式備援**：WiFi 連線失敗自動切換為基地台模式供設定
- 💡 **視覺回饋**：LED 指示燈顯示 GPS 定位狀態
- 🛡️ **資料驗證**：衛星數量門檻過濾，確保資料品質
- 💾 **持久化儲存**：設定儲存於 Flash，斷電不遺失

## 硬體需求

### 必要元件
- **ESP32 開發板**（任何 ESP32 系列均可）
- **GPS 模組**（建議：支援多 GNSS 的模組，如 NEO-6M/NEO-M8N 或更新版本）
- **連接線材**

### 接線方式

| GPS 模組 | ESP32 | 說明 |
|---------|-------|------|
| VCC     | 3.3V/5V | 電源（依模組規格） |
| GND     | GND   | 接地 |
| TX      | GPIO 16 (RX) | GPS 傳送 → ESP32 接收 |
| RX      | GPIO 17 (TX) | ESP32 傳送 → GPS 接收 |

LED（內建）：GPIO 2

## 軟體需求

### Arduino IDE 環境設定

1. **安裝 ESP32 開發板支援**
   ```
   開發板管理員 → 搜尋 "ESP32" → 安裝 "esp32 by Espressif Systems"
   ```

2. **安裝必要函式庫**（透過 Arduino Library Manager）
   - `TinyGPSPlus` by Mikal Hart
   - `PubSubClient` by Nick O'Leary

### 使用 arduino-cli 編譯

```bash
# 安裝 ESP32 核心
arduino-cli core install esp32:esp32

# 編譯專案
arduino-cli compile --fqbn esp32:esp32:esp32 gps_v20250725.ino

# 上傳到開發板（替換為實際序列埠）
arduino-cli upload -p /dev/cu.usbserial-* --fqbn esp32:esp32:esp32 gps_v20250725.ino
```

## 安裝與設定

### 方法 1：首次使用（推薦 - 使用網頁設定）

1. **複製並編輯配置檔**（僅設定 MQTT 伺服器資訊）
   ```bash
   cp config.h.example config.h
   ```
   
   編輯 `config.h`，設定 MQTT 伺服器：
   ```cpp
   #define MQTT_HOST   "您的MQTT伺服器IP"
   #define MQTT_PORT   50883
   #define MQTT_USER   "MQTT帳號"
   #define MQTT_PASS   "MQTT密碼"
   #define USER_ID     "使用者ID"
   #define DEVICE_ID   "裝置ID"
   ```

2. **上傳程式到 ESP32**

3. **裝置會自動進入 AP 模式**（因為尚未設定 WiFi）
   ```
   📡 WiFi 連線逾時，切換為 AP 模式
   
   📱 請使用手機/電腦連線至：
      SSID: GPS-Tracker-XXXXXXXX
      密碼: 12345678
   
   🌐 然後開啟瀏覽器訪問：
      http://192.168.4.1
   ```

4. **透過網頁設定 WiFi 與參數**
   - 連線至 `GPS-Tracker-XXXXXXXX` 網路
   - 開啟瀏覽器訪問 `http://192.168.4.1`
   - 設定 WiFi SSID、密碼及其他參數
   - 點擊「💾 儲存設定」
   - 重新啟動裝置

5. **完成！**裝置會自動連線至設定的 WiFi 並開始追蹤

### 方法 2：傳統方法（直接寫入配置檔）

1. **複製並完整編輯配置檔**
   ```bash
   cp config.h.example config.h
   ```
   
   編輯 `config.h`，填入所有參數：
   ```cpp
   #define WIFI_SSID   "您的WiFi名稱"
   #define WIFI_PWD    "您的WiFi密碼"
   #define MQTT_HOST   "您的MQTT伺服器IP"
   #define MQTT_PORT   50883
   #define MQTT_USER   "MQTT帳號"
   #define MQTT_PASS   "MQTT密碼"
   #define USER_ID     "使用者ID"
   #define DEVICE_ID   "裝置ID"
   ```

2. **上傳程式到 ESP32**

3. **開啟串列埠監視器（115200 baud）**觀察運作狀態

## 網頁設定介面

### 存取方式

#### 正常模式（已連線至 WiFi）
裝置連線成功後，串列埠會顯示：
```
📱 IP 位址: 192.168.1.100
🌐 網頁設定介面: http://192.168.1.100
```
在同一網路內的任何裝置開啟該網址即可設定。

#### AP 模式（WiFi 連線失敗）
當裝置 20 秒內無法連線至設定的 WiFi 時，會自動切換為 AP 模式：
1. 連線至 WiFi 網路：`GPS-Tracker-XXXXXXXX`（密碼：`12345678`）
2. 開啟瀏覽器訪問：`http://192.168.4.1`

### 可設定的參數

| 參數 | 說明 | 預設值 |
|------|------|--------|
| WiFi SSID | WiFi 網路名稱 | - |
| WiFi 密碼 | WiFi 網路密碼 | - |
| 發佈移動門檻 | 移動多少公尺才上傳 | 50 公尺 |
| 位置更新間隔 | 多久檢查一次位置 | 1500 毫秒 |
| 方向角變化門檻 | 方向改變多少度才上傳 | 25 度 |

### 設定步驟

1. 開啟網頁設定介面
2. 修改需要的參數
3. 點擊「💾 儲存設定」
4. 重新啟動裝置以套用新設定

> **💡 提示**：所有設定會儲存在 ESP32 的 Flash 記憶體中，斷電後不會遺失。

## 進階參數調整

### 可透過網頁設定的參數

| 參數 | 預設值 | 說明 |
|------|--------|------|
| WiFi SSID | - | WiFi 網路名稱 |
| WiFi 密碼 | - | WiFi 網路密碼 |
| `MOVE_THRESHOLD_METERS` | 50 | 移動距離門檻（公尺） |
| `UPDATE_INTERVAL_MS` | 1500 | 位置檢查間隔（毫秒） |
| `COURSE_THRESHOLD_DEG` | 25.0 | 方向角變化門檻（度） |

### 固定參數（需修改程式碼）

若需調整以下參數，請編輯 `gps_v20250725.ino`：

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `TIME_THRESHOLD_MS` | 30000 | 時間門檻（毫秒） |
| `IDLE_TIMEOUT_MS` | 60000 | 靜止逾時（毫秒） |
| `SPEED_THRESHOLD_KMPH` | 1.0 | 靜止速度門檻（km/h） |
| `MIN_SATELLITES` | 4 | 最小衛星數量 |
| `BATCH_SIZE` | 3 | 批次上傳筆數 |
| `UPLOAD_INTERVAL_MS` | 1000 | 每筆資料上傳間隔（毫秒） |

## MQTT 資料格式

發布至 Topic：`owntracks/{USER_ID}/{DEVICE_ID}`

```json
{
  "_type": "location",
  "tst": 1763695196,
  "lat": 24.247824,
  "lon": 120.543556,
  "acc": 3.0,
  "vel": 15.9,
  "cog": 20.5,
  "satcnt": 4
}
```

| 欄位 | 說明 |
|------|------|
| `_type` | 訊息類型（固定為 "location"） |
| `tst` | Unix 時間戳 |
| `lat` | 緯度（6 位小數） |
| `lon` | 經度（6 位小數） |
| `acc` | 精度（公尺） |
| `vel` | 速度（km/h） |
| `cog` | 方向角（度） |
| `satcnt` | 衛星數量 |

## 運作邏輯

### WiFi 連線模式

#### 正常模式
- 成功連線至設定的 WiFi
- 取得 IP 位址，可在同一網路內透過網頁設定
- 開始 GPS 追蹤與 MQTT 上傳

#### AP 模式（自動備援）
- WiFi 連線失敗超過 20 秒自動啟動
- 建立 WiFi 基地台：`GPS-Tracker-XXXXXXXX`（密碼：`12345678`）
- 提供網頁設定介面：`http://192.168.4.1`
- 適合首次設定或 WiFi 資訊變更時使用

### 觸發上傳條件（滿足任一即觸發）

1. **距離觸發**：移動距離 ≥ 50 公尺（可調整）
2. **時間觸發**：距上次發布 ≥ 30 秒
3. **方向觸發**：方向角變化 ≥ 25 度（可調整）

### 雙緩衝批次上傳機制

- **累積緩衝區**：持續接收新的 GPS 資料
- **上傳緩衝區**：背景上傳先前累積的資料
- 兩者獨立運作，互不阻塞
- 累積 3 筆資料後自動啟動批次上傳

### 資料驗證

- 衛星數量 < 4 顆：資料視為無效，不加入上傳佇列
- GPS 訊號遺失：自動熄滅 LED 並停止上傳
- MQTT 連線失敗：自動重連機制

### 靜止模式

- 速度 < 1 km/h 持續 60 秒 → 進入靜止模式
- 靜止模式：強制上傳累積資料後暫停發布
- 恢復移動：自動退出靜止模式並繼續追蹤

## LED 狀態指示

| LED 狀態 | 說明 |
|----------|------|
| 💡 亮燈 | GPS 定位成功 |
| 🔴 熄燈 | GPS 訊號遺失或尚未定位 |

## 串列埠輸出範例

### 正常啟動（WiFi 連線成功）

```
🚀 GPS + MQTT 初始化中...

📂 已載入設定:
   WiFi SSID: MyHomeWiFi
   移動門檻: 50.0 公尺
   更新間隔: 1500 毫秒
   方向門檻: 25.0 度

🔍 查詢當前 GNSS 系統配置...
🌍 設定全球四大導航系統...
✅ 系統配置:
   🇺🇸 GPS (美國)
   🇷🇺 GLONASS (俄羅斯)
   🇪🇺 Galileo (歐盟)
   🇨🇳 BeiDou (中國北斗)

🌐 連線至 WiFi... 已連線！
📱 IP 位址: 192.168.1.100
🌐 網頁設定介面: http://192.168.1.100
✅ 網頁伺服器已啟動

🔍 搜星中... 已鎖定 8 顆衛星 [■■■■■■■■□□] (即將定位...)

🎯 ========== 定位成功 ==========
⏱️  TTFF (Time To First Fix): 12.34 秒
🔥 Hot Start (熱啟動) - 優化生效！
🛰️  衛星數量: 8
📊 HDOP: 1.2
================================

===== 新位置 =====
緯度：24.247824
經度：120.543556
衛星數量：8
速度：15.9 km/h
方向角：20.5°
==================

📦 已累積 3/3 筆資料
📤 啟動背景上傳 3 筆資料（累積緩衝區已清空，可繼續接收新資料）
[MQTT 1/3] {"_type":"location",...}
[MQTT 2/3] {"_type":"location",...}
[MQTT 3/3] {"_type":"location",...}
✅ 批次上傳完成！
```

### AP 模式啟動（WiFi 連線失敗）

```
🚀 GPS + MQTT 初始化中...

📂 已載入設定:
   WiFi SSID: WrongSSID
   移動門檻: 50.0 公尺
   更新間隔: 1500 毫秒
   方向門檻: 25.0 度

🌐 連線至 WiFi.......... (10秒).......... (20秒) 連線失敗！

📡 WiFi 連線逾時，切換為 AP 模式
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ AP 模式已啟動！

📱 請使用手機/電腦連線至：
   SSID: GPS-Tracker-A1B2C3D4
   密碼: 12345678

🌐 然後開啟瀏覽器訪問：
   http://192.168.4.1
   或 http://192.168.4.1
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

✅ 網頁設定介面已啟動
```

## 故障排除

### WiFi 連線問題

#### 無法連線至 WiFi
1. ✅ 等待 20 秒，裝置會自動切換為 AP 模式
2. ✅ 連線至 `GPS-Tracker-XXXXXXXX`（密碼：`12345678`）
3. ✅ 開啟瀏覽器訪問 `http://192.168.4.1`
4. ✅ 重新設定正確的 WiFi SSID 和密碼

#### 忘記設定的 WiFi 密碼
1. ✅ 在網頁設定介面點擊「🔄 重置為預設值」
2. ✅ 或透過序列埠監視器查看載入的設定

### GPS 無法定位

1. ✅ 確認 GPS 天線朝向天空（不要平放）
2. ✅ 確認不在室內或地下室（玻璃會阻擋訊號）
3. ✅ 檢查接線：TX ↔ RX 是否交叉連接
4. ✅ 確認鮑率設定為 9600
5. ✅ 等待 30-60 秒讓 GPS 完成冷啟動

### MQTT 連線失敗

1. ✅ 檢查網路連線（確認裝置已連上 WiFi）
2. ✅ 確認 MQTT 伺服器 IP、埠號正確
3. ✅ 驗證 MQTT 帳號密碼
4. ✅ 確認防火牆沒有阻擋連線
5. ✅ 檢查 MQTT 伺服器是否正常運作

### 衛星數量異常偏低

1. ✅ 確認 GPS 模組支援多 GNSS 系統
2. ✅ 移除金屬遮蔽物或電磁干擾源
3. ✅ 等待更長時間讓 GPS 取得星曆表（冷啟動需 30-60 秒）
4. ✅ 移動到空曠處測試

### 網頁設定介面無法開啟

#### 正常模式
1. ✅ 確認裝置與電腦/手機在同一網路
2. ✅ 檢查串列埠輸出的 IP 位址是否正確
3. ✅ 嘗試關閉防火牆或 VPN

#### AP 模式
1. ✅ 確認已連線至 `GPS-Tracker-XXXXXXXX` 網路
2. ✅ 訪問 `http://192.168.4.1`（不是 https）
3. ✅ 某些手機會自動斷開無網際網路的 WiFi，請在 WiFi 設定中關閉此功能

## 技術特點

### 雙緩衝區設計
- 採用生產者-消費者模式
- 累積緩衝區：持續接收 GPS 資料
- 上傳緩衝區：背景批次上傳
- 兩者獨立運作，互不阻塞，確保不漏失資料

### 非阻塞式架構
- 網頁伺服器在背景運行
- MQTT 上傳不影響 GPS 資料收集
- 所有操作採用非阻塞式設計

### 智慧連線管理
- WiFi 連線失敗自動切換 AP 模式
- MQTT 斷線自動重連機制
- Last Will Testament (LWT) 離線通知

### 持久化儲存
- 使用 ESP32 NVS (Non-Volatile Storage)
- 設定儲存於 Flash，斷電不遺失
- 支援重置為預設值功能

## 更新日誌

### v2.0.0 (2025-11-23)
- ✨ 新增網頁設定介面
- ✨ 新增 AP 模式自動備援
- ✨ 實作雙緩衝區批次上傳機制
- ✨ 新增持久化儲存功能
- 🐛 修復上傳時阻塞資料累積的問題
- 🐛 修復衛星數量記錄不一致的問題

### v1.0.0 (2024-07-25)
- 🎉 初始版本發布
- ✅ 基本 GPS 追蹤與 MQTT 上傳功能
- ✅ 多衛星系統支援
- ✅ 智慧移動偵測

## 授權

本專案採用 MIT 授權條款。

## 貢獻

歡迎提交 Issue 或 Pull Request！

## 作者

InskyChen

## 參考資料

- [TinyGPSPlus Library](https://github.com/mikalhart/TinyGPSPlus)
- [PubSubClient Library](https://github.com/knolleary/pubsubclient)
- [OwnTracks Protocol](https://owntracks.org/booklet/tech/json/)
- [ESP32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ESP32 Preferences Library](https://github.com/espressif/arduino-esp32/tree/master/libraries/Preferences)
