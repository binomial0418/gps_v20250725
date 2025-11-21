# ESP32 GPS Tracker with MQTT

基於 ESP32 的 GPS 追蹤器，透過 MQTT 協定將位置資料發布到 OwnTracks 伺服器。支援多衛星系統（GPS、GLONASS、Galileo、BeiDou）快速定位，具備智慧移動偵測與批次上傳功能。

## 主要功能

- 🛰️ **多衛星系統支援**：同時支援 GPS、GLONASS、Galileo、北斗四大導航系統
- 📍 **智慧移動偵測**：基於距離、時間、方向角變化觸發上傳
- 📦 **批次上傳機制**：累積多筆資料後批次發送，減少網路負載
- 💤 **自動休眠**：靜止超過設定時間自動暫停上傳，節省流量
- 🔄 **自動重連**：WiFi/MQTT 斷線自動重連
- 💡 **視覺回饋**：LED 指示燈顯示 GPS 定位狀態
- 🛡️ **資料驗證**：衛星數量門檻過濾，確保資料品質

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

### 1. 複製配置檔

```bash
cp config.h.example config.h
```

### 2. 編輯 `config.h` 填入您的參數

```cpp
// WiFi 設定
#define WIFI_SSID   "您的WiFi名稱"
#define WIFI_PWD    "您的WiFi密碼"

// MQTT 伺服器設定
#define MQTT_HOST   "您的MQTT伺服器IP"
#define MQTT_PORT   50883
#define MQTT_USER   "MQTT帳號"
#define MQTT_PASS   "MQTT密碼"

// OwnTracks 設定
#define USER_ID     "使用者ID"      // 例如: "user1"
#define DEVICE_ID   "裝置ID"        // 例如: "car"
```

### 3. 上傳程式到 ESP32

使用 Arduino IDE 或 arduino-cli 編譯並上傳。

### 4. 開啟串列埠監視器（115200 baud）

觀察 GPS 搜星與定位過程。

## 參數調整

可在 `gps_v20250725.ino` 中調整以下參數：

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `MOVE_THRESHOLD_METERS` | 30 | 移動距離門檻（公尺） |
| `TIME_THRESHOLD_MS` | 30000 | 時間門檻（毫秒） |
| `UPDATE_INTERVAL_MS` | 1500 | 位置檢查間隔（毫秒） |
| `IDLE_TIMEOUT_MS` | 60000 | 靜止逾時（毫秒） |
| `SPEED_THRESHOLD_KMPH` | 1.0 | 靜止速度門檻（km/h） |
| `COURSE_THRESHOLD_DEG` | 20.0 | 方向角變化門檻（度） |
| `MIN_SATELLITES` | 4 | 最小衛星數量 |
| `BATCH_SIZE` | 3 | 批次上傳筆數 |

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

### 觸發上傳條件（滿足任一即觸發）

1. **距離觸發**：移動距離 ≥ 30 公尺
2. **時間觸發**：距上次發布 ≥ 30 秒
3. **方向觸發**：方向角變化 ≥ 20 度

### 資料驗證

- 衛星數量 < 4 顆：資料視為無效，不加入上傳佇列
- GPS 訊號遺失：自動熄滅 LED 並停止上傳

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

```
🚀 GPS + MQTT 初始化中...
🔍 查詢當前 GNSS 系統配置...
🌍 設定全球四大導航系統...
✅ 系統配置:
   🇺🇸 GPS (美國)
   🇷🇺 GLONASS (俄羅斯)
   🇪🇺 Galileo (歐盟)
   🇨🇳 BeiDou (中國北斗)

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
📤 啟動背景上傳 3 筆資料...
[MQTT 1/3] {"_type":"location",...}
[MQTT 2/3] {"_type":"location",...}
[MQTT 3/3] {"_type":"location",...}
✅ 批次上傳完成！
```

## 故障排除

### GPS 無法定位

1. ✅ 確認 GPS 天線朝向天空（不要平放）
2. ✅ 確認不在室內或地下室（玻璃會阻擋訊號）
3. ✅ 檢查接線：TX ↔ RX 是否交叉連接
4. ✅ 確認鮑率設定為 9600

### MQTT 連線失敗

1. ✅ 檢查網路連線
2. ✅ 確認 MQTT 伺服器 IP、埠號正確
3. ✅ 驗證 MQTT 帳號密碼
4. ✅ 確認防火牆沒有阻擋連線

### 衛星數量異常偏低

1. ✅ 確認 GPS 模組支援多 GNSS 系統
2. ✅ 移除金屬遮蔽物或電磁干擾源
3. ✅ 等待更長時間讓 GPS 取得星曆表（冷啟動需 30-60 秒）

## 授權

本專案採用 MIT 授權條款。

## 貢獻

歡迎提交 Issue 或 Pull Request！

## 參考資料

- [TinyGPSPlus Library](https://github.com/mikalhart/TinyGPSPlus)
- [PubSubClient Library](https://github.com/knolleary/pubsubclient)
- [OwnTracks Protocol](https://owntracks.org/booklet/tech/json/)
- [ESP32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
