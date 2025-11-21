/*
  GPS + MQTT 定位上傳
  支援批次上傳與靜止偵測
  使用 TinyGPS++ 庫解析 GPS 資料
  使用 PubSubClient 庫進行 MQTT 通訊
  適用於 ESP32 開發板

  作者: InskyChen
  日期: 2024-07-25
*/
#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "config.h"  // 引入配置檔

// ====== GPS 更新參數 ======
#define MOVE_THRESHOLD_METERS 30.0f // 發佈移動門檻（公尺）
#define TIME_THRESHOLD_MS 30000UL   // 發佈時間門檻（毫秒）
#define UPDATE_INTERVAL_MS 1500UL   // 位置更新/檢查間隔（毫秒）
#define IDLE_TIMEOUT_MS 60000UL     // 靜止逾時（毫秒）
#define SPEED_THRESHOLD_KMPH 1.0f   // 視為靜止的速度門檻（km/h）
#define COURSE_THRESHOLD_DEG 20.0f  // 方向角變化門檻（度）
#define MIN_SATELLITES 4            // 最小衛星數量門檻

// ====== 批次上傳設定 ======
#define BATCH_SIZE 3              // 批次上傳筆數（累積 N 筆才上傳）
#define UPLOAD_INTERVAL_MS 1000UL // 每筆資料間隔（毫秒）

// ====== 硬體腳位設定 ======
#define LED_PIN 2 // 藍燈 GPIO（預設板載 LED）
#define GPS_RX 16 // ESP32 讀 GPS（接 GPS 的 TX）
#define GPS_TX 17 // ESP32 寫 GPS（接 GPS 的 RX）
/* ====== 自動生成的 MQTT Topic ====== */
#define TOPIC_BASE  "owntracks/" USER_ID "/" DEVICE_ID
#define LWT_TOPIC   TOPIC_BASE "/lwt"
/* ============================== */

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);   // UART2

WiFiClient   wifiCli;
PubSubClient mqtt(wifiCli);

unsigned long startMillis = 0;
unsigned long lastUpdate  = 0;
unsigned long lastPublish = 0;  // 上次發佈 MQTT 的時間
unsigned long lastWiFiTry = 0;
unsigned long lastMovement = 0; // 上次移動（速度 > 0）的時間

bool fixAcquired    = false;   // 是否「曾經」成功定位
bool currentlyValid = false;   // 目前是否有有效定位
bool isIdle         = false;   // 是否處於靜止狀態（暂停上傳）

// GPS 診斷統計
unsigned long lastGpsUpdate = 0;     // 上次 GPS 資料更新時間
int gpsUpdateCount = 0;              // GPS 更新次數
unsigned long diagnosticTimer = 0;   // 診斷輸出計時器
int lastSatCount = 0;                // 上次衛星數量
unsigned long lastNoMoveMsg = 0;     // 上次「未移動」訊息時間

bool   hasLastPosition = false;
double lastLat = 0.0, lastLng = 0.0;
float  lastCourse = -1.0;      // 上次發佈的方向角（-1 表示尚未記錄）

// 批次上傳用的資料結構
struct GpsData {
  float lat;
  float lon;
  float acc;
  float vel;
  float cog;
  int satcnt;
  unsigned long timestamp;
};

GpsData batchBuffer[BATCH_SIZE];  // 批次緩衝區
int batchCount = 0;                // 目前已累積筆數

// 背景上傳狀態變數
bool isUploading = false;          // 是否正在上傳
int uploadIndex = 0;               // 當前上傳索引
int uploadTotal = 0;               // 本次上傳總筆數
unsigned long lastUploadTime = 0;  // 上次發送時間
  
/* ────────── 工具 ────────── */
void ledOn()  { digitalWrite(LED_PIN, HIGH); }
void ledOff() { digitalWrite(LED_PIN, LOW);  }

// 計算方向角差異（處理 0°/360° 邊界）
float courseDifference(float course1, float course2) {
  float diff = fabs(course1 - course2);
  if (diff > 180.0f) {
    diff = 360.0f - diff;
  }
  return diff;
}

bool ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;

  unsigned long now = millis();
  if (now - lastWiFiTry < 10000UL) return false;  // 每 10 秒嘗試一次
  lastWiFiTry = now;

  WiFi.reconnect(); // 不影響現有 TCP，比 WiFi.begin() 溫和
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 6000UL) {
    delay(300);
  }
  return WiFi.status() == WL_CONNECTED;
}

bool ensureMqtt() {
  if (mqtt.connected()) return true;
  if (!ensureWiFi())    return false;

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setKeepAlive(30); // 30 秒 PINGREQ

  String cid = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  bool ok = mqtt.connect(
              cid.c_str(),
              MQTT_USER, MQTT_PASS,
              LWT_TOPIC,          // willTopic
              0,                  // willQos
              true,               // willRetain
              "offline"           // willMessage
            );
  if (ok) {
    mqtt.publish(LWT_TOPIC, "online", true);
  }
  return ok;
}

// 將資料加入批次緩衝區（非阻塞）
void addToBatch(float lat, float lon, float accMeters, float velocity, float course, int satellites) {
  // 如果正在上傳，等待下次再加
  if (isUploading) {
    Serial.println("⏳ 批次上傳中，跳過本筆資料");
    return;
  }

  unsigned long epoch = time(nullptr);
  if (epoch < 1000000000UL) {
    epoch = millis() / 1000UL;
  }

  batchBuffer[batchCount].lat = lat;
  batchBuffer[batchCount].lon = lon;
  batchBuffer[batchCount].acc = accMeters;
  batchBuffer[batchCount].vel = velocity;
  batchBuffer[batchCount].cog = course;
  batchBuffer[batchCount].satcnt = satellites;
  batchBuffer[batchCount].timestamp = epoch;
  batchCount++;

  Serial.printf("📦 已累積 %d/%d 筆資料\n", batchCount, BATCH_SIZE);

  // 達到批次大小，啟動背景上傳
  if (batchCount >= BATCH_SIZE) {
    startBatchUpload();
  }
}

// 啟動批次上傳（非阻塞）
void startBatchUpload() {
  if (batchCount == 0) return;
  if (!ensureMqtt()) {
    Serial.println("⚠️ MQTT 未連線，無法啟動上傳");
    return;
  }

  isUploading = true;
  uploadIndex = 0;
  uploadTotal = batchCount;
  lastUploadTime = millis();
  Serial.printf("📤 啟動背景上傳 %d 筆資料...\n", uploadTotal);
}

// 背景處理批次上傳（在 loop 中呼叫）
void processBatchUpload() {
  if (!isUploading) return;

  // 檢查是否達到間隔時間
  if (millis() - lastUploadTime < UPLOAD_INTERVAL_MS) return;

  // 發送當前筆資料
  if (uploadIndex < uploadTotal) {
    char json[160];
    snprintf(json, sizeof(json),
         "{\"_type\":\"location\",\"tst\":%lu,\"lat\":%.6f,\"lon\":%.6f,\"acc\":%.1f,\"vel\":%.1f,\"cog\":%.1f,\"satcnt\":%d}",
         batchBuffer[uploadIndex].timestamp, batchBuffer[uploadIndex].lat, batchBuffer[uploadIndex].lon,
         batchBuffer[uploadIndex].acc, batchBuffer[uploadIndex].vel, batchBuffer[uploadIndex].cog,
         batchBuffer[uploadIndex].satcnt);

    if (mqtt.publish(TOPIC_BASE, json, true)) {
      Serial.printf("[MQTT %d/%d] %s\n", uploadIndex + 1, uploadTotal, json);
    } else {
      Serial.printf("❌ [MQTT %d/%d] 發送失敗\n", uploadIndex + 1, uploadTotal);
    }

    uploadIndex++;
    lastUploadTime = millis();
  }

  // 所有資料已發送完成
  if (uploadIndex >= uploadTotal) {
    Serial.printf("✅ 批次上傳完成！\n\n");
    isUploading = false;
    batchCount = 0;  // 清空緩衝區
    uploadIndex = 0;
    uploadTotal = 0;
  }
}

// 舊版 sendBatch 保留作為備用（不再使用）
void sendBatch() {
  if (batchCount == 0) return;
  if (!ensureMqtt()) {
    Serial.println("⚠️ MQTT 未連線，批次上傳失敗");
    return;
  }

  Serial.printf("📤 開始批次上傳 %d 筆資料...\n", batchCount);

  for (int i = 0; i < batchCount; i++) {
    char json[160];
    snprintf(json, sizeof(json),
         "{\"_type\":\"location\",\"tst\":%lu,\"lat\":%.6f,\"lon\":%.6f,\"acc\":%.1f,\"vel\":%.1f,\"cog\":%.1f,\"satcnt\":%d}",
         batchBuffer[i].timestamp, batchBuffer[i].lat, batchBuffer[i].lon,
         batchBuffer[i].acc, batchBuffer[i].vel, batchBuffer[i].cog,
         gps.satellites.value());

    if (mqtt.publish(TOPIC_BASE, json, true)) {
      Serial.printf("[MQTT %d/%d] %s\n", i + 1, batchCount, json);
    } else {
      Serial.printf("❌ [MQTT %d/%d] 發送失敗\n", i + 1, batchCount);
    }
    delay(50);  // 避免發送過快
  }

  Serial.printf("✅ 批次上傳完成！\n\n");
  batchCount = 0;  // 清空緩衝區
}

/* ────────── SETUP ────────── */
void setup() {
  pinMode(LED_PIN, OUTPUT);
  ledOff();

  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("🚀 GPS + MQTT 初始化中...");
  
  // ===== GPS + 北斗 快速定位優化 =====
  delay(100);  // 等待 GPS 模組穩定
  
  // 1. 查詢目前系統配置（PMTK605）
  Serial.println("🔍 查詢當前 GNSS 系統配置...");
  gpsSerial.println("$PMTK605*31");  // 查詢配置
  delay(500);  // 等待回應
  while (gpsSerial.available()) {
    Serial.write(gpsSerial.read());  // 顯示回應
  }
  Serial.println();
  
  // 2. 啟用全球四大導航系統（PMTK353）
  // 參數格式: GPS,GLONASS,Galileo,BeiDou,QZSS (1=啟用, 0=停用)
  Serial.println("🌍 設定全球四大導航系統...");
  gpsSerial.println("$PMTK353,1,1,1,1,0*2B");  // 啟用全部四大系統
  delay(300);
  
  // 確認設定是否成功
  Serial.println("⏳ 等待模組確認...");
  delay(500);
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    Serial.write(c);
    if (c == '\n') break;
  }
  Serial.println();
  
  Serial.println("✅ 系統配置:");
  Serial.println("   🇺🇸 GPS (美國)");
  Serial.println("   🇷🇺 GLONASS (俄羅斯)");
  Serial.println("   🇪🇺 Galileo (歐盟)");
  Serial.println("   🇨🇳 BeiDou (中國北斗)");
  
  // 2. 設定為每秒更新 5 次（200ms），加快搜星
  gpsSerial.println("$PMTK220,200*2C");  // 5Hz 更新頻率
  delay(100);
  Serial.println("⚡ 設定更新頻率: 5Hz (200ms)");
  
  // 3. 輸出 GNRMC, GNGGA, GNGSA（多系統用 GN 開頭）
  // 啟用位置、品質、衛星資訊
  gpsSerial.println("$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  delay(100);
  Serial.println("📡 啟用 GNSS 多系統綜合輸出");
  
  // 4. 設定為 Hot Start 模式（保留星曆表和時間）
  gpsSerial.println("$PMTK101*32");  // Hot Start
  delay(100);
  
  Serial.println("✅ 全球導航系統快速定位模式已啟用");
  Serial.println("   優勢: 可見衛星數 3-4x↑ / 定位速度 5x↑ / 精度更高");
  // ===========================

  WiFi.begin(WIFI_SSID, WIFI_PWD);           // 先嘗試連上
  mqtt.setBufferSize(256);

  // 取時間（GMT+8）- NTP 時間可輔助 A-GPS
  configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("⏰ NTP 時間同步中（輔助 GPS 定位）...");

  startMillis = millis();
  lastMovement = millis();  // 初始化為開機時間，避免一開始就認為靜止
}

/* ────────── LOOP ────────── */
void loop() {
  // 讀入所有 GPS UART 資料
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
    
    // 監測 GPS 資料接收（每收到完整句子會觸發）
    if (c == '\n' && gps.location.isUpdated()) {
      unsigned long now = millis();
      if (lastGpsUpdate > 0) {
        unsigned long interval = now - lastGpsUpdate;
        // 只在定位前顯示更新頻率，避免訊息過多
        if (!fixAcquired && gpsUpdateCount % 5 == 0) {
          Serial.printf("⚡ GPS 更新間隔: %lu ms (目標 200ms)\n", interval);
        }
      }
      lastGpsUpdate = now;
      gpsUpdateCount++;
    }
  }

  // 定位狀態改變：從「無效 → 有效」
  if (gps.location.isValid() && !currentlyValid) {
    currentlyValid = true;
    ledOn();

    if (!fixAcquired) {
      fixAcquired = true;
      unsigned long fixTime = millis() - startMillis;
      Serial.println("\n🎯 ========== 定位成功 ==========");
      Serial.printf("⏱️  TTFF (Time To First Fix): %.2f 秒\n", fixTime / 1000.0);
      
      // 判斷啟動類型
      if (fixTime < 5000) {
        Serial.println("🔥 Hot Start (熱啟動) - 優化生效！");
      } else if (fixTime < 30000) {
        Serial.println("🌡️  Warm Start (溫啟動)");
      } else {
        Serial.println("❄️  Cold Start (冷啟動)");
      }
      
      Serial.printf("🛰️  衛星數量: %d\n", gps.satellites.value());
      Serial.printf("📊 HDOP: %.1f\n", gps.hdop.hdop());
      Serial.printf("🔄 總更新次數: %d\n", gpsUpdateCount);
      Serial.printf("📡 平均更新頻率: %.1f Hz\n", gpsUpdateCount * 1000.0 / fixTime);
      Serial.println("================================\n");
    } else {
      Serial.println("✅ 已重新取得定位！");
    }
  }

  // 定位狀態改變：從「有效 → 無效」
  if (!gps.location.isValid() && currentlyValid) {
    currentlyValid = false;
    ledOff();
    Serial.println("⚠️ GPS 訊號遺失，已熄滅藍燈");
  }

  // 定期顯示搜星診斷（每 5 秒）
  if (!fixAcquired && millis() - diagnosticTimer >= 5000) {
    diagnosticTimer = millis();
    int sats = gps.satellites.value();
    
    if (gps.satellites.isValid()) {
      Serial.printf("🔍 搜星中... 已鎖定 %d 顆衛星 ", sats);
      
      // 顯示搜星進度條
      Serial.print("[");
      for (int i = 0; i < 10; i++) {
        if (i < sats) Serial.print("■");
        else Serial.print("□");
      }
      Serial.print("] ");
      
      if (sats >= 4) {
        Serial.println("(即將定位...)");
      } else {
        Serial.printf("(需要 4 顆以上，已等待 %d 秒)\n", (millis() - startMillis) / 1000);
        
        // ⚠️ 衛星數過少的警告
        if (sats <= 3 && (millis() - startMillis) > 30000) {
          Serial.println("   ⚠️  衛星數異常偏低！請檢查:");
          Serial.println("      1. 天線是否朝向天空（不要平放）");
          Serial.println("      2. 是否在室內（玻璃會阻擋訊號）");
          Serial.println("      3. 模組是否支援多系統（查看回應訊息）");
          Serial.println("      4. 是否有金屬遮蔽物或電磁干擾");
        }
      }
      
      // 顯示衛星增加速度
      if (sats > lastSatCount) {
        Serial.printf("   ↗️ +%d 顆衛星！搜星速度良好\n", sats - lastSatCount);
      } else if (sats == lastSatCount && sats <= 3 && (millis() - startMillis) > 20000) {
        Serial.println("   ⏸️  衛星數停滯不增加，可能訊號受阻");
      }
      lastSatCount = sats;
    } else {
      Serial.printf("📡 等待 GPS 訊號... (%d 秒)\n", (millis() - startMillis) / 1000);
      Serial.println("   💡 提示: 請確認天線朝上，遠離金屬遮蔽物");
      
      if ((millis() - startMillis) > 60000) {
        Serial.println("   ❌ 超過 1 分鐘仍無訊號，可能的原因:");
        Serial.println("      • GPS 模組未正確連接（檢查 RX/TX 接線）");
        Serial.println("      • 鮑率設定錯誤（應為 9600）");
        Serial.println("      • 模組故障或未供電");
      }
    }
  }
  
  // 只有在目前有定位、且資料更新、且超過更新間隔時才處理
  if (currentlyValid && gps.location.isUpdated() &&
      millis() - lastUpdate >= UPDATE_INTERVAL_MS) {

    lastUpdate = millis();

    double currentLat = gps.location.lat();
    double currentLng = gps.location.lng();
    float currentSpeed = gps.speed.kmph();

    // 靜止偵測：檢查速度是否 > 門檻
    if (currentSpeed > SPEED_THRESHOLD_KMPH) {
      lastMovement = millis();  // 更新最後移動時間
      if (isIdle) {
        isIdle = false;
        Serial.println("✅ 恢復移動，重新開始上傳");
      }
    } else if (lastMovement > 0 && millis() - lastMovement >= IDLE_TIMEOUT_MS) {
      // 速度為 0 且已經超過逾時時間
      if (!isIdle) {
        // ⚠️ 進入靜止前，先強制上傳所有累積的資料
        if (batchCount > 0 && !isUploading) {
          Serial.printf("⚡ 進入靜止模式前，強制上傳累積的 %d 筆資料\n", batchCount);
          startBatchUpload();
          
          // 等待上傳完成（最多等 5 秒）
          unsigned long waitStart = millis();
          while (isUploading && millis() - waitStart < 5000) {
            processBatchUpload();
            mqtt.loop();
            delay(10);
          }
          
          if (isUploading) {
            Serial.println("⚠️ 上傳逾時，部分資料可能遺失");
            isUploading = false;
            batchCount = 0;  // 清空避免下次重複上傳
          }
        }
        
        isIdle = true;
        Serial.printf("🚫 車輛靜止超過 %.1f 分鐘，暂停上傳\n", IDLE_TIMEOUT_MS / 60000.0);
      }
    }

    // 如果處於靜止狀態，跳過上傳
    if (isIdle) {
      mqtt.loop();
      return;
    }

    double dist = 0.0;
    if (hasLastPosition) {
      dist = TinyGPSPlus::distanceBetween(lastLat, lastLng, currentLat, currentLng);
    }

    // 方向角判斷：每次都取得當前方向並計算差異
    float currentCourse = gps.course.deg();
    float courseDiff = 0.0;
    bool courseValid = gps.course.isValid();
    
    // 如果之前有記錄方向角，且當前方向有效，就計算差異
    if (lastCourse >= 0.0 && courseValid) {
      courseDiff = courseDifference(lastCourse, currentCourse);
    }
    
    // 如果是第一次取得有效方向角，立即記錄（不需等上傳）
    if (lastCourse < 0.0 && courseValid) {
      lastCourse = currentCourse;
      Serial.printf("🧭 初始方向角：%.1f°\n", lastCourse);
    }

    unsigned long timeSincePublish = millis() - lastPublish;
    bool distanceReached = hasLastPosition && dist >= MOVE_THRESHOLD_METERS;
    bool timeReached = hasLastPosition && timeSincePublish >= TIME_THRESHOLD_MS;
    bool courseChanged = lastCourse >= 0.0 && courseValid && courseDiff >= COURSE_THRESHOLD_DEG;

    // 若距雩不足且時間未到且方向未變，則跳過上傳
    if (hasLastPosition && !distanceReached && !timeReached && !courseChanged) {
      // 每 5 秒才顯示一次未移動訊息，減少串口輸出
      if (millis() - lastNoMoveMsg >= 5000) {
        Serial.printf("📍 未移動（距離 %.2f m < %.1f m，時間 %.1f s < %.1f s）\n", 
                      dist, MOVE_THRESHOLD_METERS, 
                      timeSincePublish / 1000.0, TIME_THRESHOLD_MS / 1000.0);
        lastNoMoveMsg = millis();
      }
      mqtt.loop();   // 仍然跑一下，避免斷線
      return;
    }

    // 顯示新位置資訊 + 本次移動距離/時間/方向
    if (hasLastPosition) {
      if (distanceReached) {
        Serial.printf("🚶‍♂️ 移動觸發：距離 %.2f m (>= %.1f m)\n", dist, MOVE_THRESHOLD_METERS);
      } else if (timeReached) {
        Serial.printf("⏰ 時間觸發：經過 %.1f 秒 (>= %.1f s)\n", 
                      timeSincePublish / 1000.0, TIME_THRESHOLD_MS / 1000.0);
      } else if (courseChanged) {
        Serial.printf("🧭 方向觸發：變化 %.1f° (>= %.1f°)\n", courseDiff, COURSE_THRESHOLD_DEG);
      }
    } else {
      Serial.println("🚩 首次位置紀錄");
    }

    Serial.println("\n===== 新位置 =====");
    Serial.print("緯度："); Serial.println(currentLat, 6);
    Serial.print("經度："); Serial.println(currentLng, 6);
    int satCount = gps.satellites.value();
    Serial.print("衛星數量："); Serial.println(satCount);
    Serial.print("HDOP(raw)："); Serial.println(gps.hdop.value()); // TinyGPS++ 回傳 *100 的值
    Serial.print("海拔："); Serial.print(gps.altitude.meters()); Serial.println(" m");
    Serial.print("速度："); Serial.print(gps.speed.kmph()); Serial.println(" km/h");
    Serial.print("方向角："); Serial.print(gps.course.deg()); Serial.println("°");
    Serial.println("==================\n");

    // 衛星數量檢查：小於門檻值時不加入 MQTT Queue
    if (satCount < MIN_SATELLITES) {
      Serial.printf("❌ 衛星數量不足（%d < %d），資料無效，不加入上傳佇列\n\n", satCount, MIN_SATELLITES);
      mqtt.loop();
      return;
    }

    // 加入批次緩衝區（acc 目前仍給固定 3m；要用 HDOP 推算可再改）
    addToBatch((float)currentLat, (float)currentLng, 3.0f, (float)gps.speed.kmph(), (float)gps.course.deg(), satCount);

    // 更新「上一次已發佈」的位置、時間
    lastLat = currentLat;
    lastLng = currentLng;
    lastPublish = millis();
    hasLastPosition = true;
    
    // 更新方向角記錄（無論是否因方向變化觸發上傳，都要更新為當前方向）
    if (courseValid) {
      lastCourse = currentCourse;
    }
  }

  // 背景處理批次上傳（非阻塞）
  processBatchUpload();

  // 保持 MQTT 連線
  mqtt.loop();
  delay(10);
}