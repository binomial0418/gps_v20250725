# Copilot Instructions for ESP32 GPS Tracker

## Project Overview
This is an **ESP32-based GPS tracker** that publishes location data via MQTT using the OwnTracks protocol. The system tracks movement, manages WiFi/MQTT reconnection, and provides visual feedback via LED.

## Architecture & Data Flow
1. **GPS Module (UART2)** → `TinyGPSPlus` parser → location validation
2. **WiFi Client** → `PubSubClient` (MQTT) → OwnTracks server (`220.132.203.243:50883`)
3. **LED Feedback**: Blue LED (GPIO 2) on when GPS fix acquired, off when signal lost
4. **Movement Detection**: Only publishes if moved ≥50m from last position (configurable via `MOVE_THRESHOLD_METERS`)

## Hardware Configuration
- **GPS UART**: RX=GPIO16, TX=GPIO17 (HardwareSerial 2 @ 9600 baud)
- **LED**: GPIO 2 (built-in blue LED on most ESP32 boards)
- **Serial Monitor**: 115200 baud for debugging output

## Build & Upload Workflow
The project uses **arduino-cli** for compilation:

```bash
# Compile for ESP32
arduino-cli compile --fqbn esp32:esp32:esp32 gps_v20250725.ino

# Upload to device (adjust port as needed)
arduino-cli upload -p /dev/cu.usbserial-* --fqbn esp32:esp32:esp32 gps_v20250725.ino
```

**Critical**: Ensure ESP32 board support is installed:
```bash
arduino-cli core install esp32:esp32
```

## Key Libraries & Dependencies
- `TinyGPSPlus`: GPS NMEA sentence parsing
- `PubSubClient`: MQTT client (buffer size set to 256 bytes)
- `WiFi.h`: ESP32 WiFi management
- `HardwareSerial.h`: UART2 for GPS communication

Install via Arduino Library Manager or platformio if converting.

## Code Conventions & Patterns

### State Management
- **`fixAcquired`**: Tracks if GPS ever got first fix (lifetime flag)
- **`currentlyValid`**: Tracks current GPS signal status (can toggle)
- **`hasLastPosition`**: Whether we have a reference point for distance calculations

### Reconnection Strategy
- **WiFi**: `WiFi.reconnect()` every 10s if disconnected (gentle, preserves TCP)
- **MQTT**: Auto-reconnect with Last Will Testament (LWT) to `owntracks/mt/t1/lwt`
- **Keep-Alive**: 30s MQTT ping interval

### Movement Threshold Logic
```cpp
if (hasLastPosition && dist < MOVE_THRESHOLD_METERS) {
  // Skip MQTT publish, only log "未移動"
  return;
}
```
This prevents excessive MQTT traffic when stationary.

### OwnTracks JSON Format
```json
{
  "_type": "location",
  "tst": <unix_timestamp>,
  "lat": <latitude_6_decimals>,
  "lon": <longitude_6_decimals>,
  "acc": <accuracy_meters>
}
```
Published with QoS 0, retained flag true to `owntracks/mt/t1`.

## Configuration Values (Hardcoded)
Update these in the source when deploying:
- **WiFi**: `WIFI_SSID`, `WIFI_PWD`
- **MQTT**: `MQTT_HOST`, `MQTT_PORT`, `MQTT_USER`, `MQTT_PASS`
- **OwnTracks**: `USER_ID` ("mt"), `DEVICE_ID` ("t1")
- **Timing**: `UPDATE_INTERVAL_MS` (10000ms = 10s check interval)
- **Threshold**: `MOVE_THRESHOLD_METERS` (50m default)

## Debugging Tips
- **GPS not locking**: Check `gps.satellites.value()` and HDOP in serial output
- **MQTT not connecting**: Verify network access to `220.132.203.243:50883`
- **LED stays off**: GPS module wiring or baud rate mismatch (should be 9600)
- **Excessive publishes**: Increase `MOVE_THRESHOLD_METERS` or `UPDATE_INTERVAL_MS`

## Timezone & NTP
- Configured for **GMT+8** (Taiwan/Hong Kong/Singapore time)
- Uses `pool.ntp.org` and `time.nist.gov` for time sync
- Timestamp falls back to `millis()/1000` if NTP not synced

## Code Style Notes
- Chinese comments (繁體中文) for inline documentation
- Emoji prefixes in serial output (🚀 ✅ ⚠️ 📍 🚶‍♂️) for visual parsing
- camelCase for variables, UPPER_CASE for constants
- All core logic in single `.ino` file (no separate headers)
