#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <time.h>
#include <Wire.h>
#include <MPU6050.h>

// ============== CONFIGURATION ==============
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define API_KEY "YOUR_FIREBASE_API_KEY"
#define DATABASE_URL "YOUR_FIREBASE_DATABASE_URL"
#define DATABASE_SECRET "YOUR_FIREBASE_DATABASE_SECRET"

String deviceID = "demo_device_001";

// ============== DEMO LOCATION / TELEMETRY ==============
// Public-repository-safe placeholders: this sketch does not read a GPS module
// or a battery gauge. Coordinates are zeroed, gpsValid is false, and battery
// is -1 (unknown) until real sensor implementations are added.
#define FIXED_LAT 0.0
#define FIXED_LNG 0.0
#define FIXED_ALTITUDE 0.0

// ============== PINS ==============
#define TOUCH_PIN 4
#define BUZZER_PIN 18
#define MPU_SDA 21    // Default ESP32 SDA
#define MPU_SCL 22    // Default ESP32 SCL

// ============== TIMING ==============
#define BUZZER_DURATION 2000
#define SOS_TIMEOUT 300000
#define STATUS_UPDATE_INTERVAL 3000
#define SETTINGS_CHECK_INTERVAL 10000

// ============== NTP CONFIGURATION ==============
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC 28800
#define DAYLIGHT_OFFSET_SEC 0

// ============== MPU6050 FALL DETECTION CONFIGURATION ==============
#define MPU_SAMPLE_RATE 100           // Hz
#define FALL_THRESHOLD 25.0           // Total acceleration threshold (m/s²) - ~2.5g
#define IMPACT_THRESHOLD 20.0         // Impact acceleration threshold (m/s²)
#define FREE_FALL_THRESHOLD 3.0       // Free fall threshold (m/s²) - near zero g
#define FALL_DETECTION_WINDOW 500     // Time window to confirm fall (ms)
#define POST_FALL_STILLNESS 2000      // Time to check for stillness after fall (ms)
#define ACTIVITY_THRESHOLD 2.0        // Threshold to detect movement (m/s²)

// ============== DEVICE SETTINGS ==============
struct DeviceSettings {
  int sosDuration = 5000;
  bool buzzerEnabled = true;
  bool mpuEnabled = true;
  float fallThreshold = FALL_THRESHOLD;
  unsigned long lastSettingsCheck = 0;
} settings;

// ============== GLOBALS ==============
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

bool timeInitialized = false;
bool isEmergency = false;
String currentSensor = "none";
unsigned long sosStartTime = 0;
unsigned long lastAlertTime = 0;
unsigned long buzzerStartTime = 0;
bool buzzerActive = false;
bool touchActive = false;
unsigned long touchBeginTime = 0;
int lastTouchState = HIGH;
int countdownStep = 0;
bool countdownDone = false;
unsigned long touchHeldDuration = 0;
unsigned long lastWifiCheck = 0;
int wifiReconnectAttempts = 0;
bool systemReady = false;
unsigned long lastStatusUpdate = 0;
bool historySavedForCurrentSOS = false;
unsigned long lastAlertId = 0;

// ============== MPU6050 GLOBALS ==============
MPU6050 mpu;
bool mpuInitialized = false;
bool mpuAvailable = false;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float totalAccel = 0;
float prevTotalAccel = 0;
unsigned long fallDetectionStart = 0;
bool impactDetected = false;
unsigned long lastMpuRead = 0;
unsigned long lastMpuUpdate = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float temperature = 0;

enum FallState {
  STATE_NORMAL,
  STATE_FREE_FALL,
  STATE_IMPACT,
  STATE_POST_FALL
};
FallState fallState = STATE_NORMAL;

// ============== TIME FUNCTIONS ==============
void initTime() {
  Serial.println("Initializing NTP time (Kuala Lumpur GMT+8)...");
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  int retry = 0;
  while (time(nullptr) < 1000000000 && retry < 20) {
    Serial.print(".");
    delay(500);
    retry++;
  }
  if (time(nullptr) > 1000000000) {
    timeInitialized = true;
    Serial.println(" OK");
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);
    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
    Serial.printf("Current KL time: %s\n", buffer);
  } else {
    Serial.println(" FAILED - using millis fallback");
  }
}

String getCurrentTimestamp() {
  if (!timeInitialized) return String(millis());
  time_t now = time(nullptr);
  unsigned long long timestampMs = (unsigned long long)now * 1000;
  return String((unsigned long)timestampMs);
}

String getCurrentTimeString() {
  if (!timeInitialized) return "NTP not ready";
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
  return String(buffer);
}

// ============== MPU6050 FUNCTIONS ==============
void initMPU6050() {
  Serial.println("\nInitializing MPU6050...");
  
  Wire.begin(MPU_SDA, MPU_SCL);
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("✗ MPU6050 connection failed!");
    mpuAvailable = false;
    return;
  }
  
  Serial.println("✓ MPU6050 connected");
  
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setRate(9);
  mpu.setSleepEnabled(false);
  
  calibrateMPU6050();
  
  mpuInitialized = true;
  mpuAvailable = true;
  Serial.println("✓ MPU6050 initialized for fall detection");
}

void calibrateMPU6050() {
  Serial.println("Calibrating MPU6050 (keep device still)...");
  delay(1000);
  
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    mpu.getAcceleration(&ax, &ay, &az);
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    delay(10);
  }
  
  int16_t ax_offset = ax_sum / samples;
  int16_t ay_offset = ay_sum / samples;
  int16_t az_offset = (az_sum / samples) - 16384;
  
  Serial.printf("Calibration complete. Offsets: X:%d Y:%d Z:%d\n", 
                ax_offset, ay_offset, az_offset);
}

void readMPU6050() {
  if (!mpuAvailable) return;
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  temperature = mpu.getTemperature() / 340.0 + 36.53;
  
  float accelScale = 9.81 / 2048.0;
  accelX = ax * accelScale;
  accelY = ay * accelScale;
  accelZ = az * accelScale;
  
  float gyroScale = 1.0 / 16.4;
  gyroX = gx * gyroScale;
  gyroY = gy * gyroScale;
  gyroZ = gz * gyroScale;
  
  prevTotalAccel = totalAccel;
  totalAccel = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
}

void detectFall() {
  if (!mpuAvailable || !settings.mpuEnabled || isEmergency) return;
  
  unsigned long now = millis();
  
  if (now - lastMpuRead < (1000 / MPU_SAMPLE_RATE)) return;
  lastMpuRead = now;
  
  readMPU6050();
  
  switch (fallState) {
    case STATE_NORMAL:
      if (totalAccel < FREE_FALL_THRESHOLD) {
        fallState = STATE_FREE_FALL;
        fallDetectionStart = now;
        Serial.println("[MPU] Free fall detected!");
      }
      else if (totalAccel > settings.fallThreshold && 
               abs(totalAccel - prevTotalAccel) > 15.0) {
        fallState = STATE_IMPACT;
        fallDetectionStart = now;
        Serial.println("[MPU] Sudden impact detected!");
      }
      break;
      
    case STATE_FREE_FALL:
      if (totalAccel > IMPACT_THRESHOLD) {
        fallState = STATE_IMPACT;
        impactDetected = true;
        Serial.println("[MPU] Impact detected after free fall!");
      }
      else if (now - fallDetectionStart > FALL_DETECTION_WINDOW) {
        fallState = STATE_NORMAL;
        Serial.println("[MPU] Free fall timeout, returning to normal");
      }
      break;
      
    case STATE_IMPACT:
      if (totalAccel > 8.0 && totalAccel < 12.0) {
        float movement = abs(totalAccel - prevTotalAccel);
        if (movement < ACTIVITY_THRESHOLD) {
          Serial.println("\n🚨 FALL DETECTED BY MPU6050!");
          Serial.printf("   Impact Force: %.2f m/s² (%.2f g)\n", totalAccel, totalAccel/9.81);
          Serial.printf("   Temperature: %.1f°C\n", temperature);
          Serial.printf("   Time: %s\n", getCurrentTimeString().c_str());
          
          activateSOS("mpu_fall", 0);
          
          fallState = STATE_POST_FALL;
          fallDetectionStart = now;
        }
      }
      else if (now - fallDetectionStart > POST_FALL_STILLNESS) {
        fallState = STATE_NORMAL;
        Serial.println("[MPU] Impact timeout, returning to normal");
      }
      break;
      
    case STATE_POST_FALL:
      if (now - fallDetectionStart > POST_FALL_STILLNESS) {
        fallState = STATE_NORMAL;
        impactDetected = false;
        Serial.println("[MPU] Reset to normal state");
      }
      break;
  }
}

void updateMPUStatus() {
  if (!Firebase.ready() || !mpuAvailable) return;
  
  FirebaseJson json;
  json.set("mpuStatus", mpuAvailable ? "connected" : "disconnected");
  json.set("mpuAccelX", accelX);
  json.set("mpuAccelY", accelY);
  json.set("mpuAccelZ", accelZ);
  json.set("mpuTotalAccel", totalAccel);
  json.set("mpuGyroX", gyroX);
  json.set("mpuGyroY", gyroY);
  json.set("mpuGyroZ", gyroZ);
  json.set("mpuTemperature", temperature);
  json.set("mpuFallState", (int)fallState);
  json.set("lastMpuUpdate", getCurrentTimestamp());
  
  Firebase.RTDB.updateNode(&fbdo, "/devices/" + deviceID + "/sensors", &json);
}

// ============== DEVICE SETTINGS ==============
void checkDeviceSettings() {
  if (!Firebase.ready()) return;
  bool success = false;
  int retries = 0;
  while (!success && retries < 3) {
    if (Firebase.RTDB.getJSON(&fbdo, "/devices/" + deviceID + "/settings")) {
      FirebaseJson json;
      json.setJsonData(fbdo.jsonString());
      FirebaseJsonData result;
      
      json.get(result, "sosDuration");
      if (result.success) {
        int newDuration = result.intValue;
        if (newDuration >= 1000 && newDuration <= 30000 && newDuration != settings.sosDuration) {
          settings.sosDuration = newDuration;
          Serial.printf("✓ SOS duration updated: %d ms\n", settings.sosDuration);
        }
      }
      
      json.get(result, "buzzerEnabled");
      if (result.success) {
        bool newBuzzer = result.boolValue;
        if (newBuzzer != settings.buzzerEnabled) {
          settings.buzzerEnabled = newBuzzer;
          Serial.printf("✓ Buzzer setting updated: %s\n", settings.buzzerEnabled ? "ON" : "OFF");
        }
      }
      
      json.get(result, "mpuEnabled");
      if (result.success) {
        bool newMpu = result.boolValue;
        if (newMpu != settings.mpuEnabled) {
          settings.mpuEnabled = newMpu;
          Serial.printf("✓ MPU setting updated: %s\n", settings.mpuEnabled ? "ON" : "OFF");
        }
      }
      
      json.get(result, "fallThreshold");
      if (result.success) {
        float newThreshold = result.floatValue;
        if (newThreshold >= 10.0 && newThreshold <= 50.0 && newThreshold != settings.fallThreshold) {
          settings.fallThreshold = newThreshold;
          Serial.printf("✓ Fall threshold updated: %.1f m/s²\n", settings.fallThreshold);
        }
      }
      
      success = true;
    } else {
      retries++;
      Serial.printf("✗ Settings read failed (attempt %d): %s\n", retries, fbdo.errorReason().c_str());
      if (retries < 3) delay(500);
    }
  }
}

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║    IoSafe Emergency Device         ║");
  Serial.println("║    Fixed Location + Firebase       ║");
  Serial.println("║    Remote Config Enabled           ║");
  Serial.println("║    MPU6050 Fall Detection          ║");
  Serial.println("║    ANTI-SPAM Protection            ║");
  Serial.println("╚════════════════════════════════════╝");
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(TOUCH_PIN, INPUT_PULLUP);
  
  Serial.print("Testing buzzer...");
  tickBuzzer();
  delay(100);
  tickBuzzer();
  Serial.println(" OK");
  delay(50);
  
  int touchState = digitalRead(TOUCH_PIN);
  delay(10);
  if (digitalRead(TOUCH_PIN) == touchState) {
    Serial.printf("Touch sensor: %s\n", touchState == LOW ? "TOUCHED" : "not touched");
    lastTouchState = touchState;
  }
  
  initMPU6050();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting WiFi");
  int wifiRetry = 0;
  while (WiFi.status() != WL_CONNECTED && wifiRetry < 30) {
    delay(500);
    Serial.print(".");
    wifiRetry++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" FAILED! Restarting...");
    delay(1000);
    ESP.restart();
  }
  Serial.println(" OK");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  
  initTime();
  
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = DATABASE_SECRET;
  Firebase.begin(&config, &auth);
  int fbRetry = 0;
  Serial.print("Connecting Firebase");
  while (!Firebase.ready() && fbRetry < 20) {
    delay(500);
    Serial.print(".");
    fbRetry++;
    if (fbRetry % 5 == 0) Firebase.begin(&config, &auth);
  }
  if (!Firebase.ready()) {
    Serial.println(" FAILED! Will retry in loop...");
  } else {
    Serial.println(" OK");
  }
  
  checkDeviceSettings();
  settings.lastSettingsCheck = millis();
  initializeDeviceStatus();
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║         SYSTEM READY               ║");
  Serial.println("║  Hold touch, count to 5            ║");
  Serial.println("║  Release to send SOS immediately   ║");
  Serial.println("║  OR                                ║");
  Serial.println("║  MPU6050 auto fall detection       ║");
  Serial.printf("║  SOS Duration: %d ms              ║\n", settings.sosDuration);
  Serial.printf("║  Buzzer: %s                    ║\n", settings.buzzerEnabled ? "ENABLED " : "DISABLED");
  Serial.printf("║  MPU6050: %s                   ║\n", mpuAvailable ? (settings.mpuEnabled ? "ACTIVE  " : "DISABLED") : "OFFLINE ");
  Serial.printf("║  Location: %.4f, %.4f         ║\n", FIXED_LAT, FIXED_LNG);
  Serial.printf("║  Time: %s              ║\n", getCurrentTimeString().c_str());
  Serial.println("║  Anti-Spam: ENABLED                ║");
  Serial.println("╚════════════════════════════════════╝\n");
  
  systemReady = true;
  lastWifiCheck = millis();
}

// ============== MAIN LOOP ==============
void loop() {
  unsigned long now = millis();
  
  if (now - settings.lastSettingsCheck > SETTINGS_CHECK_INTERVAL) {
    checkDeviceSettings();
    settings.lastSettingsCheck = now;
  }
  
  if (now - lastStatusUpdate > STATUS_UPDATE_INTERVAL) {
    updateDeviceStatus();
    lastStatusUpdate = now;
  }
  
  if (mpuAvailable && now - lastMpuUpdate > 5000) {
    updateMPUStatus();
    lastMpuUpdate = now;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiCheck > 5000) {
      lastWifiCheck = now;
      wifiReconnectAttempts++;
      Serial.println("WiFi lost! Attempt: " + String(wifiReconnectAttempts));
      if (wifiReconnectAttempts > 10) ESP.restart();
      WiFi.disconnect();
      delay(100);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      systemReady = false;
    }
    delay(100);
    return;
  }
  
  if (!Firebase.ready()) {
    if (now - lastWifiCheck > 10000) {
      Serial.println("Firebase not ready, reconnecting...");
      Firebase.begin(&config, &auth);
      lastWifiCheck = now;
    }
    delay(100);
    return;
  }
  
  if (!systemReady) {
    Serial.println("System ready!");
    systemReady = true;
    wifiReconnectAttempts = 0;
  }
  
  checkWebCommands();
  
  detectFall();
  
  if (isEmergency && (now - sosStartTime > SOS_TIMEOUT)) {
    Serial.println("Auto-reset after 5 minutes");
    clearSOS();
  }
  
  if (buzzerActive && (now - buzzerStartTime > BUZZER_DURATION)) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
  }
  
  if (!isEmergency && (now - lastAlertTime > 5000)) {
    handleTouch();
  }
  
  delay(10);
}

// ============== FIREBASE FUNCTIONS ==============
void initializeDeviceStatus() {
  if (!Firebase.ready()) return;
  FirebaseJson json;
  json.set("sos", false);
  json.set("sensor", "none");
  json.set("lat", FIXED_LAT);
  json.set("lng", FIXED_LNG);
  json.set("altitude", FIXED_ALTITUDE);
  json.set("gpsValid", false);
  json.set("satellites", 0);
  json.set("battery", -1);
  json.set("status", "online");
  json.set("fixQuality", 0);
  json.set("mpuStatus", mpuAvailable ? "connected" : "not_connected");
  json.set("deviceType", "ESP32-Demo-MPU");
  json.set("lastUpdate", getCurrentTimestamp());
  json.set("lastUpdateString", getCurrentTimeString());
  json.set("currentSosDuration", settings.sosDuration);
  json.set("currentBuzzerEnabled", settings.buzzerEnabled);
  json.set("mpuEnabled", settings.mpuEnabled);
  json.set("fallThreshold", settings.fallThreshold);
  // 🆕 ANTI-SPAM: Initialize notification flags
  json.set("alertNotified", false);
  json.set("alertId", 0);
  
  if (Firebase.RTDB.updateNode(&fbdo, "/devices/" + deviceID, &json)) {
    Serial.println("✓ Device initialized");
  } else {
    Serial.printf("✗ Init error: %s\n", fbdo.errorReason().c_str());
  }
}

// 🆕 FIXED: Removed alertNotified from status updates - prevents spam!
void updateDeviceStatus() {
  if (!Firebase.ready()) return;
  
  FirebaseJson json;
  json.set("lat", FIXED_LAT);
  json.set("lng", FIXED_LNG);
  json.set("altitude", FIXED_ALTITUDE);
  json.set("gpsValid", false);
  json.set("satellites", 0);
  json.set("fixQuality", 0);
  json.set("gpsTime", getCurrentTimeString());
  json.set("lastUpdate", getCurrentTimestamp());
  json.set("lastUpdateString", getCurrentTimeString());
  json.set("status", isEmergency ? "emergency" : "online");
  json.set("battery", -1);
  json.set("currentSosDuration", settings.sosDuration);
  json.set("currentBuzzerEnabled", settings.buzzerEnabled);
  json.set("mpuEnabled", settings.mpuEnabled);
  json.set("fallThreshold", settings.fallThreshold);
  // 🆕 CRITICAL FIX: Do NOT update alertNotified here!
  // Let React set it to true after sending Telegram
  // ESP32 only sets it during activateSOS() and clearSOS()
  
  Firebase.RTDB.updateNode(&fbdo, "/devices/" + deviceID, &json);
}

void saveAlertHistory(String alertType, unsigned long durationMs) {
  if (!Firebase.ready()) return;
  if (historySavedForCurrentSOS) return;
  
  String historyId = getCurrentTimestamp() + String(micros() % 1000);
  String historyPath = "/devices/" + deviceID + "/history/" + historyId;
  Serial.println("💾 Saving history: " + historyPath);
  
  FirebaseJson json;
  json.set("type", alertType);
  json.set("timestamp", getCurrentTimestamp());
  json.set("timestampString", getCurrentTimeString());
  json.set("lat", FIXED_LAT);
  json.set("lng", FIXED_LNG);
  json.set("altitude", FIXED_ALTITUDE);
  json.set("durationMs", (int)durationMs);
  json.set("durationSec", (int)(durationMs / 1000));
  json.set("resolved", false);
  json.set("sensor", currentSensor);
  json.set("timezone", "Asia/Kuala_Lumpur");
  json.set("sosDurationSetting", settings.sosDuration);
  json.set("buzzerEnabledSetting", settings.buzzerEnabled);
  
  if (mpuAvailable && alertType == "mpu_fall") {
    json.set("mpuAccelX", accelX);
    json.set("mpuAccelY", accelY);
    json.set("mpuAccelZ", accelZ);
    json.set("mpuTotalAccel", totalAccel);
    json.set("mpuTemperature", temperature);
    json.set("impactForce", totalAccel / 9.81);
  }
  
  if (Firebase.RTDB.setJSON(&fbdo, historyPath, &json)) {
    historySavedForCurrentSOS = true;
    Serial.println("✓ History saved!");
  } else {
    Serial.printf("✗ History error: %s\n", fbdo.errorReason().c_str());
  }
}

// ============== TOUCH HANDLING ==============
void handleTouch() {
  int reading = digitalRead(TOUCH_PIN);
  delay(10);
  if (digitalRead(TOUCH_PIN) != reading) return;
  
  int touchState = reading;
  unsigned long now = millis();
  static unsigned long lastDebug = 0;
  
  if (now - lastDebug > 2000) {
    Serial.printf("[STATUS] Touch:%s | SOS:%s | MPU:%s | Time:%s | Duration:%dms | Buzzer:%s\n", 
                  touchState == LOW ? "YES" : "NO",
                  isEmergency ? "ACTIVE" : "ready",
                  mpuAvailable ? (settings.mpuEnabled ? "ON" : "OFF") : "N/A",
                  getCurrentTimeString().c_str(),
                  settings.sosDuration,
                  settings.buzzerEnabled ? "ON" : "OFF");
    lastDebug = now;
  }
  
  if (touchState == LOW && lastTouchState == HIGH) {
    touchActive = true;
    touchBeginTime = now;
    countdownStep = 0;
    countdownDone = false;
    touchHeldDuration = 0;
    historySavedForCurrentSOS = false;
    Serial.println("\n[TOUCH] Hold and count...");
    Serial.printf("   Target: %d ms\n", settings.sosDuration);
    Serial.println("   Time: " + getCurrentTimeString());
    tickBuzzer();
  }
  
  if (touchState == HIGH && lastTouchState == LOW) {
    touchHeldDuration = now - touchBeginTime;
    touchActive = false;
    Serial.printf("[TOUCH] Released after %lums\n", touchHeldDuration);
    
    int targetSeconds = settings.sosDuration / 1000;
    int secondsHeld = touchHeldDuration / 1000;
    if (secondsHeld > targetSeconds) secondsHeld = targetSeconds;
    
    if (touchHeldDuration >= (unsigned long)settings.sosDuration) {
      Serial.printf("  ✓ Full %ds - SOS!\n", targetSeconds);
      activateSOS("touch_release_full", touchHeldDuration);
    } else if (touchHeldDuration >= 1000) {
      Serial.printf("  ⚠ %ds/%ds - Partial SOS\n", secondsHeld, targetSeconds);
      activateSOS("touch_release_partial", touchHeldDuration);
    } else {
      Serial.println("  ✗ Cancelled");
      tickBuzzer();
      historySavedForCurrentSOS = false;
    }
    countdownStep = 0;
    countdownDone = false;
  }
  
  if (touchActive && touchState == LOW && !countdownDone) {
    unsigned long held = now - touchBeginTime;
    int targetSeconds = settings.sosDuration / 1000;
    int currentSecond = held / 1000;
    
    if (currentSecond > countdownStep && currentSecond <= targetSeconds) {
      countdownStep = currentSecond;
      Serial.printf("  %d/%d...\n", currentSecond, targetSeconds);
      tickBuzzer();
    }
    
    if (held >= (unsigned long)settings.sosDuration && !countdownDone) {
      countdownDone = true;
      Serial.printf("  %d! Release to send!\n", targetSeconds);
      tickBuzzer();
    }
  }
  
  lastTouchState = touchState;
}

// ============== BUZZER ==============
void tickBuzzer() {
  if (!settings.buzzerEnabled) return;
  digitalWrite(BUZZER_PIN, HIGH);
  delay(50);
  digitalWrite(BUZZER_PIN, LOW);
}

void startBuzzer() {
  if (!settings.buzzerEnabled) {
    buzzerActive = false;
    return;
  }
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerActive = true;
  buzzerStartTime = millis();
}

// ============== SOS & FIREBASE ==============
void activateSOS(String reason, unsigned long durationMs) {
  // Prevent duplicate activation
  if (isEmergency) {
    Serial.println("[SOS] Already in emergency state - skipping duplicate activation");
    return;
  }
  
  isEmergency = true;
  currentSensor = reason;
  sosStartTime = millis();
  lastAlertTime = millis();
  int secondsHeld = durationMs / 1000;
  
  // Generate unique alert ID
  lastAlertId = millis();
  
  Serial.println("\n🚨 SOS TRIGGERED!");
  Serial.printf("📍 %.6f, %.6f\n", FIXED_LAT, FIXED_LNG);
  Serial.printf("🔔 Sensor: %s\n", reason.c_str());
  Serial.printf("🆔 Alert ID: %lu\n", lastAlertId);
  
  if (reason == "mpu_fall") {
    Serial.printf("💥 Impact Force: %.2f m/s² (%.2f g)\n", totalAccel, totalAccel/9.81);
    Serial.printf("🌡️ Temperature: %.1f°C\n", temperature);
  } else {
    Serial.printf("⏱️ %ds held (target: %ds)\n", secondsHeld, settings.sosDuration / 1000);
  }
  
  Serial.printf("🔊 Buzzer: %s\n", settings.buzzerEnabled ? "ON" : "OFF");
  Serial.printf("🕐 %s\n", getCurrentTimeString().c_str());
  
  startBuzzer();
  saveAlertHistory(reason, durationMs);
  
  delay(100);
  
  FirebaseJson json;
  json.set("sos", true);
  json.set("sensor", reason);
  json.set("lat", FIXED_LAT);
  json.set("lng", FIXED_LNG);
  json.set("altitude", FIXED_ALTITUDE);
  json.set("gpsValid", false);
  json.set("satellites", 0);
  json.set("gpsTime", getCurrentTimeString());
  json.set("sosTriggeredAt", getCurrentTimestamp());
  json.set("sosTriggeredAtString", getCurrentTimeString());
  json.set("status", "emergency");
  json.set("touchDurationMs", (int)durationMs);
  json.set("touchDurationSec", secondsHeld);
  json.set("lastUpdate", getCurrentTimestamp());
  json.set("lastUpdateString", getCurrentTimeString());
  json.set("sosDurationSetting", settings.sosDuration);
  json.set("buzzerEnabledSetting", settings.buzzerEnabled);
  // 🆕 ANTI-SPAM: Set fresh alert flags
  json.set("alertNotified", false);  // Reset for new alert
  json.set("alertId", (int)lastAlertId);  // Unique ID
  
  if (reason == "mpu_fall" && mpuAvailable) {
    json.set("mpuAccelX", accelX);
    json.set("mpuAccelY", accelY);
    json.set("mpuAccelZ", accelZ);
    json.set("mpuTotalAccel", totalAccel);
    json.set("mpuGyroX", gyroX);
    json.set("mpuGyroY", gyroY);
    json.set("mpuGyroZ", gyroZ);
    json.set("mpuTemperature", temperature);
    json.set("impactForceG", totalAccel / 9.81);
  }
  
  if (Firebase.RTDB.updateNode(&fbdo, "/devices/" + deviceID, &json)) {
    Serial.println("✓ SOS sent to Firebase!");
  } else {
    Serial.printf("✗ SOS error: %s\n", fbdo.errorReason().c_str());
  }
}

void checkWebCommands() {
  if (!Firebase.ready()) return;
  
  if (Firebase.RTDB.getBool(&fbdo, "/devices/" + deviceID + "/sos")) {
    bool webSOS = fbdo.boolData();
    
    if (Firebase.RTDB.getString(&fbdo, "/devices/" + deviceID + "/sensor")) {
      String webSensor = fbdo.stringData();
      
      if (webSOS && !isEmergency && webSensor == "web_test") {
        Serial.println("[WEB] Test at " + getCurrentTimeString());
        
        // Check if already notified to prevent duplicate
        if (Firebase.RTDB.getBool(&fbdo, "/devices/" + deviceID + "/alertNotified")) {
          bool alreadyNotified = fbdo.boolData();
          if (alreadyNotified) {
            Serial.println("[WEB] Test already notified, skipping");
            return;
          }
        }
        
        isEmergency = true;
        currentSensor = "web_test";
        sosStartTime = millis();
        lastAlertTime = millis();
        lastAlertId = millis();
        startBuzzer();
        saveAlertHistory("web_test", 0);
        
        delay(100);
        FirebaseJson updateJson;
        updateJson.set("sos", true);
        updateJson.set("sensor", "web_test");
        updateJson.set("lat", FIXED_LAT);
        updateJson.set("lng", FIXED_LNG);
        updateJson.set("gpsValid", false);
        updateJson.set("sosTriggeredAt", getCurrentTimestamp());
        updateJson.set("sosTriggeredAtString", getCurrentTimeString());
        updateJson.set("lastUpdate", getCurrentTimestamp());
        updateJson.set("alertNotified", false);
        updateJson.set("alertId", (int)lastAlertId);
        Firebase.RTDB.updateNode(&fbdo, "/devices/" + deviceID, &updateJson);
      }
      
      if (!webSOS && isEmergency) {
        Serial.println("[WEB] Reset received");
        clearSOS();
      }
    }
  }
}

void clearSOS() {
  isEmergency = false;
  currentSensor = "none";
  touchActive = false;
  countdownStep = 0;
  countdownDone = false;
  historySavedForCurrentSOS = false;
  lastAlertId = 0;
  
  fallState = STATE_NORMAL;
  impactDetected = false;
  
  delay(10);
  lastTouchState = digitalRead(TOUCH_PIN);
  digitalWrite(BUZZER_PIN, LOW);
  buzzerActive = false;
  
  FirebaseJson json;
  json.set("sos", false);
  json.set("sensor", "none");
  json.set("status", "online");
  json.set("sosClearedAt", getCurrentTimestamp());
  json.set("sosClearedAtString", getCurrentTimeString());
  json.set("lastUpdate", getCurrentTimestamp());
  json.set("lastUpdateString", getCurrentTimeString());
  json.set("alertNotified", false);  // Reset for next time
  json.set("alertId", 0);
  
  Firebase.RTDB.updateNode(&fbdo, "/devices/" + deviceID, &json);
  Serial.println("✓ SOS cleared at " + getCurrentTimeString());
  lastAlertTime = millis() - 5000;
}