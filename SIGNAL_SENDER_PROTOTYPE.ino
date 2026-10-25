#include <esp_now.h>
#include <WiFi.h>

#define BUTTON_PIN 4
#define BUZZER_PIN 14

// Receiver MAC Address
uint8_t receiverAddress[] = {
  0xC8, 0x2E, 0x18, 0x8E, 0xBC, 0x5C
};

typedef struct {
  bool sos;
} Message;

Message msg;

unsigned long pressTime = 0;
bool buttonHeld = false;

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);   // ✅ safer for buttons
  pinMode(BUZZER_PIN, OUTPUT);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
    return;
  }

  Serial.println("✅ Transmitter Ready");
}

void loop() {
  // Button pressed = LOW (because INPUT_PULLUP)
  if (digitalRead(BUTTON_PIN) == LOW) {

    if (!buttonHeld) {
      pressTime = millis();
      buttonHeld = true;
      Serial.println("⏳ Button held...");
    }

    // Held for 2 seconds
    if (millis() - pressTime >= 2000) {

      Serial.println("🚨 SOS SENT");

      // Passive buzzer sound
      tone(BUZZER_PIN, 2000);   // 2 kHz
      delay(500);
      noTone(BUZZER_PIN);

      msg.sos = true;
      esp_now_send(receiverAddress, (uint8_t *)&msg, sizeof(msg));

      delay(500);               // prevent multiple sends
      buttonHeld = false;
    }

  } else {
    buttonHeld = false;
  }
}
