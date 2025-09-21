#include <WiFi.h>
#include <PubSubClient.h>
#include <BluetoothSerial.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <cstring>

#define WIFI_STA_SSID "nihongogajouzudesune"
#define WIFI_STA_PASS "nihongogajouzudesune"
#define WIFI_AP_SSID  "ESP32_CarAP"
#define WIFI_AP_PASS  "guthib123"

#define MQTT_SERVER_IP "192.168.33.79"
#define MQTT_PORT 1883
#define MQTT_TOPIC_CMD   "car/cmd"
#define MQTT_TOPIC_REPLY "car/response"

#define DHTPIN 15
#define DHTTYPE DHT11

#define TRIG_PIN 19
#define ECHO_PIN 34
#define OBSTACLE_CM 10

#define FRONT_LEFT_PWM_CH   0
#define FRONT_RIGHT_PWM_CH  1
#define REAR_LEFT_PWM_CH    2
#define REAR_RIGHT_PWM_CH   3

#define FRONT_LEFT_PWM_PIN   12
#define FRONT_RIGHT_PWM_PIN  13
#define REAR_LEFT_PWM_PIN     4
#define REAR_RIGHT_PWM_PIN    5

#define FRONT_LEFT_DIR_PIN1  26
#define FRONT_LEFT_DIR_PIN2  27
#define FRONT_RIGHT_DIR_PIN1 32
#define FRONT_RIGHT_DIR_PIN2 33
#define REAR_LEFT_DIR_PIN1   16
#define REAR_LEFT_DIR_PIN2   17
#define REAR_RIGHT_DIR_PIN1  23
#define REAR_RIGHT_DIR_PIN2  25

#define LED_BLUE_PIN 2
#define LED_RED_PIN  18
#define BUTTON_INT_PIN 14

#define BATT_PIN 35

#define OLED_SDA 21
#define OLED_SCL 22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

#define BATT_V_MIN 6.0f
#define BATT_V_MAX 8.4f

DHT dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BluetoothSerial BT;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

volatile bool emergencyInterrupt = false;
volatile unsigned long lastInterruptMillis = 0;

static char latestCmdBuf[16];
static bool hasCmd = false;
SemaphoreHandle_t cmdMutex;

char motionState[24] = "Stopped";
int requestedSpeed = 150;
float lastTemp = 0, lastHum = 0, lastDistance = 999;

char mqttClientId[32];

float batteryVoltage = 0.0f;
float batteryPercent = 100.0f;
bool batteryLowNotified = false;

void setupWiFi();
void mqttCallback(char *topic, byte *message, unsigned int length);
void commsTask(void *pvParameters);
void sensorsTask(void *pvParameters);
void controlTask(void *pvParameters);
void stopMotors();
void setMotorAxle(int pwmVal, int dir,int leftChannel, int leftDir1, int leftDir2,int rightChannel, int rightDir1, int rightDir2);
void setMotorSide(int pwmVal, int dir, int pwmChannel, int dirPin1, int dirPin2);
void setAxles(int speedFront, int dirFront, int speedRear, int dirRear);
float readUltrasonicCM();
void IRAM_ATTR handleButtonInterrupt();
void showOLED();
void processCommandC(const char *cmd);
void publishAck(const char *cmd, const char *result);
void checkHeapAndRecover();
void readBattery();

void setup() {
  Serial.begin(115200);

  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_BLUE_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  ledcSetup(FRONT_LEFT_PWM_CH, 20000, 8);
  ledcAttachPin(FRONT_LEFT_PWM_PIN, FRONT_LEFT_PWM_CH);
  ledcWrite(FRONT_LEFT_PWM_CH, 0);

  ledcSetup(FRONT_RIGHT_PWM_CH, 20000, 8);
  ledcAttachPin(FRONT_RIGHT_PWM_PIN, FRONT_RIGHT_PWM_CH);
  ledcWrite(FRONT_RIGHT_PWM_CH, 0);

  ledcSetup(REAR_LEFT_PWM_CH, 20000, 8);
  ledcAttachPin(REAR_LEFT_PWM_PIN, REAR_LEFT_PWM_CH);
  ledcWrite(REAR_LEFT_PWM_CH, 0);

  ledcSetup(REAR_RIGHT_PWM_CH, 20000, 8);
  ledcAttachPin(REAR_RIGHT_PWM_PIN, REAR_RIGHT_PWM_CH);
  ledcWrite(REAR_RIGHT_PWM_CH, 0);

  pinMode(FRONT_LEFT_DIR_PIN1, OUTPUT);
  pinMode(FRONT_LEFT_DIR_PIN2, OUTPUT);
  pinMode(FRONT_RIGHT_DIR_PIN1, OUTPUT);
  pinMode(FRONT_RIGHT_DIR_PIN2, OUTPUT);
  pinMode(REAR_LEFT_DIR_PIN1, OUTPUT);
  pinMode(REAR_LEFT_DIR_PIN2, OUTPUT);
  pinMode(REAR_RIGHT_DIR_PIN1, OUTPUT);
  pinMode(REAR_RIGHT_DIR_PIN2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(BUTTON_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_INT_PIN), handleButtonInterrupt, FALLING);

  dht.begin();
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  if (!BT.begin("ESP32_Car")) {
    Serial.println("BT failed");
  }

  uint64_t mac = ESP.getEfuseMac();
  snprintf(mqttClientId, sizeof(mqttClientId), "ESP32_Car-%llX", (unsigned long long)mac);
  mqttClientId[sizeof(mqttClientId) - 1] = 0;
  Serial.printf("MQTT clientId: %s\n", mqttClientId);

  setupWiFi();
  mqttClient.setServer(MQTT_SERVER_IP, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  cmdMutex = xSemaphoreCreateMutex();

  xTaskCreate(commsTask,   "commsTask",   4096, NULL, 2, NULL);
  xTaskCreate(sensorsTask, "sensorsTask", 4096, NULL, 1, NULL);
  xTaskCreate(controlTask, "controlTask", 4096, NULL, 2, NULL);
}

void commsTask(void *pvParameters) {
  static unsigned long nextMqttAttempt = 0;
  static int reconnectAttempts = 0;

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected! Reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
      int retries = 0;
      while (WiFi.status() != WL_CONNECTED && retries < 10) {
        delay(1000);
        Serial.print(".");
        retries++;
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to WIFI_STA_SSID!");
      } else {
        Serial.println("\nFailed to connect to WIFI_STA_SSID. Trying WIFI_AP_SSID...");
        WiFi.disconnect();
        WiFi.begin(WIFI_AP_SSID, WIFI_AP_PASS);
        retries = 0;
        while (WiFi.status() != WL_CONNECTED && retries < 10) {
          delay(1000);
          Serial.print(".");
          retries++;
        }
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("\nConnected to WIFI_AP_SSID!");
        } else {
          Serial.println("\nFailed to connect to both SSIDs.");
        }
      }
    }

    bool wifiConnected = (WiFi.status() == WL_CONNECTED);
    digitalWrite(LED_BLUE_PIN, wifiConnected ? HIGH : LOW);

    if (wifiConnected) {
      unsigned long now = millis();
      if (mqttClient.connected()) {
        mqttClient.loop();
        reconnectAttempts = 0;
        nextMqttAttempt = 0;
      } else {
        if (nextMqttAttempt == 0 || now >= nextMqttAttempt) {
          Serial.println("MQTT attempting connect...");
          if (mqttClient.connect(mqttClientId)) {
            mqttClient.subscribe(MQTT_TOPIC_CMD, 1);
            Serial.println("MQTT connected!");
            reconnectAttempts = 0;
            nextMqttAttempt = 0;
          } else {
            reconnectAttempts++;
            int capAttempts = (reconnectAttempts > 6) ? 6 : reconnectAttempts;
            int backoffSec = (1 << (capAttempts - 1));
            if (backoffSec > 30) backoffSec = 30;
            int jitter = random(0, 500);
            nextMqttAttempt = now + (unsigned long)backoffSec * 1000UL + jitter;
            Serial.printf("MQTT connect failed (state=%d). retry in %d s (+%d ms jitter)\n", mqttClient.state(), backoffSec, jitter);
          }
        }
      }
    }

    if (BT.available()) {
      char buf[16]; memset(buf, 0, sizeof(buf));
      size_t len = BT.readBytesUntil('\n', buf, sizeof(buf) - 1);
      if (len > 0) {
        while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == '\n' || buf[len - 1] == ' ')) { buf[--len] = 0; }
        size_t start = 0; while (buf[start] == ' ' && start < len) start++;
        if (start > 0) { memmove(buf, buf + start, len - start + 1); len -= start; }
        if (len > 0) {
          if (xSemaphoreTake(cmdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            strncpy(latestCmdBuf, buf, sizeof(latestCmdBuf) - 1);
            latestCmdBuf[sizeof(latestCmdBuf) - 1] = 0;
            hasCmd = true;
            xSemaphoreGive(cmdMutex);
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void sensorsTask(void *pvParameters) {
  unsigned long lastDHTReadTime = 0;
  const unsigned long DHT_READ_INTERVAL = 2000;

  for (;;) {
    unsigned long now = millis();
    if (now - lastDHTReadTime >= DHT_READ_INTERVAL) {
      float t = dht.readTemperature(); if (!isnan(t)) lastTemp = t;
      float h = dht.readHumidity();    if (!isnan(h)) lastHum = h;
      lastDHTReadTime = now;
    }

    lastDistance = readUltrasonicCM();
    if (lastDistance <= OBSTACLE_CM && lastDistance > 0) {
      stopMotors();
      strncpy(motionState, "Obstacle", sizeof(motionState)-1);
      motionState[sizeof(motionState)-1]=0;
      publishAck("obstacle", motionState);
      Serial.println("Obstacle detected! Motors stopped.");
    }

    readBattery();
    if (batteryPercent < 20.0) {
      if (!batteryLowNotified) {
        publishAck("battery", "low");
        batteryLowNotified = true;
      }
    } else {
      batteryLowNotified = false;
    }

    showOLED();
    checkHeapAndRecover();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void controlTask(void *pvParameters) {
  for (;;) {
    static bool lastEmergencyState = false;
    if (emergencyInterrupt) {
      digitalWrite(LED_RED_PIN, HIGH);
      digitalWrite(LED_BLUE_PIN, LOW);
      stopMotors();
    } else {
      digitalWrite(LED_RED_PIN, LOW);
    }

    if (emergencyInterrupt != lastEmergencyState) {
      if (emergencyInterrupt) {
        Serial.println("[ALERT] Emergency Interrupt Activated!");
        publishAck("interrupt", "on");
      } else {
        Serial.println("[INFO] Emergency Interrupt Cleared");
        publishAck("interrupt", "off");
      }
      lastEmergencyState = emergencyInterrupt;
    }

    if (xSemaphoreTake(cmdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (hasCmd) {
        char cmdLocal[16];
        strncpy(cmdLocal, latestCmdBuf, sizeof(cmdLocal) - 1);
        cmdLocal[sizeof(cmdLocal) - 1] = 0;
        hasCmd = false;
        latestCmdBuf[0] = 0;
        xSemaphoreGive(cmdMutex);

        processCommandC(cmdLocal);
        Serial.printf("cmd=%s -> motionState=%s\n", cmdLocal, motionState);
        publishAck(cmdLocal, motionState);
      } else {
        xSemaphoreGive(cmdMutex);
      }
    }

    if (!emergencyInterrupt){
      int dir = 0;
      if (strcmp(motionState, "Forward") == 0)      dir = 1;
      else if (strcmp(motionState, "Backward") == 0) dir = 2;

      if (strcmp(motionState, "Left") == 0) {
        setMotorSide(requestedSpeed, 2, FRONT_LEFT_PWM_CH, FRONT_LEFT_DIR_PIN1, FRONT_LEFT_DIR_PIN2);
        setMotorSide(requestedSpeed, 2, REAR_LEFT_PWM_CH,  REAR_LEFT_DIR_PIN1,  REAR_LEFT_DIR_PIN2);
        setMotorSide(requestedSpeed, 1, FRONT_RIGHT_PWM_CH,FRONT_RIGHT_DIR_PIN1,FRONT_RIGHT_DIR_PIN2);
        setMotorSide(requestedSpeed, 1, REAR_RIGHT_PWM_CH, REAR_RIGHT_DIR_PIN1, REAR_RIGHT_DIR_PIN2);
      } else if (strcmp(motionState, "Right") == 0) {
        setMotorSide(requestedSpeed, 1, FRONT_LEFT_PWM_CH, FRONT_LEFT_DIR_PIN1,  FRONT_LEFT_DIR_PIN2);
        setMotorSide(requestedSpeed, 1, REAR_LEFT_PWM_CH,  REAR_LEFT_DIR_PIN1,   REAR_LEFT_DIR_PIN2);
        setMotorSide(requestedSpeed, 2, FRONT_RIGHT_PWM_CH,FRONT_RIGHT_DIR_PIN1, FRONT_RIGHT_DIR_PIN2);
        setMotorSide(requestedSpeed, 2, REAR_RIGHT_PWM_CH, REAR_RIGHT_DIR_PIN1,  REAR_RIGHT_DIR_PIN2);
      } else {
        setAxles(requestedSpeed, dir, requestedSpeed, dir);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void processCommandC(const char *cmd) {
  if (cmd == NULL) return;

  if      (strcasecmp(cmd, "w") == 0) { strncpy(motionState, "Forward",  sizeof(motionState)-1); }
  else if (strcasecmp(cmd, "s") == 0) { strncpy(motionState, "Backward", sizeof(motionState)-1); }
  else if (strcasecmp(cmd, "a") == 0) { strncpy(motionState, "Left",     sizeof(motionState)-1); }
  else if (strcasecmp(cmd, "d") == 0) { strncpy(motionState, "Right",    sizeof(motionState)-1); }
  else if (strcasecmp(cmd, "Idle") == 0 || strcasecmp(cmd, "Stop") == 0) {
    strncpy(motionState, "Stopped", sizeof(motionState)-1);
  }
  else if (strcasecmp(cmd, "i") == 0) {
    strncpy(motionState, "Interrupt", sizeof(motionState)-1);
    emergencyInterrupt = !emergencyInterrupt;
  }
  else {
    int v = atoi(cmd);
    if (v != 0) {
      int absV = v > 0 ? v : -v;
      requestedSpeed = constrain(absV, 0, 255);
      if (v > 0) strncpy(motionState, "Forward",  sizeof(motionState)-1);
      else       strncpy(motionState, "Backward", sizeof(motionState)-1);
    }
  }
  motionState[sizeof(motionState)-1] = 0;
}

void publishAck(const char *cmd, const char *result) {
  if (mqttClient.connected()) {
    char buff[64];
    snprintf(buff, sizeof(buff), "%s %s", cmd, result);
    mqttClient.publish(MQTT_TOPIC_REPLY, buff);
  }
}

void stopMotors() {
  setMotorAxle(0, 0, FRONT_LEFT_PWM_CH, FRONT_LEFT_DIR_PIN1, FRONT_LEFT_DIR_PIN2,
                     FRONT_RIGHT_PWM_CH, FRONT_RIGHT_DIR_PIN1, FRONT_RIGHT_DIR_PIN2);
  setMotorAxle(0, 0, REAR_LEFT_PWM_CH,  REAR_LEFT_DIR_PIN1,  REAR_LEFT_DIR_PIN2,
                     REAR_RIGHT_PWM_CH, REAR_RIGHT_DIR_PIN1, REAR_RIGHT_DIR_PIN2);
}

void setAxles(int speedFront, int dirFront, int speedRear, int dirRear) {
  setMotorAxle(speedFront, dirFront, FRONT_LEFT_PWM_CH, FRONT_LEFT_DIR_PIN1, FRONT_LEFT_DIR_PIN2,
                                     FRONT_RIGHT_PWM_CH, FRONT_RIGHT_DIR_PIN1, FRONT_RIGHT_DIR_PIN2);
  setMotorAxle(speedRear,  dirRear,  REAR_LEFT_PWM_CH,  REAR_LEFT_DIR_PIN1,  REAR_LEFT_DIR_PIN2,
                                     REAR_RIGHT_PWM_CH, REAR_RIGHT_DIR_PIN1, REAR_RIGHT_DIR_PIN2);
}

void setMotorAxle(int pwmVal, int dir,
                  int leftChannel, int leftDir1, int leftDir2,
                  int rightChannel, int rightDir1, int rightDir2) {
  pwmVal = constrain(pwmVal, 0, 255);
  if (dir == 0) {
    digitalWrite(leftDir1, LOW);  digitalWrite(leftDir2, LOW);
    digitalWrite(rightDir1, LOW); digitalWrite(rightDir2, LOW);
    ledcWrite(leftChannel, 0);    ledcWrite(rightChannel, 0);
  } else if (dir == 1) {
    digitalWrite(leftDir1, HIGH); digitalWrite(leftDir2, LOW);
    digitalWrite(rightDir1, HIGH);digitalWrite(rightDir2, LOW);
    ledcWrite(leftChannel, pwmVal); ledcWrite(rightChannel, pwmVal);
  } else if (dir == 2) {
    digitalWrite(leftDir1, LOW);  digitalWrite(leftDir2, HIGH);
    digitalWrite(rightDir1, LOW); digitalWrite(rightDir2, HIGH);
    ledcWrite(leftChannel, pwmVal); ledcWrite(rightChannel, pwmVal);
  }
}

void setMotorSide(int pwmVal, int dir, int pwmChannel, int dirPin1, int dirPin2) {
  pwmVal = constrain(pwmVal, 0, 255);
  if (dir == 0) {
    digitalWrite(dirPin1, LOW); digitalWrite(dirPin2, LOW);
    ledcWrite(pwmChannel, 0);
  } else if (dir == 1) {
    digitalWrite(dirPin1, HIGH); digitalWrite(dirPin2, LOW);
    ledcWrite(pwmChannel, pwmVal);
  } else if (dir == 2) {
    digitalWrite(dirPin1, LOW); digitalWrite(dirPin2, HIGH);
    ledcWrite(pwmChannel, pwmVal);
  }
}

float readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 20000);
  if (dur == 0) return 999;
  return (dur / 2.0) / 29.1;
}

void IRAM_ATTR handleButtonInterrupt() {
  unsigned long now = millis();
  if (now - lastInterruptMillis < 300) return;
  lastInterruptMillis = now;
  emergencyInterrupt = !emergencyInterrupt;
}

void checkHeapAndRecover() {
  size_t freeHeap = ESP.getFreeHeap();
  static unsigned long lastLogTime = 0;
  static size_t lowestSeen = SIZE_MAX;
  if (freeHeap < lowestSeen) lowestSeen = freeHeap;
  unsigned long now = millis();
  if (now - lastLogTime >= 10000) {
    Serial.printf("Free heap: %u bytes, lowest seen: %u\n", (unsigned)freeHeap, (unsigned)lowestSeen);
    lastLogTime = now;
  }
  if (freeHeap < 18000) {
    Serial.println("Free heap too low -> restarting to recover");
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP.restart();
  }
}

void readBattery() {
  static unsigned long lastStopMs = 0;
  static bool wasStopped = false;
  bool isStopped = (strcmp(motionState, "Stopped") == 0);
  if (isStopped) {
    if (!wasStopped) {
      lastStopMs = millis();
    }
    if (millis() - lastStopMs >= 400) {
      uint32_t sum = 0;
      const int samples = 16;
      for (int i = 0; i < samples; i++) {
        sum += analogRead(BATT_PIN);
        delay(5);
      }
      float adcAvg = sum * 1.0f / samples;
      float vAdc = adcAvg * (3.3f / 4095.0f);
      float vBat = vAdc * ((10000.0f + 4700.0f) / 4700.0f);
      batteryVoltage = vBat;
      float pct = (batteryVoltage - BATT_V_MIN) / (BATT_V_MAX - BATT_V_MIN) * 100.0f;
      if (pct > 100.0f) pct = 100.0f;
      if (pct < 0.0f) pct = 0.0f;
      batteryPercent = pct;
    }
  }
  wasStopped = isStopped;
}

void showOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("State: %s\n", motionState);
  display.printf("Temp: %.1fC Hum:%.1f%%\n", lastTemp, lastHum);
  display.printf("Dist: %.1f cm\n", lastDistance);
  display.printf("Speed: %d\n", requestedSpeed);
  display.printf("Emergency: %s\n", emergencyInterrupt ? "ON" : "OFF");
  display.printf("Batt: %.0f%% %.1fV\n", batteryPercent, batteryVoltage);
  display.display();
}

void mqttCallback(char *topic, byte *message, unsigned int length) {
  char buf[16];
  size_t copyLen = (length < sizeof(buf) - 1) ? length : sizeof(buf) - 1;
  memcpy(buf, message, copyLen);
  buf[copyLen] = 0;
  Serial.printf("MQTT message received: topic=%s, message=%s\n", topic, buf);
  publishAck("rx", buf);
  if (strcmp(topic, MQTT_TOPIC_CMD) == 0) {
    if (xSemaphoreTake(cmdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      strncpy(latestCmdBuf, buf, sizeof(latestCmdBuf) - 1);
      latestCmdBuf[sizeof(latestCmdBuf) - 1] = 0;
      hasCmd = true;
      xSemaphoreGive(cmdMutex);
    }
  }
}

void setupWiFi() {
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
  Serial.print("Connecting to WiFi");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    retry++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("Wifi: ESP32_CarAP");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect WiFi");
  }
}

void loop() {}
