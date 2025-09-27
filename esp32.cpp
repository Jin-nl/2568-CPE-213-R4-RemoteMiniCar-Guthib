#include <WiFi.h>
#include <PubSubClient.h>
#include <BluetoothSerial.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <ESP32Servo.h>
#include <cstring>
#include "esp_bt.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>

#define MQTT_PORT 1883
#define MQTT_TOPIC_CMD "car/cmd"
#define MQTT_TOPIC_REPLY "car/response"
#define MQTT_SERVER_IP "192.168.33.79"

#define TRIG_PIN 19
#define ECHO_PIN 34
#define OB_MIN_CM 4
#define OB_MAX_CM 200
#define OB_CONFIRM_COUNT 2
#define OB_RELEASE_COUNT 3
#define OB_HYSTERESIS_CM 8
#define OB_AUTO_RESUME 0
#define OB_BACK_SPEED 200
#define OB_ENABLE_TURN_AFTER_BACK 0
#define OB_TURN_SPEED 200
#define OB_TURN_TIME_MS 400
#define OB_TURN_DIR_RIGHT 1
#define OB_HARD_STOP_CM 8
#define OB_BACK_BURST_MS 260
#define OB_BASE_CM 14
#define OB_K_PER_PWM 0.05f
#define OBSTACLE_CM 18

#define FRONT_LEFT_PWM_CH 0
#define FRONT_RIGHT_PWM_CH 1
#define REAR_LEFT_PWM_CH 2
#define REAR_RIGHT_PWM_CH 3

#define FRONT_LEFT_PWM_PIN 12
#define FRONT_RIGHT_PWM_PIN 13
#define REAR_LEFT_PWM_PIN 4
#define REAR_RIGHT_PWM_PIN 5

#define FRONT_LEFT_DIR_PIN1 26
#define FRONT_LEFT_DIR_PIN2 27
#define FRONT_RIGHT_DIR_PIN1 32
#define FRONT_RIGHT_DIR_PIN2 33
#define REAR_LEFT_DIR_PIN1 16
#define REAR_LEFT_DIR_PIN2 17
#define REAR_RIGHT_DIR_PIN1 23
#define REAR_RIGHT_DIR_PIN2 25

#define LED_BLUE_PIN 2
#define LED_RED_PIN 18
#define BUTTON_INT_PIN 14

#define OLED_SDA 21
#define OLED_SCL 22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

#define USE_SINGLE_BATT 1
#define PIN_BATT_PACK 36
#define BATT_PACK_V_MIN 6.6f
#define BATT_PACK_V_MAX 8.4f
#define R_TOP 10000.0f
#define R_BOT 4700.0f

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BluetoothSerial BT;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiUDP udp;

static bool udpReady = false;
static uint16_t brokerPort = MQTT_PORT;
static char brokerHost[32] = MQTT_SERVER_IP;

bool isIPv4(const char *s)
{
  if (!s || !*s)
    return false;
  int a, b, c, d;
  char tail;
  if (sscanf(s, "%d.%d.%d.%d%c", &a, &b, &c, &d, &tail) != 4)
    return false;
  if ((a | b | c | d) < 0)
    return false;
  if (a > 255 || b > 255 || c > 255 || d > 255)
    return false;
  return true;
}

bool obstacleActive = false;
int obConfirm = 0, obRelease = 0;
char prevStateBeforeOb[24] = "Stopped";
enum ObPhase
{
  OB_IDLE = 0,
  OB_BACK = 1,
  OB_TURN = 2
};
ObPhase obPhase = OB_IDLE;
unsigned long obPhaseUntil = 0;
int obTurnDir = OB_TURN_DIR_RIGHT ? 1 : 0;

volatile bool emergencyInterrupt = false;
volatile unsigned long lastInterruptMillis = 0;
char motionState[24] = "Stopped";
int requestedSpeed = 255;
int savedSpeedBeforeAvoid = 255;

float lastDistance = 999;
bool distFresh = false;
unsigned long distFreshMs = 0;
static const unsigned long SENSOR_VALID_MS = 200;

float motorVoltage = 0.0f, motorPercent = 100.0f;
float espVoltage = 0.0f, espPercent = 100.0f;
bool motorLowNotified = false, espLowNotified = false;
#if USE_SINGLE_BATT
float packVoltage = 0.0f, packPercent = 100.0f;
#endif

static volatile bool wifiConnected = false, btConnected = false, lastWifiConnected = false, lastBtConnected = false;
unsigned long lastBlueBlink = 0;
bool blueOn = false;
unsigned long lastWiFiTryMs = 0, lastMQTTAttempt = 0;
int reconnectAttempts = 0;
const unsigned long WIFI_RETRY_MS = 3000;
const unsigned long MQTT_RETRY_MIN_MS = 500, MQTT_RETRY_MAX_MS = 30000;
static char mqttClientId[32];

enum WiFiFSM
{
  WIFI_IDLE,
  WIFI_TRY_BEGIN,
  WIFI_WAIT_CONN
};
static WiFiFSM wifiFsm = WIFI_IDLE;
static int wifiIdx = 0;
static unsigned long wifiPhaseStart = 0;
static const unsigned long WIFI_PER_SSID_MS = 2500;

static volatile bool pcOnline = false;
static unsigned long lastPcBeatMs = 0;
static unsigned long lastCmdMs = 0;
static const unsigned long PC_TIMEOUT_MS = 2000;
static const unsigned long CMD_TIMEOUT_MS = 900;

static inline void makeClientId()
{
  uint64_t mac = ESP.getEfuseMac();
  snprintf(mqttClientId, sizeof(mqttClientId), "ESP32_Car-%012llX", (unsigned long long)mac);
}

void setupWiFi();
static void wifiConnectTick();
static bool discoverTick();
bool discoverBroker();
void mqttCallback(char *topic, byte *message, unsigned int length);
void commsTask(void *pvParameters);
void sensorsTask(void *pvParameters);
void controlTask(void *pvParameters);
void stopMotors();
void setMotorAxle(int pwmVal, int dir, int leftChannel, int leftDir1, int leftDir2, int rightChannel, int rightDir1, int rightDir2);
void setMotorSide(int pwmVal, int dir, int pwmChannel, int dirPin1, int dirPin2);
void setAxles(int speedFront, int dirFront, int speedRear, int dirRear);
bool readUltrasonicCM_ok(float *out_cm);
void IRAM_ATTR handleButtonInterrupt();
void showOLED();
void checkHeapAndRecover();
void publishAck(const char *cmd, const char *result);
void processCommandC(const char *cmd);
static bool mdns_ready = false;

static inline bool distValid()
{
  return distFresh && (millis() - distFreshMs) <= SENSOR_VALID_MS;
}
static inline float obSafeMargin(int spd)
{
  float s = OB_BASE_CM + OB_K_PER_PWM * spd;
  if (s < 10)
    s = 10;
  if (s > 45)
    s = 45;
  return s;
}
static float clamp01(float x)
{
  if (x < 0)
    return 0;
  if (x > 1)
    return 1;
  return x;
}

struct WifiCred
{
  const char *ssid;
  const char *pass;
};
WifiCred WIFI_LIST[] = {
    {"nihongogajouzudesune", "nihongogajouzudesune"},
    {"backup_hotspot", "12345678"}};
const int WIFI_LIST_N = sizeof(WIFI_LIST) / sizeof(WifiCred);

bool startMDNS(const char *hostname)
{
  if (!mdns_ready)
    mdns_ready = MDNS.begin(hostname);
  return mdns_ready;
}

static void wifiConnectTick()
{
  switch (wifiFsm)
  {
  case WIFI_IDLE:
    if (WiFi.status() != WL_CONNECTED)
    {
      if (WiFi.getMode() != WIFI_MODE_STA)
      {
        WiFi.mode(WIFI_MODE_STA);
        WiFi.persistent(false);
      }
      wifiIdx = 0;
      wifiFsm = WIFI_TRY_BEGIN;
    }
    break;
  case WIFI_TRY_BEGIN:
    if (wifiIdx >= WIFI_LIST_N)
    {
      wifiFsm = WIFI_IDLE;
      break;
    }
    Serial.printf("[WiFi] begin(%s)\n", WIFI_LIST[wifiIdx].ssid);
    WiFi.begin(WIFI_LIST[wifiIdx].ssid, WIFI_LIST[wifiIdx].pass);
    wifiPhaseStart = millis();
    wifiFsm = WIFI_WAIT_CONN;
    break;
  case WIFI_WAIT_CONN:
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("[WiFi] connected");
      wifiFsm = WIFI_IDLE;
      break;
    }
    if (millis() - wifiPhaseStart >= WIFI_PER_SSID_MS)
    {
      Serial.println("[WiFi] timeout, next SSID");
      wifiIdx++;
      wifiFsm = WIFI_TRY_BEGIN;
    }
    break;
  }
}

static bool discoverTick()
{
  if (isIPv4(brokerHost))
    return true;

  static enum { DISC_IDLE,
                DISC_MDNS,
                DISC_UDP_TX,
                DISC_UDP_RX } discFsm = DISC_IDLE;
  static unsigned long discStart = 0;
  const unsigned long DISC_MDNS_BUDGET_MS = 200;
  const unsigned long DISC_UDP_RX_MS = 800;

  switch (discFsm)
  {
  case DISC_IDLE:
    discFsm = DISC_MDNS;
    discStart = millis();
    break;

  case DISC_MDNS:
  {
    String name = String(brokerHost);
    name.replace(".local", "");
    if (!name.length())
      name = "mqtt-broker";
    IPAddress ip = MDNS.queryHost(name.c_str());
    if (ip)
    {
      String s = ip.toString();
      strncpy(brokerHost, s.c_str(), sizeof(brokerHost) - 1);
      brokerHost[sizeof(brokerHost) - 1] = 0;
      brokerPort = MQTT_PORT;
      discFsm = DISC_IDLE;
      return true;
    }
    if (millis() - discStart >= DISC_MDNS_BUDGET_MS)
      discFsm = DISC_UDP_TX;
  }
  break;

  case DISC_UDP_TX:
  {
    const uint16_t PORT = 49300;
    const char *REQ = "ESP32_CAR_DISCOVER";
    if (!udpReady)
    {
      udp.stop();
      udpReady = udp.begin(PORT);
    }
    if (!udpReady)
    {
      discFsm = DISC_IDLE;
      break;
    }
    udp.beginPacket(IPAddress(255, 255, 255, 255), PORT);
    udp.write((const uint8_t *)REQ, strlen(REQ));
    udp.endPacket();
    discStart = millis();
    discFsm = DISC_UDP_RX;
  }
  break;

  case DISC_UDP_RX:
  {
    int sz = udp.parsePacket();
    if (sz > 0)
    {
      char buf[96];
      int n = udp.read(buf, sizeof(buf) - 1);
      if (n > 0)
      {
        buf[n] = 0;
        if (strncmp(buf, "MQTT ", 5) == 0)
        {
          const char *p = buf + 5;
          int a, b, c, d, port = MQTT_PORT;
          char extra;
          if (sscanf(p, "%d.%d.%d.%d:%d%c", &a, &b, &c, &d, &port, &extra) >= 5)
          {
            if (a >= 0 && a <= 255 && b >= 0 && b <= 255 && c >= 0 && c <= 255 && d >= 0 && d <= 255 && port > 0 && port < 65536)
            {
              snprintf(brokerHost, sizeof(brokerHost), "%d.%d.%d.%d", a, b, c, d);
              brokerPort = (uint16_t)port;
              discFsm = DISC_IDLE;
              return true;
            }
          }
          else if (isIPv4(p))
          {
            strncpy(brokerHost, p, sizeof(brokerHost) - 1);
            brokerHost[sizeof(brokerHost) - 1] = 0;
            brokerPort = MQTT_PORT;
            discFsm = DISC_IDLE;
            return true;
          }
        }
      }
    }
    if (millis() - discStart >= DISC_UDP_RX_MS)
      discFsm = DISC_IDLE;
  }
  break;
  }
  return false;
}

bool discoverBroker()
{
  unsigned long t0 = millis();
  while (millis() - t0 < 1000)
  {
    if (discoverTick())
      return true;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  return isIPv4(brokerHost);
}

void setupWiFi()
{
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.setSleep(true);
}

void setup()
{
  Serial.begin(115200);

  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_BLUE_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  ledcSetup(FRONT_LEFT_PWM_CH, 5000, 8);
  ledcAttachPin(FRONT_LEFT_PWM_PIN, FRONT_LEFT_PWM_CH);
  ledcWrite(FRONT_LEFT_PWM_CH, 0);
  ledcSetup(FRONT_RIGHT_PWM_CH, 5000, 8);
  ledcAttachPin(FRONT_RIGHT_PWM_PIN, FRONT_RIGHT_PWM_CH);
  ledcWrite(FRONT_RIGHT_PWM_CH, 0);
  ledcSetup(REAR_LEFT_PWM_CH, 5000, 8);
  ledcAttachPin(REAR_LEFT_PWM_PIN, REAR_LEFT_PWM_CH);
  ledcWrite(REAR_LEFT_PWM_CH, 0);
  ledcSetup(REAR_RIGHT_PWM_CH, 5000, 8);
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

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
  BT.begin("ESP32_Car");
  BT.setTimeout(10);

  setupWiFi();

  WiFi.onEvent([](arduino_event_id_t e, arduino_event_info_t info)
               {
    if (e == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      wifiConnected = true;
      Serial.printf("[WiFi] Connected: %s\n", WiFi.localIP().toString().c_str());
      publishAck("wifi", "connected");
    } else if (e == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      wifiConnected = false;
      Serial.println("[WiFi] Disconnected");
      publishAck("wifi", "disconnected");
    } });

#if USE_SINGLE_BATT
  analogSetPinAttenuation(PIN_BATT_PACK, ADC_11db);
#endif

  uint64_t mac = ESP.getEfuseMac();
  snprintf(mqttClientId, sizeof(mqttClientId), "ESP32_Car-%llX", (unsigned long long)mac);
  mqttClientId[sizeof(mqttClientId) - 1] = 0;

  mqttClient.setServer(brokerHost, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  MDNS.begin("esp32car");
  mdns_ready = true;

  xTaskCreate(commsTask, "commsTask", 4096, NULL, 2, NULL);
  xTaskCreate(sensorsTask, "sensorsTask", 4096, NULL, 1, NULL);
  xTaskCreate(controlTask, "controlTask", 4096, NULL, 2, NULL);
}

void commsTask(void *pvParameters)
{
  static enum { LED_BLINK, LED_SOLID_WAIT, LED_SOLID } ledState = LED_BLINK;
  static unsigned long ledStateSince = 0;
  const unsigned long COMMOK_HOLD_MS = 600;
  const unsigned long BLINK_MS = 300;

  for (;;)
  {
    unsigned long now = millis();

    btConnected = BT.hasClient();
    bool commOk = (btConnected || (wifiConnected && mqttClient.connected() && pcOnline));

    switch (ledState)
    {
      case LED_BLINK:
        if (now - lastBlueBlink >= BLINK_MS) {
          lastBlueBlink = now;
          blueOn = !blueOn;
          digitalWrite(LED_BLUE_PIN, blueOn ? HIGH : LOW);
        }
        if (commOk) {
          ledState = LED_SOLID_WAIT;
          ledStateSince = now;
        }
        break;

      case LED_SOLID_WAIT:
        if (now - lastBlueBlink >= BLINK_MS) {
          lastBlueBlink = now;
          blueOn = !blueOn;
          digitalWrite(LED_BLUE_PIN, blueOn ? HIGH : LOW);
        }
        if (!commOk) {
          ledState = LED_BLINK;
        } else if (now - ledStateSince >= COMMOK_HOLD_MS) {
          digitalWrite(LED_BLUE_PIN, HIGH);
          blueOn = true;
          ledState = LED_SOLID;
        }
        break;

      case LED_SOLID:
        if (!commOk) {
          ledState = LED_BLINK;
          lastBlueBlink = now;
          blueOn = false;
          digitalWrite(LED_BLUE_PIN, LOW);
        }
        break;
    }

    wifiConnectTick();

    static unsigned long lastDiscTick = 0;
    if (wifiConnected && (now - lastDiscTick >= 50))
    {
      lastDiscTick = now;
      if (!isIPv4(brokerHost) && discoverTick())
      {
        mqttClient.setServer(brokerHost, brokerPort);
        publishAck("broker", brokerHost);
      }
    }

    if (wifiConnected)
    {
      if (mqttClient.connected())
      {
        mqttClient.loop();
        reconnectAttempts = 0;
        lastMQTTAttempt = 0;
      }
      else
      {
        if (lastMQTTAttempt == 0 || now >= lastMQTTAttempt)
        {
          if (mqttClient.connect(mqttClientId))
          {
            mqttClient.subscribe(MQTT_TOPIC_CMD, 1);
            mqttClient.subscribe("car/ctl/pc/status", 1);
            mqttClient.subscribe("car/ctl/pc/heartbeat", 0);
            reconnectAttempts = 0;
            lastMQTTAttempt = 0;
          }
          else
          {
            reconnectAttempts++;
            int cap = (reconnectAttempts > 6) ? 6 : reconnectAttempts;
            int back = (1 << (cap - 1)) * 1000;
            if (back > (int)MQTT_RETRY_MAX_MS) back = MQTT_RETRY_MAX_MS;
            if (back < (int)MQTT_RETRY_MIN_MS) back = MQTT_RETRY_MIN_MS;
            int jitter = random(0, 500);
            lastMQTTAttempt = now + back + jitter;
          }
        }
      }
    }

    if (BT.available())
    {
      static char btBuf[64];
      static size_t btLen = 0;
      static unsigned long btLastCharMs = 0;

      while (BT.available())
      {
        int c = BT.read();
        if (c < 0) break;
        btLastCharMs = now;
        if (c == '\n' || c == '\r')
        {
          if (btLen > 0)
          {
            btBuf[btLen] = 0;
            processCommandC(btBuf);
            lastCmdMs = millis();
            publishAck("bt", btBuf);
            btLen = 0;
          }
        }
        else if (btLen < sizeof(btBuf) - 1)
        {
          btBuf[btLen++] = (char)c;
        }
      }
      if (btLen > 0 && (now - btLastCharMs) > 400)
      {
        btBuf[btLen] = 0;
        processCommandC(btBuf);
        lastCmdMs = millis();
        publishAck("bt", btBuf);
        btLen = 0;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(15));
  }
}


void sensorsTask(void *pvParameters)
{
  for (;;)
  {
    float dtmp;
    if (readUltrasonicCM_ok(&dtmp))
    {
      lastDistance = dtmp;
      distFresh = true;
      distFreshMs = millis();
    }
    else
    {
      distFresh = false;
      lastDistance = 999.0f;
    }

    if (emergencyInterrupt)
    {
      obstacleActive = false;
      obPhase = OB_IDLE;
      obConfirm = obRelease = 0;
    }

    bool isMoving = (strcmp(motionState, "Forward") == 0 || strcmp(motionState, "Backward") == 0 ||
                     strcmp(motionState, "Left") == 0 || strcmp(motionState, "Right") == 0);

    if (!obstacleActive)
    {
      if (!emergencyInterrupt && isMoving && distValid() && lastDistance <= OBSTACLE_CM)
      {
        obConfirm++;
        if (obConfirm >= OB_CONFIRM_COUNT)
        {
          obstacleActive = true;
          obConfirm = 0;
          obRelease = 0;
          strncpy(prevStateBeforeOb, motionState, sizeof(prevStateBeforeOb) - 1);
          prevStateBeforeOb[sizeof(prevStateBeforeOb) - 1] = 0;
          obPhase = OB_BACK;
          savedSpeedBeforeAvoid = requestedSpeed;
          requestedSpeed = OB_BACK_SPEED;
          strncpy(motionState, "Avoid", sizeof(motionState) - 1);
          motionState[sizeof(motionState) - 1] = 0;
          publishAck("obstacle", "on");
        }
      }
      else
      {
        obConfirm = 0;
      }
    }
    else
    {
      if (obPhase == OB_IDLE)
      {
        if (distValid() && lastDistance > (OBSTACLE_CM + OB_HYSTERESIS_CM))
        {
          obRelease++;
          if (obRelease >= OB_RELEASE_COUNT)
          {
            obstacleActive = false;
            obRelease = 0;
            publishAck("obstacle", "clear");
            requestedSpeed = savedSpeedBeforeAvoid;
#if OB_AUTO_RESUME
            strncpy(motionState, prevStateBeforeOb, sizeof(motionState) - 1);
#else
            strncpy(motionState, "Stopped", sizeof(motionState) - 1);
#endif
            motionState[sizeof(motionState) - 1] = 0;
          }
        }
        else
        {
          obRelease = 0;
        }
      }
    }

#if USE_SINGLE_BATT
    {
      const int samples = 16;
      uint32_t sumP = 0;
      for (int i = 0; i < samples; i++)
      {
        sumP += analogReadMilliVolts(PIN_BATT_PACK);
        delay(2);
      }
      float vAdcP = (sumP / (float)samples) / 1000.0f;
      packVoltage = vAdcP * ((R_TOP + R_BOT) / R_BOT);
      float pp = (packVoltage - BATT_PACK_V_MIN) / (BATT_PACK_V_MAX - BATT_PACK_V_MIN);
      packPercent = clamp01(pp) * 100.0f;
      motorVoltage = packVoltage;
      espVoltage = packVoltage;
      motorPercent = packPercent;
      espPercent = packPercent;
      if (packPercent < 20.0f && !motorLowNotified)
      {
        publishAck("battery_motor", "low");
        motorLowNotified = true;
      }
      if (packPercent >= 25.0f)
        motorLowNotified = false;
      if (packPercent < 20.0f && !espLowNotified)
      {
        publishAck("battery_esp", "low");
        espLowNotified = true;
      }
      if (packPercent >= 25.0f)
        espLowNotified = false;
    }
#endif

    showOLED();
    checkHeapAndRecover();
    vTaskDelay(pdMS_TO_TICKS(60));
  }
}

void controlTask(void *pvParameters)
{
  for (;;)
  {
    static bool lastEmergencyState = false;
    unsigned long now = millis();

    if (pcOnline && (now - lastPcBeatMs) > PC_TIMEOUT_MS)
    {
      pcOnline = false;
      publishAck("pc", "offline");
    }

    if ((now - lastCmdMs) > CMD_TIMEOUT_MS)
    {
      if (strcmp(motionState, "Stopped") != 0 && !emergencyInterrupt && !obstacleActive)
      {
        strncpy(motionState, "Stopped", sizeof(motionState) - 1);
        motionState[sizeof(motionState) - 1] = 0;
        stopMotors();
        publishAck("auto", "idle");
      }
    }

    if (emergencyInterrupt)
    {
      digitalWrite(LED_RED_PIN, HIGH);
      stopMotors();
      obstacleActive = false;
      obPhase = OB_IDLE;
    }
    else
    {
      digitalWrite(LED_RED_PIN, LOW);
    }

    if (emergencyInterrupt != lastEmergencyState)
    {
      obstacleActive = false;
      obPhase = OB_IDLE;
      requestedSpeed = savedSpeedBeforeAvoid;
      publishAck("interrupt", emergencyInterrupt ? "on" : "off");
      lastEmergencyState = emergencyInterrupt;
    }

    if (!emergencyInterrupt && distValid() && lastDistance <= OB_HARD_STOP_CM)
    {
      stopMotors();
      setAxles(OB_BACK_SPEED, 2, OB_BACK_SPEED, 2);
      vTaskDelay(pdMS_TO_TICKS(OB_BACK_BURST_MS));
      stopMotors();
      strncpy(motionState, "Stopped", sizeof(motionState) - 1);
      motionState[sizeof(motionState) - 1] = 0;
      obstacleActive = false;
      obPhase = OB_IDLE;
      requestedSpeed = savedSpeedBeforeAvoid;
      vTaskDelay(pdMS_TO_TICKS(15));
      continue;
    }

    if (obstacleActive)
    {
      if (emergencyInterrupt)
      {
        stopMotors();
        obstacleActive = false;
        obPhase = OB_IDLE;
        requestedSpeed = savedSpeedBeforeAvoid;
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }

      if (obPhase == OB_BACK)
      {
        setAxles(OB_BACK_SPEED, 2, OB_BACK_SPEED, 2);
        const unsigned long SENSOR_TIMEOUT_MS = 800;
        if (!distValid() && (now - distFreshMs > SENSOR_TIMEOUT_MS))
        {
          stopMotors();
          obstacleActive = false;
          obPhase = OB_IDLE;
          requestedSpeed = savedSpeedBeforeAvoid;
          publishAck("obstacle", "sensor_lost");
          vTaskDelay(pdMS_TO_TICKS(30));
          continue;
        }
        static int safeCount = 0;
        float need = (OBSTACLE_CM + obSafeMargin(requestedSpeed));
        if (distValid() && lastDistance > need)
        {
          if (safeCount < 3)
            safeCount++;
        }
        else
          safeCount = 0;

        if (safeCount >= 3)
        {
#if OB_ENABLE_TURN_AFTER_BACK
          obPhase = OB_TURN;
          obPhaseUntil = now + OB_TURN_TIME_MS;
          safeCount = 0;
#else
          obstacleActive = false;
          obPhase = OB_IDLE;
          stopMotors();
          requestedSpeed = savedSpeedBeforeAvoid;
          publishAck("obstacle", "cleared");
#if OB_AUTO_RESUME
          strncpy(motionState, prevStateBeforeOb, sizeof(motionState) - 1);
#else
          strncpy(motionState, "Stopped", sizeof(motionState) - 1);
#endif
          motionState[sizeof(motionState) - 1] = 0;
#endif
        }
        vTaskDelay(pdMS_TO_TICKS(30));
        continue;
      }

      if (obPhase == OB_TURN)
      {
        if (obTurnDir == 1)
        {
          setMotorSide(OB_TURN_SPEED, 1, FRONT_LEFT_PWM_CH, FRONT_LEFT_DIR_PIN1, FRONT_LEFT_DIR_PIN2);
          setMotorSide(OB_TURN_SPEED, 1, REAR_LEFT_PWM_CH, REAR_LEFT_DIR_PIN1, REAR_LEFT_DIR_PIN2);
          setMotorSide(OB_TURN_SPEED, 2, FRONT_RIGHT_PWM_CH, FRONT_RIGHT_DIR_PIN1, FRONT_RIGHT_DIR_PIN2);
          setMotorSide(OB_TURN_SPEED, 2, REAR_RIGHT_PWM_CH, REAR_RIGHT_DIR_PIN1, REAR_RIGHT_DIR_PIN2);
        }
        else
        {
          setMotorSide(OB_TURN_SPEED, 2, FRONT_LEFT_PWM_CH, FRONT_LEFT_DIR_PIN1, FRONT_LEFT_DIR_PIN2);
          setMotorSide(OB_TURN_SPEED, 2, REAR_LEFT_PWM_CH, REAR_LEFT_DIR_PIN1, REAR_LEFT_DIR_PIN2);
          setMotorSide(OB_TURN_SPEED, 1, FRONT_RIGHT_PWM_CH, FRONT_RIGHT_DIR_PIN1, FRONT_RIGHT_DIR_PIN2);
          setMotorSide(OB_TURN_SPEED, 1, REAR_RIGHT_PWM_CH, REAR_RIGHT_DIR_PIN1, REAR_RIGHT_DIR_PIN2);
        }
        if (now >= obPhaseUntil)
        {
          obstacleActive = false;
          obPhase = OB_IDLE;
          stopMotors();
          requestedSpeed = savedSpeedBeforeAvoid;
          publishAck("obstacle", "avoid_done");
#if OB_AUTO_RESUME
          strncpy(motionState, prevStateBeforeOb, sizeof(motionState) - 1);
#else
          strncpy(motionState, "Stopped", sizeof(motionState) - 1);
#endif
          motionState[sizeof(motionState) - 1] = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }

      stopMotors();
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    if (!emergencyInterrupt)
    {
      int dir = 0;
      if (strcmp(motionState, "Forward") == 0)
        dir = 1;
      else if (strcmp(motionState, "Backward") == 0)
        dir = 2;

      if (strcmp(motionState, "Left") == 0)
      {
        setMotorSide(requestedSpeed, 2, FRONT_LEFT_PWM_CH, FRONT_LEFT_DIR_PIN1, FRONT_LEFT_DIR_PIN2);
        setMotorSide(requestedSpeed, 2, REAR_LEFT_PWM_CH, REAR_LEFT_DIR_PIN1, REAR_LEFT_DIR_PIN2);
        setMotorSide(requestedSpeed, 1, FRONT_RIGHT_PWM_CH, FRONT_RIGHT_DIR_PIN1, FRONT_RIGHT_DIR_PIN2);
        setMotorSide(requestedSpeed, 1, REAR_RIGHT_PWM_CH, REAR_RIGHT_DIR_PIN1, REAR_RIGHT_DIR_PIN2);
      }
      else if (strcmp(motionState, "Right") == 0)
      {
        setMotorSide(requestedSpeed, 1, FRONT_LEFT_PWM_CH, FRONT_LEFT_DIR_PIN1, FRONT_LEFT_DIR_PIN2);
        setMotorSide(requestedSpeed, 1, REAR_LEFT_PWM_CH, REAR_LEFT_DIR_PIN1, REAR_LEFT_DIR_PIN2);
        setMotorSide(requestedSpeed, 2, FRONT_RIGHT_PWM_CH, FRONT_RIGHT_DIR_PIN1, FRONT_RIGHT_DIR_PIN2);
        setMotorSide(requestedSpeed, 2, REAR_RIGHT_PWM_CH, REAR_RIGHT_DIR_PIN1, REAR_RIGHT_DIR_PIN2);
      }
      else
      {
        setAxles(requestedSpeed, dir, requestedSpeed, dir);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(15));
  }
}

void processCommandC(const char *cmd)
{
  if (!cmd)
    return;
  lastCmdMs = millis();

  if (strncasecmp(cmd, "broker:", 7) == 0)
  {
    strncpy(brokerHost, cmd + 7, sizeof(brokerHost) - 1);
    brokerHost[sizeof(brokerHost) - 1] = 0;
    mqttClient.disconnect();
    mqttClient.setServer(brokerHost, MQTT_PORT);
    publishAck("broker", brokerHost);
    return;
  }
  if (strcasecmp(cmd, "discover") == 0)
  {
    if (discoverBroker())
    {
      mqttClient.disconnect();
      mqttClient.setServer(brokerHost, MQTT_PORT);
      publishAck("broker", brokerHost);
    }
    else
      publishAck("broker", "discover_failed");
    return;
  }
  if (strcasecmp(cmd, "wifi") == 0)
  {
    WiFi.disconnect(true, true);
    lastWiFiTryMs = 0;
    publishAck("wifi", "searching");
    return;
  }
  if (strcasecmp(cmd, "whoami") == 0)
  {
    char info[128];
    snprintf(info, sizeof(info), "wifi=%s ip=%s bt=%s broker=%s",
             (WiFi.status() == WL_CONNECTED) ? "on" : "off",
             (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString().c_str() : "0.0.0.0",
             BT.hasClient() ? "on" : "off",
             brokerHost);
    publishAck("whoami", info);
    return;
  }
  if (strcasecmp(cmd, "1") == 0 || strcasecmp(cmd, "spd1") == 0)
  {
    requestedSpeed = 200;
    publishAck("speed", "200");
    return;
  }
  if (strcasecmp(cmd, "2") == 0 || strcasecmp(cmd, "spd2") == 0)
  {
    requestedSpeed = 230;
    publishAck("speed", "230");
    return;
  }
  if (strcasecmp(cmd, "3") == 0 || strcasecmp(cmd, "spd3") == 0)
  {
    requestedSpeed = 255;
    publishAck("speed", "255");
    return;
  }
  if (strcasecmp(cmd, "w") == 0)
  {
    strncpy(motionState, "Forward", sizeof(motionState) - 1);
  }
  else if (strcasecmp(cmd, "s") == 0)
  {
    strncpy(motionState, "Backward", sizeof(motionState) - 1);
  }
  else if (strcasecmp(cmd, "a") == 0)
  {
    strncpy(motionState, "Left", sizeof(motionState) - 1);
  }
  else if (strcasecmp(cmd, "d") == 0)
  {
    strncpy(motionState, "Right", sizeof(motionState) - 1);
  }
  else if (strcasecmp(cmd, "Idle") == 0 || strcasecmp(cmd, "Stop") == 0)
  {
    strncpy(motionState, "Stopped", sizeof(motionState) - 1);
  }
  else if (strcasecmp(cmd, "i") == 0)
  {
    strncpy(motionState, "Interrupt", sizeof(motionState) - 1);
    emergencyInterrupt = !emergencyInterrupt;
  }
  else
  {
    char *endp = nullptr;
    long v = strtol(cmd, &endp, 10);
    if (endp && *endp == '\0')
    {
      int absV = v >= 0 ? v : -v;
      requestedSpeed = constrain(absV, 0, 255);
      char spdbuf[8];
      snprintf(spdbuf, sizeof(spdbuf), "%d", requestedSpeed);
      publishAck("speed", spdbuf);
    }
    else
    {
      publishAck("cmd_unknown", cmd);
    }
  }
  motionState[sizeof(motionState) - 1] = 0;
}

void mqttCallback(char *topic, byte *message, unsigned int length)
{
  char buf[64];
  size_t n = (length < sizeof(buf) - 1) ? length : (sizeof(buf) - 1);
  memcpy(buf, message, n);
  buf[n] = 0;

  if (strcmp(topic, MQTT_TOPIC_CMD) == 0)
  {
    lastCmdMs = millis();
    processCommandC(buf);
    publishAck("ack", motionState);
    return;
  }

  if (strcmp(topic, "car/ctl/pc/status") == 0)
  {
    if (strcasecmp(buf, "online") == 0)
    {
      pcOnline = true;
      publishAck("pc", "online");
    }
    else if (strcasecmp(buf, "offline") == 0)
    {
      pcOnline = false;
      publishAck("pc", "offline");
    }
    return;
  }

  if (strcmp(topic, "car/ctl/pc/heartbeat") == 0)
  {
    lastPcBeatMs = millis();
    if (!pcOnline)
    {
      pcOnline = true;
      publishAck("pc", "online");
    }
    return;
  }

  publishAck("rx", buf);
}

void publishAck(const char *cmd, const char *result)
{
  if (mqttClient.connected())
  {
    char buff[128];
    snprintf(buff, sizeof(buff), "%s %s", cmd, result);
    mqttClient.publish(MQTT_TOPIC_REPLY, buff);
  }
}

void stopMotors()
{
  setMotorAxle(0, 0, FRONT_LEFT_PWM_CH, FRONT_LEFT_DIR_PIN1, FRONT_LEFT_DIR_PIN2,
               FRONT_RIGHT_PWM_CH, FRONT_RIGHT_DIR_PIN1, FRONT_RIGHT_DIR_PIN2);
  setMotorAxle(0, 0, REAR_LEFT_PWM_CH, REAR_LEFT_DIR_PIN1, REAR_LEFT_DIR_PIN2,
               REAR_RIGHT_PWM_CH, REAR_RIGHT_DIR_PIN1, REAR_RIGHT_DIR_PIN2);
}

void setAxles(int speedFront, int dirFront, int speedRear, int dirRear)
{
  setMotorAxle(speedFront, dirFront, FRONT_LEFT_PWM_CH, FRONT_LEFT_DIR_PIN1, FRONT_LEFT_DIR_PIN2,
               FRONT_RIGHT_PWM_CH, FRONT_RIGHT_DIR_PIN1, FRONT_RIGHT_DIR_PIN2);
  setMotorAxle(speedRear, dirRear, REAR_LEFT_PWM_CH, REAR_LEFT_DIR_PIN1, REAR_LEFT_DIR_PIN2,
               REAR_RIGHT_PWM_CH, REAR_RIGHT_DIR_PIN1, REAR_RIGHT_DIR_PIN2);
}

void setMotorAxle(int pwmVal, int dir, int leftChannel, int leftDir1, int leftDir2, int rightChannel, int rightDir1, int rightDir2)
{
  pwmVal = constrain(pwmVal, 0, 255);
  if (dir == 0)
  {
    digitalWrite(leftDir1, LOW);
    digitalWrite(leftDir2, LOW);
    digitalWrite(rightDir1, LOW);
    digitalWrite(rightDir2, LOW);
    ledcWrite(leftChannel, 0);
    ledcWrite(rightChannel, 0);
  }
  else if (dir == 1)
  {
    digitalWrite(leftDir1, HIGH);
    digitalWrite(leftDir2, LOW);
    digitalWrite(rightDir1, HIGH);
    digitalWrite(rightDir2, LOW);
    ledcWrite(leftChannel, pwmVal);
    ledcWrite(rightChannel, pwmVal);
  }
  else if (dir == 2)
  {
    digitalWrite(leftDir1, LOW);
    digitalWrite(leftDir2, HIGH);
    digitalWrite(rightDir1, LOW);
    digitalWrite(rightDir2, HIGH);
    ledcWrite(leftChannel, pwmVal);
    ledcWrite(rightChannel, pwmVal);
  }
}

void setMotorSide(int pwmVal, int dir, int pwmChannel, int dirPin1, int dirPin2)
{
  pwmVal = constrain(pwmVal, 0, 255);
  if (dir == 0)
  {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
    ledcWrite(pwmChannel, 0);
  }
  else if (dir == 1)
  {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    ledcWrite(pwmChannel, pwmVal);
  }
  else if (dir == 2)
  {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    ledcWrite(pwmChannel, pwmVal);
  }
}

bool readUltrasonicCM_ok(float *out_cm)
{
  const int N = 5;
  float v[N];
  for (int i = 0; i < N; i++)
  {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    unsigned long dur = pulseIn(ECHO_PIN, HIGH, 20000);
    v[i] = (dur == 0) ? NAN : (dur / 2.0f) / 29.1f;
    delay(5);
  }
  int m = 0;
  for (int i = 0; i < N; i++)
    if (!isnan(v[i]))
      v[m++] = v[i];
  if (m == 0)
    return false;
  for (int i = 0; i < m - 1; i++)
    for (int j = i + 1; j < m; j++)
      if (v[j] < v[i])
      {
        float t = v[i];
        v[i] = v[j];
        v[j] = t;
      }
  float med = v[m / 2];
  if (med < OB_MIN_CM || med > OB_MAX_CM)
    return false;
  *out_cm = med;
  return true;
}

void IRAM_ATTR handleButtonInterrupt()
{
  unsigned long now = millis();
  if (now - lastInterruptMillis < 300)
    return;
  lastInterruptMillis = now;
  emergencyInterrupt = !emergencyInterrupt;
}

void checkHeapAndRecover()
{
  size_t freeHeap = ESP.getFreeHeap();
  static unsigned long lastLogTime = 0;
  static size_t lowestSeen = SIZE_MAX;
  if (freeHeap < lowestSeen)
    lowestSeen = freeHeap;
  unsigned long now = millis();
  if (now - lastLogTime >= 10000)
  {
    Serial.printf("Free heap: %u bytes, lowest seen: %u\n", (unsigned)freeHeap, (unsigned)lowestSeen);
    lastLogTime = now;
  }
  if (freeHeap < 18000)
  {
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP.restart();
  }
}

void showOLED()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  char distStr[12];
  if (distValid())
    snprintf(distStr, sizeof(distStr), "%.1fcm", lastDistance);
  else
    snprintf(distStr, sizeof(distStr), "--");

  const char *linkStr = btConnected ? "BT"
                                    : (wifiConnected ? "WiFi" : "None");
  const char *pcStr = pcOnline ? "OK" : "--";

  display.printf("State:%s Spd:%d\n", motionState, requestedSpeed);
  display.printf("Dist:%s Emg:%s\n", distStr, emergencyInterrupt ? "ON" : "OFF");
#if USE_SINGLE_BATT
  display.printf("Batt :%5.2fV(%3.0f%%)\n", packVoltage, packPercent);
#endif
  display.printf("Link:%s PC:%s\n", linkStr, pcStr); // <— เพิ่มบรรทัดนี้
  display.printf("WiFi:%s BT:%s\n", wifiConnected ? "OK" : "...",
                 btConnected ? "OK" : "...");
  display.display();
}

void mqttCallback(char *topic, byte *message, unsigned int length);

void loop() {}
