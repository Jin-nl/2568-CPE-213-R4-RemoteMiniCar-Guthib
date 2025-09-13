#include <WiFi.h>
#include <PubSubClient.h>
#include <BluetoothSerial.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <cstring> // for strlen, strcmp, etc.

// ----- CONFIG -----
#define WIFI_STA_SSID     "btm02_2.4G"   
#define WIFI_STA_PASS     "136167118"    
#define WIFI_AP_SSID      "ESP32_CarAP" 
#define WIFI_AP_PASS      "guthib123"    

#define MQTT_SERVER_IP    "192.168.2.34" 
#define MQTT_PORT         1883           
#define MQTT_TOPIC_CMD    "car/cmd"      // Topic สำหรับรับคำสั่ง
#define MQTT_TOPIC_REPLY  "car/response" // Topic สำหรับส่ง ACK / status

//MQTT clientId: ESP32_Car-30AB7F2A13D8

#define DHTPIN          15       // ต่อกับ DHT11
#define DHTTYPE         DHT11    // ประเภทของเซ็นเซอร์ (DHT11, DHT22)

#define TRIG_PIN        27       // ส่งสัญญาณ Trigger
#define ECHO_PIN        34       // รับสัญญาณ Echo
#define OBSTACLE_CM     10       // ระยะขั้นต่ำที่ถือว่ามีสิ่งกีดขวาง (cm)

// Motor Pins
#define FRONT_PWM_PIN     18     //PWM pin สำหรับความเร็วมอเตอร์หน้า
#define FRONT_DIR_PIN1    32     //มอเตอร์หน้าซ้าย
#define FRONT_DIR_PIN2    33     //มอเตอร์หน้าขวา
#define FRONT_PWM_CH      0      //Channel PWM สำหรับล้อหน้า

#define SERVO_PIN         13     // พิน GPIO สำหรับสัญญาณ PWM ไปยัง Servo

#define REAR_PWM_PIN      19     // PWM pin สำหรับความเร็วมอเตอร์หลัง
#define REAR_DIR_PIN1     25     // มอเตอร์หลังซ้าย
#define REAR_DIR_PIN2     26     // มอเตอร์หลังขวา
#define REAR_PWM_CH       1      // Channel PWM สำหรับล้อหลัง

#define LED_BLUE_PIN      2      // LED สีน้ำเงิน ใช้แสดงสถานะ WiFi
#define LED_RED_PIN       4      // LED สีแดง ใช้แจ้งเหตุฉุกเฉิน
#define BUTTON_INT_PIN    14     // ปุ่ม Interrupt ใช้ INPUT_PULLUP

#define BATT_PIN          35     // พิน ADC สำหรับวัดแรงดันแบตเตอรี่

#define OLED_SDA          21     // SDA สำหรับ I2C ของ OLED
#define OLED_SCL          22     // SCL สำหรับ I2C ของ OLED
#define SCREEN_WIDTH      128    // ความกว้างหน้าจอ OLED (pixel)
#define SCREEN_HEIGHT     64     // ความสูงหน้าจอ OLED (pixel)
#define OLED_RESET        -1     // Reset pin ของ OLED (-1 = ไม่ใช้)


// ----- GLOBALS -----
DHT dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BluetoothSerial BT;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Servo steering;

volatile bool emergencyInterrupt = false;
volatile unsigned long lastInterruptMillis = 0;

static char latestCmdBuf[16];    
static bool hasCmd = false;      
SemaphoreHandle_t cmdMutex;

char motionState[24] = "Stopped";
int requestedSpeed = 150;
float lastTemp=0, lastHum=0, lastDistance=999;

char mqttClientId[32]; 

float batteryVoltage = 0.0;     // ใช้เก็บค่าแรงดันแบตเตอรี่ (Voltage)
float batteryPercent = 100.0;   // ใช้เก็บค่าระดับแบตเตอรี่ Percent


// ----- PROTOTYPES -----
void setupWiFi();
void mqttCallback(char* topic, byte* message, unsigned int length);
void commsTask(void *pvParameters);
void sensorsTask(void *pvParameters);
void controlTask(void *pvParameters);
void stopMotors();
void setMotorAxle(int pwmVal,int dir,int channel,int dir1,int dir2);
void setAxles(int speedFront,int dirFront,int speedRear,int dirRear);
float readUltrasonicCM();
void IRAM_ATTR handleButtonInterrupt();
void showOLED();
void processCommandC(const char* cmd);
void publishAck(const char* cmd, const char* result);
void checkHeapAndRecover();
void readBattery();

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_BLUE_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  ledcSetup(FRONT_PWM_CH, 20000, 8); ledcAttachPin(FRONT_PWM_PIN, FRONT_PWM_CH); ledcWrite(FRONT_PWM_CH,0);
  ledcSetup(REAR_PWM_CH,  20000, 8); ledcAttachPin(REAR_PWM_PIN, REAR_PWM_CH);   ledcWrite(REAR_PWM_CH,0);

  pinMode(FRONT_DIR_PIN1, OUTPUT); pinMode(FRONT_DIR_PIN2, OUTPUT);
  pinMode(REAR_DIR_PIN1, OUTPUT); pinMode(REAR_DIR_PIN2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(BUTTON_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_INT_PIN), handleButtonInterrupt, FALLING);

  dht.begin();
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  steering.attach(SERVO_PIN);
  steering.write(90);

  if(!BT.begin("ESP32_Car")) Serial.println("BT failed");

  uint64_t mac = ESP.getEfuseMac();
  snprintf(mqttClientId, sizeof(mqttClientId), "ESP32_Car-%llX", (unsigned long long)mac);
  mqttClientId[sizeof(mqttClientId)-1] = 0;
  Serial.printf("MQTT clientId: %s\n", mqttClientId);

  setupWiFi();
  mqttClient.setServer(MQTT_SERVER_IP, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  cmdMutex = xSemaphoreCreateMutex();

  xTaskCreate(commsTask, "commsTask", 4096, NULL, 2, NULL);
  xTaskCreate(sensorsTask, "sensorsTask", 4096, NULL, 1, NULL);
  xTaskCreate(controlTask, "controlTask", 4096, NULL, 2, NULL);
}

// -------------------- TASKS --------------------
void commsTask(void *pvParameters) {
  static unsigned long nextMqttAttempt = 0;
  static int reconnectAttempts = 0;

  for(;;){
    if(WiFi.status() != WL_CONNECTED){
      Serial.println("WiFi disconnected! Reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    }

    bool wifiConnected = (WiFi.status()==WL_CONNECTED);
    digitalWrite(LED_BLUE_PIN, wifiConnected ? HIGH : LOW);

    if(wifiConnected){
      unsigned long now = millis();
      if(mqttClient.connected()){
        mqttClient.loop();
        reconnectAttempts = 0;
        nextMqttAttempt = 0;
      } else {
        if(nextMqttAttempt == 0 || now >= nextMqttAttempt){
          Serial.println("MQTT attempting connect...");
          if(mqttClient.connect(mqttClientId)){
            mqttClient.subscribe(MQTT_TOPIC_CMD, 1);
            Serial.println("MQTT connected!");
            reconnectAttempts = 0;
            nextMqttAttempt = 0;
          } else {
            reconnectAttempts++;
            int capAttempts = (reconnectAttempts > 6) ? 6 : reconnectAttempts;
            int backoffSec = (1 << (capAttempts - 1));
            if(backoffSec > 30) backoffSec = 30;
            int jitter = random(0, 500);
            nextMqttAttempt = now + (unsigned long)backoffSec * 1000UL + jitter;
            Serial.printf("MQTT connect failed (state=%d). retry in %d s (+%d ms jitter)\n", mqttClient.state(), backoffSec, jitter);
          }
        }
      }
    }

    if(BT.available()){
      char buf[16];
      memset(buf,0,sizeof(buf));
      size_t len = BT.readBytesUntil('\n', buf, sizeof(buf)-1);
      if(len>0){
        while(len>0 && (buf[len-1]=='\r' || buf[len-1]=='\n' || buf[len-1]==' ')) { buf[len-1]=0; len--; }
        size_t start=0;
        while(buf[start]==' ' && start < len) start++;
        if(start>0){
          memmove(buf, buf+start, len-start+1);
          len -= start;
        }
        if(len>0){
          if(xSemaphoreTake(cmdMutex, pdMS_TO_TICKS(10))==pdTRUE){
            strncpy(latestCmdBuf, buf, sizeof(latestCmdBuf)-1);
            latestCmdBuf[sizeof(latestCmdBuf)-1]=0;
            hasCmd = true;
            xSemaphoreGive(cmdMutex);
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void sensorsTask(void *pvParameters){
  unsigned long lastDHTReadTime = 0;
  const unsigned long DHT_READ_INTERVAL = 2000; // 2 วินาที

  for(;;){
    unsigned long now = millis();
    // อ่าน DHT ทุก 2 วินาที
    if (now - lastDHTReadTime >= DHT_READ_INTERVAL) {
      float t = dht.readTemperature(); 
      if (!isnan(t)) lastTemp = t;

      float h = dht.readHumidity(); 
      if (!isnan(h)) lastHum = h;

      lastDHTReadTime = now;
    }

    lastDistance = readUltrasonicCM();
    if (lastDistance <= OBSTACLE_CM && lastDistance > 0) {
      stopMotors();
      strncpy(motionState, "Obstacle", sizeof(motionState)-1);
      motionState[sizeof(motionState)-1] = 0;
      publishAck("obstacle", motionState);
      Serial.println("Obstacle detected! Motors stopped.");
    }

    readBattery();
    if (batteryPercent < 20.0) {
        publishAck(MQTT_TOPIC_REPLY, "Battery Low!");
      //digitalWrite(LED_RED_PIN, HIGH);
    }

    showOLED();
    checkHeapAndRecover();

    vTaskDelay(pdMS_TO_TICKS(500)); // loop ถี่ขึ้นเล็กน้อย
  }
}


void controlTask(void *pvParameters){
  for(;;){
    static bool lastEmergencyState = false;
    if(emergencyInterrupt){
        digitalWrite(LED_RED_PIN,HIGH);
        digitalWrite(LED_BLUE_PIN,LOW);
        stopMotors();
    } else {
        digitalWrite(LED_RED_PIN,LOW);
    }

    if (emergencyInterrupt != lastEmergencyState) {
        if (emergencyInterrupt){
            Serial.println("[ALERT] Emergency Interrupt Activated!");
            publishAck(MQTT_TOPIC_REPLY, "Emergency Interrupt Activated");
        } else {
            Serial.println("[INFO] Emergency Interrupt Cleared");
            publishAck(MQTT_TOPIC_REPLY, "Emergency Interrupt Cleared");
        }
        lastEmergencyState = emergencyInterrupt;
    }

    if (xSemaphoreTake(cmdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if(hasCmd){
        char cmdLocal[16];
        strncpy(cmdLocal, latestCmdBuf, sizeof(cmdLocal)-1);
        cmdLocal[sizeof(cmdLocal)-1]=0;
        hasCmd = false;
        latestCmdBuf[0]=0;
        xSemaphoreGive(cmdMutex);

        processCommandC(cmdLocal);
        Serial.printf("cmd=%s -> motionState=%s\n", cmdLocal, motionState);
        publishAck(cmdLocal, motionState);
          
      } else {
        xSemaphoreGive(cmdMutex);
      }
    }

    if(!emergencyInterrupt){
      int dir = 0;
      if(strcmp(motionState, "Forward")==0) dir=1;
      else if(strcmp(motionState, "Backward")==0) dir=2;
      else dir = 0;

      if(strcmp(motionState,"Left")==0) steering.write(60);
      else if(strcmp(motionState,"Right")==0) steering.write(120);
      else steering.write(90);

      setAxles(requestedSpeed, dir, requestedSpeed, dir);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// -------------------- HELPERS --------------------
void processCommandC(const char* cmd){
  if(cmd == NULL) return;
  if(strcasecmp(cmd, "w")==0){ strncpy(motionState, "Forward", sizeof(motionState)-1); }
  else if(strcasecmp(cmd, "s")==0){ strncpy(motionState, "Backward", sizeof(motionState)-1); }
  else if(strcasecmp(cmd, "a")==0){ strncpy(motionState, "Left", sizeof(motionState)-1); }
  else if(strcasecmp(cmd, "d")==0){ strncpy(motionState, "Right", sizeof(motionState)-1); }
  else if(strcasecmp(cmd, "x")==0 || strcasecmp(cmd,"Stop")==0){ strncpy(motionState, "Stopped", sizeof(motionState)-1); }
  else if(strcasecmp(cmd, "i")==0){ strncpy(motionState, "Interrupt", sizeof(motionState)-1);emergencyInterrupt = !emergencyInterrupt; }
  else { int v = atoi(cmd); if(v > 0){ strncpy(motionState, "Set speed", sizeof(motionState)-1);requestedSpeed = constrain(v, 0, 255); }}
  motionState[sizeof(motionState)-1] = 0;
}

void publishAck(const char* cmd, const char* result){
  if(mqttClient.connected()){
    char buff[64];
    snprintf(buff, sizeof(buff), "%s %s success", cmd, result);//ฟังก์ชันที่ใช้เขียนข้อความ (format string) ลงในบัฟเฟอร์อย่างปลอดภัย
    mqttClient.publish(MQTT_TOPIC_REPLY, buff);
  }
}

void stopMotors(){ 
  // ล้อหน้าไม่ขับ
  //setMotorAxle(0,0,FRONT_PWM_CH,FRONT_DIR_PIN1,FRONT_DIR_PIN2);
  setMotorAxle(0,0,REAR_PWM_CH,REAR_DIR_PIN1,REAR_DIR_PIN2);  //ล้อหลังหยุด
}

void setAxles(int speedFront,int dirFront,int speedRear,int dirRear){
  // ล้อหน้าไม่ขับ
  //setMotorAxle(speedFront, dirFront, FRONT_PWM_CH, FRONT_DIR_PIN1, FRONT_DIR_PIN2); //FL + FR ขับ พร้อมกัน (วิ่งหน้า–หลังด้วยทิศเดียวกันและความเร็วเดียวกัน)
  setMotorAxle(speedRear, dirRear, REAR_PWM_CH, REAR_DIR_PIN1, REAR_DIR_PIN2); //RL + RR ขับ พร้อมกัน (วิ่งหน้า–หลังด้วยทิศเดียวกันและความเร็วเดียวกัน)
}

void setMotorAxle(int pwmVal,int dir,int channel,int dir1,int dir2){
  pwmVal=constrain(pwmVal,0,255);
  if(dir==0){ digitalWrite(dir1,LOW); digitalWrite(dir2,LOW); ledcWrite(channel,0); }  // stop → DIR1=LOW, DIR2=LOW
  else if(dir==1){ digitalWrite(dir1,HIGH); digitalWrite(dir2,LOW); ledcWrite(channel,pwmVal); }  // forward → DIR1=HIGH, DIR2=LOW
  else if(dir==2){ digitalWrite(dir1,LOW); digitalWrite(dir2,HIGH); ledcWrite(channel,pwmVal); }  // backward → DIR1=LOW, DIR2=HIGH
  // DIR1 + DIR2 ใช้กำหนดทิศทาง:
}

float readUltrasonicCM(){
  digitalWrite(TRIG_PIN,LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN,HIGH); delayMicroseconds(10); digitalWrite(TRIG_PIN,LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 20000);
  if(dur == 0) return 999;
  return (dur / 2.0) / 29.1;
}

void IRAM_ATTR handleButtonInterrupt() {
  unsigned long now = millis();
  if (now - lastInterruptMillis < 300) return;
  lastInterruptMillis = now;
  emergencyInterrupt = !emergencyInterrupt;
}

void checkHeapAndRecover(){
  size_t freeHeap = ESP.getFreeHeap();  // ตรวจสอบ heap memory ที่ยังว่าง byte
  static unsigned long lastLogTime = 0; // จำเวลาที่ log ล่าสุด
  static size_t lowestSeen = SIZE_MAX;  // เก็บค่าต่ำสุดที่เคยเห็นไว้ (ตอนเริ่มจะเป็นค่าสูงสุด)
  if(freeHeap < lowestSeen) lowestSeen = freeHeap;  // อัปเดต lowest ถ้าต่ำกว่าครั้งก่อน

  // ปริ้น log ทุก 10 วินาที
  unsigned long now = millis();
  if (now - lastLogTime >= 10000) {
    Serial.printf("Free heap: %u bytes, lowest seen: %u\n",
                  (unsigned)freeHeap, (unsigned)lowestSeen); // แสดงผล heap ปัจจุบันและต่ำสุดที่เคยมี
    lastLogTime = now;
  }

  if(freeHeap < 18000){  // ถ้ามีน้อยกว่า 18KB
    Serial.println("Free heap too low -> restarting to recover");
    vTaskDelay(pdMS_TO_TICKS(200));  // รอให้ Serial ส่งข้อความทัน
    ESP.restart();       // รีสตาร์ท ESP32 เพื่อเคลียร์ heap
  }
}

void readBattery(){
  int adcVal = analogRead(BATT_PIN);                                //อ่านค่าจากขาอนาล็อก BATT_PIN ซึ่งจะให้ค่า 0–4095
  batteryVoltage = adcVal * (3.3 / 4095.0);                         //แปลงค่า ADC เป็นแรงดันไฟฟ้า (Voltage)          
  batteryVoltage = batteryVoltage * ((10000.0 + 4700.0) / 4700.0);  //ถูกแบ่งด้วย voltage divider (R1 + R2)/R2
  //batteryPercent = (batteryVoltage - 6.0) / (9.0 - 6.0) * 100.0;  //กำหนด แรงดันต่ำสุด (6V) → เต็ม (9V)
  batteryPercent = (batteryVoltage - 4.0) / (6.0 - 4.0) * 100.0;    //กำหนดว่า 4.0V → 0% 6.0V → 100% ถ้าแบตเตอรี่อยู่ที่ 5.0V = 50%
  if(batteryPercent > 100) batteryPercent = 100;         
  if(batteryPercent < 0) batteryPercent = 0;
}

void showOLED(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.printf("State: %s\n", motionState);
  display.printf("Temp: %.1fC Hum:%.1f%%\n", lastTemp, lastHum);
  display.printf("Dist: %.1f cm\n", lastDistance);
  display.printf("Speed: %d\n", requestedSpeed);
  display.printf("Emergency: %s\n", emergencyInterrupt ? "ON" : "OFF");
  display.printf("Batt: %.0f%% %.1fV\n", batteryPercent, batteryVoltage);
  display.display();
}

void mqttCallback(char* topic, byte* message, unsigned int length){
  char buf[16];
  size_t copyLen = (length < sizeof(buf)-1) ? length : sizeof(buf)-1;
  memcpy(buf, message, copyLen);
  buf[copyLen]=0;
  int end = (int)strlen(buf)-1;
  while(end>=0 && (buf[end]=='\r' || buf[end]=='\n' || buf[end]==' ')) { buf[end]=0; end--; }
  int start = 0;
  while(buf[start]==' ' && start <= end) start++;
  if(start>0) memmove(buf, buf+start, end-start+2);
  if(strlen(buf)>0){
    if(xSemaphoreTake(cmdMutex, pdMS_TO_TICKS(10))==pdTRUE){
      strncpy(latestCmdBuf, buf, sizeof(latestCmdBuf)-1);
      latestCmdBuf[sizeof(latestCmdBuf)-1]=0;
      hasCmd = true;
      xSemaphoreGive(cmdMutex);
    }
    // if(mqttClient.connected()){
    //   char ack[48];
    //   snprintf(ack, sizeof(ack), "recv %s", buf);
    //   mqttClient.publish(MQTT_TOPIC_REPLY, ack);
    // }
    publishAck("recv", buf);
  }
}

void setupWiFi(){
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
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("\nWiFi connected!");
    Serial.print("Wifi: ESP32_CarAP");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else Serial.println("\nFailed to connect WiFi");
}

void loop() {
  // FreeRTOS tasks handle main work
} 
