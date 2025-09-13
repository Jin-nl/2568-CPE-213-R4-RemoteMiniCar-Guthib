#include <WiFi.h>
#include <PubSubClient.h>
#include <BluetoothSerial.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <cstring> // for strlen, strcmp, etc.

// ----- CONFIG -----
#define WIFI_STA_SSID   "btm02_2.4G"
#define WIFI_STA_PASS   "136167118"
#define WIFI_AP_SSID    "ESP32_CarAP"
#define WIFI_AP_PASS    "guthib123"  // หรือ "" ถ้าต้องการ open AP

#define MQTT_SERVER_IP  "192.168.2.33"
#define MQTT_PORT       1883
#define MQTT_TOPIC_CMD  "car/cmd"
#define MQTT_TOPIC_REPLY "car/response"  // Topic สำหรับส่ง ACK

#define DHTPIN          15
#define DHTTYPE         DHT11

#define TRIG_PIN        27
#define ECHO_PIN        34
#define OBSTACLE_CM     10

// Motor Pins (front axle + rear axle)
#define FRONT_PWM_PIN     18
#define FRONT_DIR_PIN1    32
#define FRONT_DIR_PIN2    33
#define FRONT_PWM_CH      0

#define SERVO_PIN         13

#define REAR_PWM_PIN      19
#define REAR_DIR_PIN1     25
#define REAR_DIR_PIN2     26
#define REAR_PWM_CH       1

#define LED_BLUE_PIN    2
#define LED_RED_PIN     4
#define BUTTON_INT_PIN  14

#define OLED_SDA        21
#define OLED_SCL        22
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1

// ----- GLOBALS -----
DHT dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BluetoothSerial BT;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Servo steering;

volatile bool emergencyInterrupt = false;
volatile unsigned long lastInterruptMillis = 0;

// แทนการใช้ String ในส่วนรับคำสั่ง: ใช้ buffer เล็ก ๆ (char array)
static char latestCmdBuf[16];    // เก็บคำสั่งล่าสุด (ไม่เกิน 15 chars)
static bool hasCmd = false;      // flag ว่ามีคำสั่งใหม่
SemaphoreHandle_t cmdMutex;

// สถานะ motion เก็บเป็น char[] เพื่อลดการใช้ String
char motionState[24] = "Stopped";

int requestedSpeed = 150;
float lastTemp=0, lastHum=0, lastDistance=999;

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
void processCommandC(const char* cmd); // ใช้ char* แทน String
void publishAck(const char* cmd, const char* result); // ส่ง ACK ทาง MQTT

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_BLUE_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  // PWM setup: channels และ attach pins
  ledcSetup(FRONT_PWM_CH, 20000, 8); ledcAttachPin(FRONT_PWM_PIN, FRONT_PWM_CH); ledcWrite(FRONT_PWM_CH,0);
  ledcSetup(REAR_PWM_CH,  20000, 8); ledcAttachPin(REAR_PWM_PIN, REAR_PWM_CH);   ledcWrite(REAR_PWM_CH,0);

  // motor direction pins
  pinMode(FRONT_DIR_PIN1, OUTPUT); pinMode(FRONT_DIR_PIN2, OUTPUT);
  pinMode(REAR_DIR_PIN1, OUTPUT); pinMode(REAR_DIR_PIN2, OUTPUT);

  // ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // emergency button (INPUT_PULLUP) -> ต่อปุ่มกับ GND เมื่อกด
  pinMode(BUTTON_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_INT_PIN), handleButtonInterrupt, FALLING);

  // sensors + display
  dht.begin();
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  // steering servo
  steering.attach(SERVO_PIN);
  steering.write(90); // ตรงกลาง

  if(!BT.begin("ESP32_Car")) Serial.println("BT failed");

  setupWiFi();
  mqttClient.setServer(MQTT_SERVER_IP, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  cmdMutex = xSemaphoreCreateMutex();

  // FreeRTOS tasks
  xTaskCreate(commsTask, "commsTask", 4096, NULL, 2, NULL);
  xTaskCreate(sensorsTask, "sensorsTask", 4096, NULL, 1, NULL);
  xTaskCreate(controlTask, "controlTask", 4096, NULL, 2, NULL);
}

// -------------------- TASKS --------------------
void commsTask(void *pvParameters) {
  for(;;){
    // WiFi reconnect (ถ้าหลุดจะพยายามเชื่อมใหม่)
    if(WiFi.status() != WL_CONNECTED){
      Serial.println("WiFi disconnected! Reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    }

    bool wifiConnected = (WiFi.status()==WL_CONNECTED);
    digitalWrite(LED_BLUE_PIN, wifiConnected ? HIGH : LOW);

    // MQTT reconnect และ loop
    if(wifiConnected){
      if(!mqttClient.connected()){
        Serial.println("MQTT reconnecting...");
        if(mqttClient.connect("ESP32_Car")){
          mqttClient.subscribe(MQTT_TOPIC_CMD, 1);
          Serial.println("MQTT connected!");
        }
      } else {
        mqttClient.loop(); // สำคัญ: ให้ callback ทำงาน
      }
    }

    // อ่าน Bluetooth (ไม่ใช้ String) -> readBytesUntil('\n')
    if(BT.available()){
      char buf[16];
      memset(buf,0,sizeof(buf));
      size_t len = BT.readBytesUntil('\n', buf, sizeof(buf)-1); // อ่านจน '\n' หรือเต็ม
      if(len>0){
        // ลบ CR และช่องว่างทางซ้าย/ขวา
        // trim ใน-place
        // remove trailing CR/LF
        while(len>0 && (buf[len-1]=='\r' || buf[len-1]=='\n' || buf[len-1]==' ')) { buf[len-1]=0; len--; }
        // remove leading spaces
        size_t start=0;
        while(buf[start]==' ' && start < len) start++;
        if(start>0){
          // shift left
          memmove(buf, buf+start, len-start+1);
          len -= start;
        }
        if(len>0){
          // เก็บคำสั่งลง shared buffer แบบ thread-safe
          if(xSemaphoreTake(cmdMutex, (TickType_t)10/portTICK_PERIOD_MS)==pdTRUE){
            strncpy(latestCmdBuf, buf, sizeof(latestCmdBuf)-1);
            latestCmdBuf[sizeof(latestCmdBuf)-1]=0;
            hasCmd = true;
            xSemaphoreGive(cmdMutex);
          }
          // ไม่พิมพ์ซ้ำที่นี่ (จะพิมพ์ใน controlTask หลัง process)
        }
      }
    }

    vTaskDelay(50/portTICK_PERIOD_MS);
  }
}

void sensorsTask(void *pvParameters){
  for(;;){
    float t=dht.readTemperature(); if(!isnan(t)) lastTemp=t;
    float h=dht.readHumidity(); if(!isnan(h)) lastHum=h;
    lastDistance=readUltrasonicCM();
    if(lastDistance<=OBSTACLE_CM && lastDistance>0){
      stopMotors();
      strncpy(motionState, "Stopped(Obstacle)", sizeof(motionState)-1);
      motionState[sizeof(motionState)-1]=0;
    }
    showOLED();
    vTaskDelay(1000/portTICK_PERIOD_MS);
    void checkHeapAndRecover();
  }
}

void controlTask(void *pvParameters){
  for(;;){
    if(emergencyInterrupt){
      digitalWrite(LED_RED_PIN,HIGH);
      digitalWrite(LED_BLUE_PIN,LOW);
      stopMotors();
    } else digitalWrite(LED_RED_PIN,LOW);

    // ถ้ามีคำสั่งใหม่ ให้ดึงมา process (ใช้ mutex)
    if(xSemaphoreTake(cmdMutex,(TickType_t)10/portTICK_PERIOD_MS)==pdTRUE){
      if(hasCmd){
        char cmdLocal[16];
        strncpy(cmdLocal, latestCmdBuf, sizeof(cmdLocal)-1);
        cmdLocal[sizeof(cmdLocal)-1]=0;
        hasCmd = false;                       // รับแล้ว
        latestCmdBuf[0]=0;
        xSemaphoreGive(cmdMutex);

        // process และพิมพ์ผล
        processCommandC(cmdLocal);
        Serial.printf("cmd=%s -> motionState=%s\n", cmdLocal, motionState);

        // ส่ง ACK กลับทาง MQTT (ถ้าเชื่อม)
        if(mqttClient.connected()){
          // สร้างข้อความ ACK เช่น "w forward success"
          char ackBuf[64];
          snprintf(ackBuf, sizeof(ackBuf), "%s %s success", cmdLocal, motionState);
          mqttClient.publish(MQTT_TOPIC_REPLY, ackBuf);
        }
      } else {
        xSemaphoreGive(cmdMutex);
      }
    }

    // ถ้าไม่มี emergency ให้สั่งมอเตอร์ตาม motionState
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

    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

// -------------------- HELPERS --------------------
// แปลงคำสั่งจาก char* เป็น action (ไม่ใช้ String)
void processCommandC(const char* cmd){
  if(cmd == NULL) return;
  // คำสั่งตัวอักษรเดียว (case-sensitive ตามโค้ดเดิม)
  if(strcmp(cmd, "w")==0){
    strncpy(motionState, "Forward", sizeof(motionState)-1);
  } else if(strcmp(cmd, "s")==0){
    strncpy(motionState, "Backward", sizeof(motionState)-1);
  } else if(strcmp(cmd, "a")==0){
    strncpy(motionState, "Left", sizeof(motionState)-1);
  } else if(strcmp(cmd, "d")==0){
    strncpy(motionState, "Right", sizeof(motionState)-1);
  } else if(strcmp(cmd, "x")==0 || strcmp(cmd,"stop")==0){
    strncpy(motionState, "Stopped", sizeof(motionState)-1);
  } else if(strcmp(cmd, "I")==0){
    emergencyInterrupt = !emergencyInterrupt;
  } else {
    // ถ้าเป็นตัวเลข ปรับความเร็ว
    int v = atoi(cmd);
    if(v > 0) requestedSpeed = constrain(v, 0, 255);
  }
  // ป้องกันล้นและให้ string จบด้วย null
  motionState[sizeof(motionState)-1] = 0;
}

// ฟังก์ชันส่ง ACK (ถ้าต้องการเรียกแยก)
void publishAck(const char* cmd, const char* result){
  if(mqttClient.connected()){
    char buff[64];
    snprintf(buff, sizeof(buff), "%s %s success", cmd, result);
    mqttClient.publish(MQTT_TOPIC_REPLY, buff);
  }
}

void stopMotors(){ 
  setMotorAxle(0,0,FRONT_PWM_CH,FRONT_DIR_PIN1,FRONT_DIR_PIN2);
  setMotorAxle(0,0,REAR_PWM_CH,REAR_DIR_PIN1,REAR_DIR_PIN2);
}

void setAxles(int speedFront,int dirFront,int speedRear,int dirRear){
  setMotorAxle(speedFront, dirFront, FRONT_PWM_CH, FRONT_DIR_PIN1, FRONT_DIR_PIN2);
  setMotorAxle(speedRear, dirRear, REAR_PWM_CH, REAR_DIR_PIN1, REAR_DIR_PIN2);
}

// pwmVal: 0-255, dir: 0 stop, 1 forward, 2 backward
void setMotorAxle(int pwmVal,int dir,int channel,int dir1,int dir2){
  pwmVal=constrain(pwmVal,0,255);
  if(dir==0){ digitalWrite(dir1,LOW); digitalWrite(dir2,LOW); ledcWrite(channel,0); }
  else if(dir==1){ digitalWrite(dir1,HIGH); digitalWrite(dir2,LOW); ledcWrite(channel,pwmVal); }
  else if(dir==2){ digitalWrite(dir1,LOW); digitalWrite(dir2,HIGH); ledcWrite(channel,pwmVal); }
}

float readUltrasonicCM(){
  digitalWrite(TRIG_PIN,LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN,HIGH); delayMicroseconds(10); digitalWrite(TRIG_PIN,LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 20000);
  if(dur == 0) return 999;
  return (dur / 2.0) / 29.1;
}

void IRAM_ATTR handleButtonInterrupt(){
  unsigned long now=millis();
  if(now-lastInterruptMillis<300) return;
  lastInterruptMillis=now;
  emergencyInterrupt=!emergencyInterrupt;
}

void checkHeapAndRecover(){
  size_t freeHeap = ESP.getFreeHeap();  // ตรวจสอบ heap memory ที่ยังว่าง (หน่วย: byte)
  static size_t lowestSeen = SIZE_MAX;  // เก็บค่าต่ำสุดที่เคยเห็นไว้ (แค่ตอนเริ่มจะเป็นค่าสูงสุด)
  if(freeHeap < lowestSeen) lowestSeen = freeHeap;  // อัปเดต lowest ถ้าต่ำกว่าครั้งก่อน
  Serial.printf("Free heap: %u bytes, lowest seen: %u\n", 
                (unsigned)freeHeap, (unsigned)lowestSeen);  // แสดงผล heap ปัจจุบันและต่ำสุดที่เคยมี
  if(freeHeap < 18000){  // ถ้ามีน้อยกว่า 18,000 bytes (ประมาณ 18KB)
    Serial.println("Free heap too low -> restarting to recover");
    vTaskDelay(pdMS_TO_TICKS(200)); // ใช้ vTaskDelay แทน delay() เมื่ออยู่ใน FreeRTOS task
    ESP.restart();       // รีสตาร์ท ESP32 เพื่อเคลียร์ heap
  }
}

void showOLED(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.printf("State: %s\n", motionState);
  display.printf("Temp: %.1fC Hum: %.1f%%\n", lastTemp, lastHum);
  display.printf("Dist: %.1f cm\n", lastDistance);
  display.printf("Speed: %d\n", requestedSpeed);
  display.display();
}

// MQTT callback — รับ payload เป็น byte* -> copy เป็น char[] -> processCommandC
void mqttCallback(char* topic, byte* message, unsigned int length){
  char buf[16];
  size_t copyLen = (length < sizeof(buf)-1) ? length : sizeof(buf)-1;
  memcpy(buf, message, copyLen);
  buf[copyLen]=0;
  // trim CR/LF/space
  int end = (int)strlen(buf)-1;
  while(end>=0 && (buf[end]=='\r' || buf[end]=='\n' || buf[end]==' ')) { buf[end]=0; end--; }
  int start = 0;
  while(buf[start]==' ' && start <= end) start++;
  if(start>0) memmove(buf, buf+start, end-start+2);
  if(strlen(buf)>0){
    // เก็บลง latestCmdBuf โดยไม่ต้องใช้ String
    if(xSemaphoreTake(cmdMutex, (TickType_t)10/portTICK_PERIOD_MS)==pdTRUE){
      strncpy(latestCmdBuf, buf, sizeof(latestCmdBuf)-1);
      latestCmdBuf[sizeof(latestCmdBuf)-1]=0;
      hasCmd = true;
      xSemaphoreGive(cmdMutex);
    }
    // เราจะ process คำสั่งใน controlTask (centralized) เพื่อหลีกเลี่ยง race
    // แต่เราส่ง ACK เล็ก ๆ ที่บอกว่าได้รับข้อความแล้ว (optional)
    if(mqttClient.connected()){
      char ack[48];
      snprintf(ack, sizeof(ack), "recv %s", buf);
      mqttClient.publish(MQTT_TOPIC_REPLY, ack);
    }
  }
}

void setupWiFi(){
  // ถ้าต้องการ open AP ให้เปลี่ยนเป็น WiFi.softAP(WIFI_AP_SSID);
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
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else Serial.println("\nFailed to connect WiFi");
}

void loop() {
  // FreeRTOS tasks handle main work
}
