# ESP32 Hybrid Car – Wi-Fi + MQTT + Bluetooth + Battery Monitor
This project is a smart remote-controlled car using **ESP32**, supporting hybrid communication via both **Wi-Fi (MQTT)** and **Bluetooth**.  
It enables remote operation, obstacle detection using an ultrasonic sensor, environment monitoring via DHT11,  
and includes a **Battery Monitor**, **OLED Display**, and an **Emergency Stop** system for safety.

## Hybrid Communication
- Connects to Wi-Fi for communication via MQTT  
- Supports Bluetooth and can work simultaneously with Wi-Fi (hybrid)  
- Automatically reconnects to Wi-Fi and MQTT if the connection is lost  

## MQTT Control
- Receives commands via MQTT Topic: car/cmd
- Sends ACK / status via MQTT Topic: car/response 
- Uses a unique MQTT client ID generated from ESP32's MAC address  
- Includes Exponential Backoff for stable MQTT reconnections  

## Motor Control
- All-wheel drive with PWM controlled DC motor
- Direction control commands:  
  w = Forward, s = Backward, a = Turn Left, d = Turn Right, x = Stop  
- Speed control via numeric values (range: 0–255)  
- "Steering" control by stopping the front wheels - left/right (no servo)
- Emergency Stop supported via an interrupt button: 'i' 

## Sensors & Detection
- **DHT11**: Measures temperature and humidity  
- **Ultrasonic Sensor**: Detects obstacles  
  - Automatically stops motors when objects are too close  
- **Battery Monitor**:
  - Reads battery voltage via ADC  
  - Displays percentage and voltage on OLED  
  - Low battery alert below 20%  

## OLED Display
- Displays car status, temperature, humidity, ultrasonic distance, speed, emergency status, and battery level

## Safety & Stability
- Monitors Free Heap Memory and automatically restarts ESP32 if memory is too low  
- Interrupt button allows for immediate Emergency Stop or toggle of Emergency status  

## Software Structure
- **FreeRTOS Tasks**
  - commsTask: Manages Wi-Fi, MQTT, and Bluetooth  
  - sensorsTask: Reads DHT, ultrasonic, and battery sensors  
  - controlTask: Processes commands and controls motors  
- Uses **Semaphore** and **Mutex** for thread-safe data access  
- Avoids dynamic String objects to prevent memory fragmentation  

## platformio.ini Setup
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
board_build.partitions = min_spiffs.csv
monitor_port = COM3 
monitor_speed = 115200 
upload_speed = 115200
lib_deps = 
    adafruit/Adafruit SSD1306@^2.5.15
    arduinogetstarted/ezButton@^1.0.6
    adafruit/DHT sensor library
    adafruit/Adafruit Unified Sensor
    knolleary/PubSubClient
    madhephaestus/ESP32Servo@^1.1.0

## Installation
-  Download and install MQTT Broker: https://mosquitto.org/download/
-  Install Python dependencies:
   - pip install paho-mqtt
   - pip install pyserial
 
## Usage
-  Edit Wi-Fi SSID and Password in the STA configuration section of the code
-  Set the IP address to match the machine running the MQTT Broker (default port: 1883)
-  Start the Mosquitto MQTT broker
-  Upload the main code to the ESP32
-  Open pc_mqtt_control.py:
   - Set the IP address of the MQTT broker machine
   - Run the script to control via keyboard
-  Open pc_bt_control.py:
   - Set the Bluetooth serial port
   - Run the script to control via Bluetooth using keyboard
