import paho.mqtt.client as mqtt
import keyboard
import time
import re

MQTT_BROKER = "192.168.2.34"
MQTT_PORT = 1883
MQTT_TOPIC = "car/cmd"

# สร้าง client MQTT
client = mqtt.Client()
# เชื่อมต่อกับ MQTT Broker
client.connect(MQTT_BROKER, MQTT_PORT, keepalive=15)

# ส่งคำสั่งไปยัง ESP32
def send_cmd(key):
    cmd = key.name  # w, a, s, d
    if cmd in ["w", "a", "s", "d"]:
        # ส่งคำสั่ง "w" ไปยัง ESP32 (Move Forward etc..)
        client.publish(MQTT_TOPIC, cmd, qos=1)
        print(f"Sent {cmd}")

#ดึง int จากข้อความ
def extract_int(s):
    # หา pattern ตัวเลข (อาจมีลบหน้าได้)
    match = re.search(r'-?\d+', s)
    if match:
        return match.group(0)  # คืนค่าตัวเลขเป็น string เช่น "500" หรือ "-123"
    return None

# ใช้ client.loop_start() เพื่อให้ MQTT ทำงานอยู่ใน background
client.loop_start()

print("Press WASD to control car. Press ESC to exit.")

while True:
    if keyboard.is_pressed("w"):
        client.publish(MQTT_TOPIC, "w", qos=1)
        print("Sent w (Forward)")  # ส่งคำสั่งขับไปข้างหน้า
    elif keyboard.is_pressed("a"):
        client.publish(MQTT_TOPIC, "a", qos=1)
        print("Sent a (Left)")
    elif keyboard.is_pressed("s"):
        client.publish(MQTT_TOPIC, "s", qos=1)
        print("Sent s (Backward)")
    elif keyboard.is_pressed("d"):
        client.publish(MQTT_TOPIC, "d", qos=1)
        print("Sent d (Right)")
    elif keyboard.is_pressed("i"):
        client.publish(MQTT_TOPIC, "i", qos=1)
        print("Sent i (Interrupt)")
        time.sleep(0.05)
    elif keyboard.is_pressed("x"):
        client.publish(MQTT_TOPIC, "x", qos=1)
        print("Sent x (Stop)")
        time.sleep(0.05)

    elif keyboard.is_pressed("enter"):
        
        # รอปล่อยปุ่ม Enter ก่อนจะเริ่มรับ input
        while keyboard.is_pressed("enter"):
            time.sleep(0.05)
        try:
            raw_value = input("Enter number to setting speed: ").strip() #ตัดช่องว่างด้านหน้าและหลังออก เช่น " 500 " → "500"
            value = extract_int(raw_value)
            if value.lstrip("-").isdigit():#ตัดเครื่องหมายลบด้านหน้าทิ้งชั่วคราว (เพื่อให้รับเลขลบได้)
                client.publish(MQTT_TOPIC, value, qos=1)
                print(f"Sent {value} (Set speed)")
                
            else:
                print("Invalid input, not a number.")
        except Exception as e:
            print("Error:", e)

    # for digit in "0123456789":
    #     if keyboard.is_pressed(digit):
    #         client.publish(MQTT_TOPIC, digit, qos=1)
    #         print(f"Sent {digit} (Set speed)")
    
    # รอประมาณ 0.1 วินาทีเพื่อลดความเร็วในการส่งคำสั่ง
    time.sleep(0.1)

    # กด ESC เพื่อออกจากโปรแกรม
    if keyboard.is_pressed("esc"):
        break
    

keyboard.wait("esc")  # รอการกด ESC เพื่อออกจากโปรแกรม

# # จับการกดปุ่ม
# keyboard.on_press_key("w", send_cmd)
# keyboard.on_press_key("a", send_cmd)
# keyboard.on_press_key("s", send_cmd)
# keyboard.on_press_key("d", send_cmd)
# print("Press WASD to control car. Press ESC to exit.")
# # รอการกดปุ่ม ESC เพื่อปิดโปรแกรม
# keyboard.wait("esc")


