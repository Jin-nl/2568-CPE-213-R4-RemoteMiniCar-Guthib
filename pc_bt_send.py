import serial
import keyboard
import time
import re

BT_PORT = "COM4"   # เปลี่ยนเป็นพอร์ตของคุณ
BAUD = 115200

# เปิดการเชื่อมต่อ Bluetooth
ser = serial.Serial(BT_PORT, BAUD, timeout=1)

# ส่งคำสั่งไปยัง ESP32
def send_cmd(cmd):
    ser.write(f"{cmd}\n".encode())  # ส่งคำสั่งผ่าน Bluetooth
    print(f"Sent {cmd} to ESP32 via Bluetooth")  # แสดงคำสั่งที่ส่งออกไป

#ดึง int จากข้อความ
def extract_int(s):
    # หา pattern ตัวเลข (อาจมีลบหน้าได้)
    match = re.search(r'-?\d+', s)
    if match:
        return match.group(0)  # คืนค่าตัวเลขเป็น string เช่น "500" หรือ "-123"
    return None

print("Press WASD to control car. Press ESC to exit.")

while True:
    if keyboard.is_pressed("w"):
        send_cmd("w")
    elif keyboard.is_pressed("a"):
        send_cmd("a")
    elif keyboard.is_pressed("s"):
        send_cmd("s")
    elif keyboard.is_pressed("d"):
        send_cmd("d")
    elif keyboard.is_pressed("i"):
        send_cmd("i")
        time.sleep(0.05)
    elif keyboard.is_pressed("x"):
        send_cmd("x")
        time.sleep(0.05)

    elif keyboard.is_pressed("enter"):
        
        # รอปล่อยปุ่ม Enter ก่อนจะเริ่มรับ input
        while keyboard.is_pressed("enter"):
            time.sleep(0.05)
        try:
            raw_value = input("Enter number to setting speed: ").strip() #ตัดช่องว่างด้านหน้าและหลังออก เช่น " 500 " → "500"
            value = extract_int(raw_value)
            if value.lstrip("-").isdigit():#ตัดเครื่องหมายลบด้านหน้าทิ้งชั่วคราว (เพื่อให้รับเลขลบได้)
                #client.publish(MQTT_TOPIC, value, qos=1)
                send_cmd(value)
                print(f"Sent {value} (Set speed)")
                
            else:
                print("Invalid input, not a number.")
        except Exception as e:
            print("Error:", e)

    # รอประมาณ 0.1 วินาทีเพื่อลดความถี่ในการส่งคำสั่ง
    time.sleep(0.1)

    # กด ESC เพื่อออกจากโปรแกรม
    if keyboard.is_pressed("esc"):
        break

ser.close()  # ปิดการเชื่อมต่อ Bluetooth
keyboard.wait("esc")  # รอการกด ESC เพื่อออกจากโปรแกรม
