# pc_mqtt_hold_to_move.py
import paho.mqtt.client as mqtt
import keyboard, time

BROKER = "192.168.33.79"  # <-- แก้ให้ตรงกับเครื่องที่รัน Mosquitto
PORT   = 1883
TOPIC  = "car/cmd"

current_speed = 255

def send(client, msg):
    client.publish(TOPIC, msg, qos=1)
    print("SEND:", msg)

def active_cmd():
    if keyboard.is_pressed("a"): return "a"
    if keyboard.is_pressed("d"): return "d"
    if keyboard.is_pressed("w"): return "w"
    if keyboard.is_pressed("s"): return "s"
    return None

client = mqtt.Client()
client.connect(BROKER, PORT, 15)
client.loop_start()

print("Hold WASD to drive. Release to stop. 'i' toggles interrupt. ESC to exit.")
last = None
try:
    while True:
        if keyboard.is_pressed("i"):
            send(client, "i")
            while keyboard.is_pressed("i"): time.sleep(0.03)
        if keyboard.is_pressed("1"):
            current_speed = 100
            send(client, current_speed)
            print("Speed set to LOW =", current_speed)
            while keyboard.is_pressed("1"): time.sleep(0.05)

        if keyboard.is_pressed("2"):
            current_speed = 180
            send(client, current_speed)
            print("Speed set to MEDIUM =", current_speed)
            while keyboard.is_pressed("2"): time.sleep(0.05)

        if keyboard.is_pressed("3"):
            current_speed = 255
            send(client, current_speed)
            print("Speed set to HIGH =", current_speed)
            while keyboard.is_pressed("3"): time.sleep(0.05)

        now = active_cmd()
        if now != last:
            if now is None:
                send(client, "Idle")
            else:
                send(client, now)
            last = now

        if keyboard.is_pressed("esc"): break
        time.sleep(0.03)
finally:
    client.loop_stop()
    client.disconnect()
