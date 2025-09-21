import serial, keyboard, time

BT_PORT = "COM10"
BAUD = 115200
ser = serial.Serial(BT_PORT, BAUD, timeout=1)

current_speed = 255 
def send(cmd):
    ser.write((cmd + "\n").encode())
    print("SEND:", cmd)

def active_cmd():
    if keyboard.is_pressed("a"): return "a"
    if keyboard.is_pressed("d"): return "d"
    if keyboard.is_pressed("w"): return "w"
    if keyboard.is_pressed("s"): return "s"
    return None

print("Hold WASD to drive. 1=Low 2=Med 3=High, i=interrupt, ESC=exit.")
last = None
try:
    while True:
        # interrupt
        if keyboard.is_pressed("i"):
            send("i")
            while keyboard.is_pressed("i"): time.sleep(0.05)

        # speed presets
        if keyboard.is_pressed("1"):
            current_speed = 100
            send(str(current_speed))
            print("Speed set to LOW =", current_speed)
            while keyboard.is_pressed("1"): time.sleep(0.05)

        if keyboard.is_pressed("2"):
            current_speed = 180
            send(str(current_speed))
            print("Speed set to MEDIUM =", current_speed)
            while keyboard.is_pressed("2"): time.sleep(0.05)

        if keyboard.is_pressed("3"):
            current_speed = 255
            send(str(current_speed))
            print("Speed set to HIGH =", current_speed)
            while keyboard.is_pressed("3"): time.sleep(0.05)

        # movement
        now = active_cmd()
        if now != last:
            if now is None:
                send("Idle")
            else:
                send(now)
            last = now

        if keyboard.is_pressed("esc"): break
        time.sleep(0.03)
finally:
    ser.close()
