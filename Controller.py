import time, threading
import keyboard
import serial
import serial.tools.list_ports
import paho.mqtt.client as mqtt

BROKER_IP     = "192.168.33.79"
BROKER_PORT   = 1883
TOPIC_CMD     = "car/cmd"
TOPIC_REPLY   = "car/response"

BT_COM        = None
BT_BAUD       = 115200

CONNECT_TIMEOUT_S = 3.0
PRINT_PREFIX  = "[PCCTL]"

def log(*a):
    print(PRINT_PREFIX, *a)

def help_text():
    print("""
W/A/S/D : Hold-to-move
1/2/3   : Speed preset 200/230/255
SPACE   : Stop (Idle)
i       : Toggle interrupt
o       : whoami
R       : Reconnect current mode
M       : Switch mode (MQTT <-> BT)
h       : Help
ESC     : Exit
""".strip())

class ControllerBase:
    name = "BASE"
    def connect(self): ...
    def disconnect(self): ...
    def is_connected(self): return False
    def send(self, msg: str): ...
    def loop(self): ...
    def mode_hint(self): return self.name

class MqttController(ControllerBase):
    name = "MQTT"
    def __init__(self, host, port, topic_cmd, topic_reply):
        self.host = host
        self.port = port
        self.topic_cmd = topic_cmd
        self.topic_reply = topic_reply
        self.cli = mqtt.Client(protocol=mqtt.MQTTv311, transport="tcp")
        self.cli.on_connect = self._on_connect
        self.cli.on_message = self._on_msg
        self._connected = False
        self._connecting = False
        self._hb_running = False
        self.cli.will_set("car/ctl/pc/status", payload="offline", qos=1, retain=True)

    def _on_connect(self, client, userdata, flags, rc):
        self._connected = (rc == 0)
        if self._connected:
            log("MQTT connected ->", self.host)
            try:
                self.cli.subscribe(self.topic_reply, qos=1)
                self.cli.publish("car/ctl/pc/status", "online", qos=1, retain=True)
            except Exception as e:
                log("MQTT subscribe/publish err:", e)
        else:
            log("MQTT connect rc=", rc)

    def _on_msg(self, client, userdata, msg):
        try:
            payload = msg.payload.decode(errors="ignore")
        except:
            payload = str(msg.payload)
        print(f"[RESP] {msg.topic} -> {payload}")

    def connect(self):
        if self._connecting:
            return
        self._connecting = True
        self._connected = False
        try:
            self.cli.connect(self.host, self.port, keepalive=15)
            self.cli.loop_start()
            t0 = time.time()
            while time.time() - t0 < CONNECT_TIMEOUT_S and not self._connected:
                time.sleep(0.05)
            if self._connected and not self._hb_running:
                self._hb_running = True
                threading.Thread(target=self._heartbeat_loop, daemon=True).start()
        except Exception as e:
            log("MQTT connect exception:", e)
        finally:
            self._connecting = False
        return self._connected

    def disconnect(self):
        try:
            self.cli.publish("car/ctl/pc/status", "offline", qos=1, retain=True)
        except: pass
        try:
            self.cli.loop_stop()
        except: pass
        try:
            self.cli.disconnect()
        except: pass
        self._connected = False
        self._hb_running = False

    def is_connected(self):
        return self._connected

    def send(self, msg: str):
        if not self._connected: return False
        try:
            if msg is None:
                msg = "Idle"
            self.cli.publish(self.topic_cmd, msg, qos=1)
            print("SEND (MQTT):", msg)
            return True
        except Exception as e:
            log("MQTT send err:", e)
            return False

    def loop(self):
        pass

    def _heartbeat_loop(self):
        while self._hb_running:
            try:
                if self._connected:
                    self.cli.publish("car/ctl/pc/heartbeat", str(time.time()), qos=0, retain=False)
            except:
                pass
            time.sleep(0.5)

class BtSerialController(ControllerBase):
    name = "BT"
    def __init__(self, com=None, baud=115200):
        self.user_com = com
        self.baud = baud
        self.ser = None

    def _auto_find_port(self):
        ports = list(serial.tools.list_ports.comports())
        def score(p):
            s = (p.device or "") + " " + (p.description or "") + " " + (p.hwid or "")
            s = s.lower()
            sc = 0
            if "bluetooth" in s: sc += 5
            if "standard serial over bluetooth" in s: sc += 4
            if "esp" in s: sc += 2
            if "com" in (p.device or "").lower(): sc += 1
            return sc
        ports.sort(key=score, reverse=True)
        return [p.device for p in ports]

    def connect(self):
        cand = []
        if self.user_com:
            cand.append(self.user_com)
        cand += [c for c in self._auto_find_port() if c not in cand]
        for dev in cand:
            try:
                self.ser = serial.Serial(dev, self.baud, timeout=0.05)
                self.send("whoami")
                time.sleep(0.2)
                log("BT connected ->", dev)
                return True
            except Exception:
                try:
                    if self.ser: self.ser.close()
                except: pass
                self.ser = None
        return False

    def disconnect(self):
        try:
            if self.ser: self.ser.close()
        except: pass
        self.ser = None

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def send(self, msg: str):
        if not self.is_connected(): return False
        try:
            if msg is None:
                msg = "Idle"
            self.ser.write((msg+"\n").encode())
            print("SEND (BT):", msg)
            return True
        except Exception as e:
            log("BT send err:", e)
            self.disconnect()
            return False

    def loop(self):
        if not self.is_connected(): return
        try:
            data = self.ser.read(256)
            if data:
                try:
                    txt = data.decode(errors="ignore")
                except:
                    txt = str(data)
                for line in txt.splitlines():
                    if line.strip():
                        print(f"[RESP] bt -> {line.strip()}")
        except Exception:
            self.disconnect()

class Runner:
    def __init__(self):
        self.mqtt = MqttController(BROKER_IP, BROKER_PORT, TOPIC_CMD, TOPIC_REPLY)
        self.bt   = BtSerialController(BT_COM, BT_BAUD)
        self.mode = None
        self.last_cmd = None
        self._deb = {k:False for k in ["i","1","2","3","space","o","R","M","h"]}

    def try_connect(self, prefer="MQTT"):
        order = ["MQTT","BT"] if prefer=="MQTT" else ["BT","MQTT"]
        for m in order:
            if m=="MQTT":
                log("Connecting via MQTT ...")
                if self.mqtt.connect():
                    self.mode = "MQTT"
                    log("MODE =", self.mode)
                    return True
            else:
                log("Connecting via BT ...")
                if self.bt.connect():
                    self.mode = "BT"
                    log("MODE =", self.mode)
                    return True
        self.mode = None
        log("No connection available.")
        return False

    def disconnect_all(self):
        self.mqtt.disconnect()
        self.bt.disconnect()
        self.mode = None

    def send(self, msg):
        if self.mode == "MQTT":
            return self.mqtt.send(msg)
        elif self.mode == "BT":
            return self.bt.send(msg)
        else:
            return False

    def pump_inputs(self):
        def edge(key):
            if keyboard.is_pressed(key):
                if not self._deb[key]:
                    self._deb[key] = True
                    return True
            else:
                self._deb[key] = False
            return False

        if edge("i"):
            self.send("i")

        if edge("1"):
            self.send("1"); print("Speed -> 200")
        if edge("2"):
            self.send("2"); print("Speed -> 230")
        if edge("3"):
            self.send("3"); print("Speed -> 255")

        if edge("o"):
            self.send("whoami")

        if edge("space"):
            self.send("Idle")
            self.last_cmd = "Idle"

        now = None
        if keyboard.is_pressed("a"): now = "a"
        elif keyboard.is_pressed("d"): now = "d"
        elif keyboard.is_pressed("w"): now = "w"
        elif keyboard.is_pressed("s"): now = "s"

        if now != self.last_cmd:
            self.send(now if now else "Idle")
            self.last_cmd = now

        if edge("R"):
            log("Reconnecting current mode ...")
            if self.mode == "MQTT":
                self.mqtt.disconnect()
                self.mqtt.connect()
            elif self.mode == "BT":
                self.bt.disconnect()
                self.bt.connect()
            time.sleep(0.2)

        if edge("M"):
            log("Switch mode requested.")
            cur = self.mode
            self.disconnect_all()
            pref = "BT" if cur == "MQTT" else "MQTT"
            self.try_connect(pref)
            time.sleep(0.2)

        if edge("h"):
            help_text()

    def loop(self):
        if self.mode == "MQTT":
            self.mqtt.loop()
        elif self.mode == "BT":
            self.bt.loop()

def main():
    log("Auto controller starting...")
    help_text()
    r = Runner()
    r.try_connect(prefer="MQTT")
    try:
        while True:
            r.pump_inputs()
            r.loop()
            if keyboard.is_pressed("esc"):
                break
            time.sleep(0.03)
    finally:
        r.disconnect_all()
        log("Bye.")

if __name__ == "__main__":
    main()