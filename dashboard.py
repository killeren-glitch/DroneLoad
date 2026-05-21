import paho.mqtt.client as mqtt
import json
import threading
import time

# --- CONFIGURATION ---
# L'IP de ta box hAP lite ou de ton Raspberry Pi 5
BROKER_IP = "192.168.2.3"


# --- RÉCEPTION MQTT ---
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"\n[+] Connecté au drone sur {BROKER_IP}")
    client.subscribe("drone/logs")
    client.subscribe("drone/telemetry")

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode("utf-8")

    if topic == "drone/logs":
        print(f"\n[DRONE LOG] {payload}")

    elif topic == "drone/telemetry":
        data = json.loads(payload)
        # On utilise le retour chariot (\r) pour rafraîchir la ligne sans scroller à l'infini
        info = (f"\r[TÉLÉMÉTRIE] "
                f"Mode: {data['mode']:<10} | "
                f"FPS: {data['fps']:<3} | "
                f"CPU: {data['cpu_percent']:<4}% | "
                f"RAM: {data['ram_percent']:<4}% | "
                f"Vol: {data['flight_time_sec']}s | "
                f"Batt: {data['batt_v']}V ({data['batt_pct']}%)")
        print(info, end="", flush=True)


# --- THREAD D'ÉCOUTE ---
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "PC_Dashboard")
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_IP, 1883, 60)
client.loop_start()

# --- INTERFACE COMMANDES (Menu CLI) ---
time.sleep(1)  # Attendre la connexion
print("\n--- TABLEAU DE BORD DRONE ---")
print("Commandes Vol : [1] Mode Attente  [2] Mode Epreuve1  [3] Mode Epreuve2")
print("Commandes Sys : [A] Armer  [D] Désarmer")
print("Commandes Pi  : [R] Reboot Pi  [S] Restart Script")
print("Commandes Vid : [B] Toggle Black/Normal Background")
print("Quitter       : [Q]")

black_bg_state = False

while True:
    cmd = input("\n> ").strip().upper()

    if cmd == '1':
        client.publish("drone/cmd", json.dumps({"action": "set_mode", "mode": "attente"}))
    elif cmd == '2':
        client.publish("drone/cmd", json.dumps({"action": "set_mode", "mode": "epreuve1"}))
    elif cmd == '3':
        client.publish("drone/cmd", json.dumps({"action": "set_mode", "mode": "epreuve2"}))
    elif cmd == 'A':
        client.publish("drone/cmd", json.dumps({"action": "arm"}))
    elif cmd == 'D':
        client.publish("drone/cmd", json.dumps({"action": "disarm"}))
    elif cmd == 'B':
        black_bg_state = not black_bg_state
        client.publish("drone/cmd", json.dumps({"action": "video_bg", "black": black_bg_state}))
    elif cmd == 'S':
        print("Envoi commande RESTART SCRIPT...")
        client.publish("drone/system", json.dumps({"action": "restart_script"}))
    elif cmd == 'R':
        confirm = input("T'es sûr de vouloir REBOOT LE PI 5 (O/N) ? ").upper()
        if confirm == 'O':
            client.publish("drone/system", json.dumps({"action": "reboot"}))
    elif cmd == 'Q':
        print("Fermeture du Dashboard.")
        client.loop_stop()
        break