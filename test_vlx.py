"""
import time
import numpy
import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE

print("Uploading firmware, please wait...")
vl53 = vl53l5cx.VL53L5CX()
print("Done!")
vl53.set_resolution(8 * 8)
vl53.enable_motion_indicator(8 * 8)
# vl53.set_integration_time_ms(50)

# Enable motion indication at 8x8 resolution
vl53.enable_motion_indicator(8 * 8)

# Default motion distance is quite far, set a sensible range
# eg: 40cm to 1.4m
vl53.set_motion_distance(400, 1400)

vl53.start_ranging()

while True:
    if vl53.data_ready():
        data = vl53.get_data()
        # 2d array of motion data (always 4x4?)
        motion = numpy.flipud(numpy.array(data.motion_indicator.motion[0:16]).reshape((4, 4)))
        # 2d array of distance
        distance = numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8)))
        # 2d array of reflectance
        reflectance = numpy.flipud(numpy.array(data.reflectance).reshape((8, 8)))
        # 2d array of good ranging data
        status = numpy.isin(numpy.flipud(numpy.array(data.target_status).reshape((8, 8))), (STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE))
        print(motion, distance, reflectance, status)
    time.sleep(0.1)
"""

import time
import math
from pymavlink import mavutil

def request_message_interval(master, message_id, frequency_hz):
    """ Demande l'envoi d'un message spécifique à une fréquence donnée """
    interval_us = int(1000000 / frequency_hz)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, interval_us, 0, 0, 0, 0, 0
    )

def main():
    print("Connexion au Pixhawk...")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
    
    print("Attente du Heartbeat (Signal de vie)...")
    master.wait_heartbeat()
    print("Cible connectée ! Démarrage de la lecture (Ctrl+C pour quitter)\n")

    # Demandes des messages à 10Hz
    request_message_interval(master, 30, 10)  # ATTITUDE
    request_message_interval(master, 74, 10)  # VFR_HUD (Altitude EKF3)
    request_message_interval(master, 173, 10) # RANGEFINDER (Lidar brut)
    request_message_interval(master, 132, 10) # DISTANCE_SENSOR (Lidar brut)

    roll_deg = 0.0
    pitch_deg = 0.0
    alt_ekf = 0.0
    alt_lidar_brut = 0.0
    
    last_print_time = time.time()

    try:
        while True:
            # On écoute tous ces messages
            msg = master.recv_match(
                type=['ATTITUDE', 'VFR_HUD', 'RANGEFINDER', 'DISTANCE_SENSOR'], 
                blocking=True, 
                timeout=0.1
            )
            
            if msg:
                if msg.get_type() == 'ATTITUDE':
                    roll_deg = math.degrees(msg.roll)
                    pitch_deg = math.degrees(msg.pitch)
                elif msg.get_type() == 'VFR_HUD':
                    alt_ekf = msg.alt
                elif msg.get_type() == 'RANGEFINDER':
                    alt_lidar_brut = msg.distance
                elif msg.get_type() == 'DISTANCE_SENSOR':
                    alt_lidar_brut = msg.current_distance / 100.0 # cm vers mètres

            # Affichage à 5Hz
            now = time.time()
            if now - last_print_time >= 0.2:
                print(f"\rLidar: {alt_lidar_brut:>5.2f}m | EKF: {alt_ekf:>5.2f}m | Pitch: {pitch_deg:>5.1f}° | Roll: {roll_deg:>5.1f}°  ", end="", flush=True)
                last_print_time = now

    except KeyboardInterrupt:
        print("\n\nArrêt du test.")

if __name__ == "__main__":
    main()