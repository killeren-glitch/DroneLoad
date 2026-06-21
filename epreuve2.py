import init_function as fa
import capteurs as capt
import time
import epreuve2_function as f
import mavlink
import head as h
import sensors as s


def run_epreuve2():

    """
    ---------- PHASE 1 : PASS THROUGH THE FIRST WINDOW ---------
                            WINDOW B
    """
    cap = fa.camera_init()
    detector, aruco_dict = fa.aruco_init()
    mavlink_command = mavlink.DroneController()
    sensors = s.HardwareManager()


    # Décollage à 1m et attendre d'arriver
    vx = 0
    vy = 0
    vz = -0.1
    yaw = 0
    while True:
        mavlink_command.send_velocity_body(vx, vy, vz, yaw)
        altitude = sensors.get_altitude()
        if altitude >= 1:break
        else: time.sleep(0.05)

    # Stabilisation 5 secondes
    f.keep_position(5, mavlink_command)

    # Avancer vers la fenêtre en vérifiant le capteur en continu (on s'arréte a 1 m de l'obstacle detecté)
    while True:
        distance = sensors.get_forward_distance()
        if distance is None:
            time.sleep(0.05)
            continue

        distance_m = distance / 1000.0

        if distance_m <= 1.0:
            # Trop proche d'un obstacle → stop
            mavlink_command.send_velocity_body(0, 0, 0, yaw)
            break

        # Avancer
        yaw = 0
        vx = 0.1
        mavlink_command.send_velocity_body(vx, 0.0, 0.0, yaw)
        time.sleep(0.05)  # ← laisser le temps à la boucle de tourner

    # Recherche fenêtre B
    found = f.search_window_with_height(cap, detector, mavlink_command, sensors)

    # Traversée
    if found:
        went_through = f.go_through_window(cap, detector,mavlink_command)
        if not went_through:
            print("Échec traversée fenêtre B")
            mavlink_command.land()
    else:
        print("Fenêtre B introuvable → atterrissage")
        mavlink_command.land()


