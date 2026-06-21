"""
                     ---------- PHASE 2 : PASS THROUGH THE SECOND WINDOW ---------
                                           WINDOW C
   """

found = False

while True:
    distance = fwrd_distance_reader.get_distance()
    yaw = 0
    d = 0.5
    vx = 0.2

    if distance > 0.7:
        temps = d / vx
        # Avancer de 0.5m et attendre d'arriver
        mavlink_command.send_velocity_body_with_stop_time(vx, 0, 0, yaw, temps)

        # ------- Vérification gauche -------
        mavlink_command.rotate_drone(master, -90)  # face à gauche
        time.sleep(0.5)
        dist_left = fwrd_distance_reader.get_distance()
        time_duration_g = 1.0 if dist_left > 0.7 else 0.5

        # ------- Vérification droite -------
        mavlink_command.rotate_drone(master, 180)  # face à droite
        time.sleep(0.5)
        dist_right = fwrd_distance_reader.get_distance()
        time_duration_d = 1.0 if dist_right > 0.7 else 0.5

        # ------- Retour face avant -------
        mavlink_command.rotate_drone(master, -90)  # face avant
        time.sleep(0.5)

        # ------- Recherche fenêtre C -------
        search_pattern = [
            # (vx, vy, durée_sec)
            (0.0, 0.2, time_duration_d),  # droite
            (0.0, -0.2, time_duration_g)  # gauche
        ]
        found = f.search_window_with_height(
            cap, detector, mavlink_command, altitude_reader, search_pattern)

        if found:
            break

    else:
        # Obstacle devant → reculer
        d = 0.5
        temps = d / vx
        # Avancer de 0.5m et attendre d'arriver
        mavlink_command.send_velocity_body_with_stop_time(-vx, 0, 0, yaw, temps)
        continue

# Traversée fenêtre C
if found:
    went_through = f.go_through_window(cap, detector, mavlink_command)

# Zone E + demi-tour (reach zone C and hover
"""
Après traversée de C, avance vers E et hover 5 secondes.
"""
# Avancer encore un peu après C vers le bord
# Avancer de 0.5m et attendre d'arriver
vx = 0.2
temps = h.DISTANCE_C_TO_E / vx
mavlink_command.send_velocity_body_with_stop_time(vx, 0, 0, yaw, temps)

# Hover 5 secondes
print("Hovering zone E...")
time.sleep(5.0)

print("Demi-tour...")
mavlink_command.rotate_drone(180)

"""
                  ---------- PHASE 3 : Retour en A ------------

"""
while True:
    distance = fwrd_distance_reader.get_distance()

    # on avance de 1 métre , on cherche la fenetre E puis on la traverse
    if distance is not None and distance > 1.0:
        vx = -0.1
        vy = 0
        vz = 0.0
        yaw = 0
        mavlink_command.send_velocity_body(vx, vy, vz, yaw)

        went_through = f.go_through_window(cap, detector, master)

    # on vérifie que le premier obstacle est assez loin et qu'on peut avancer
    distance = fwrd_distance_reader.get_distance()

    if distance is not None and distance > 0.5:
        d = 0.5
        temps = d / vx
        # Avancer de 0.5m et attendre d'arriver
        mavlink_command.send_velocity_body_with_stop_time(-vx, 0, 0, yaw, temps)
    else:
        mavlink_command.send_velocity_body(0.0, 0.0, 0.0)

    # la ca veut dire qu'on est devant le premier obstacle et que l'on peut tourner vers le deuxiéme
    mavlink_command.rotate_drone(-90)  # face à gauche
    time.sleep(0.5)

    # on vérifie que le deuxiéme obstacle est bien devant nous
    if distance is not None and distance > 0.5:
        d = 0.5
        dx = abs(x - distance)
        temps = d / vx
        # Avancer de 0.5m et attendre d'arriver
        mavlink_command.send_velocity_body_with_stop_time(-vx, 0, 0, yaw, temps)

    # on tourne a droite
    mavlink_command.rotate_drone(90)  # face à gauche
    time.sleep(0.5)

    # on avance de 1 métre
    distance = fwrd_distance_reader.get_distance()

    # on avance de 1 métre , on cherche la fenetre E puis on la traverse
    if distance is not None and distance > 1.0:
        d = 1.0
        temps = d / vx
        # Avancer de 0.5m et attendre d'arriver
        mavlink_command.send_velocity_body_with_stop_time(-vx, 0, 0, yaw, temps)

        went_through = f.go_through_window(cap, detector, master)

    # Phase 3 : pivoter caméra vers le bas + atterrissage sur ArUco 0
    # servo.look_down()
    time.sleep(0.5)

    print("Recherche ArUco 0 pour atterrissage...")
    mavlink_command.land()
