def run_epreuve1(drone, hw, arucos_data, frame_width):
    """
    S'exécute à chaque tour de boucle (tick).
    aruco_data : dictionnaire {id_aruco: (x, y)}
    """
    TARGET_ID = 0
    DISTANCE_MIN = 0.8  # S'arrêter à 80 cm du mur/fenêtre

    distance_avant = hw.get_forward_distance()

    if TARGET_ID in arucos_data:
        # L'ArUco est vu !
        center_x, center_y = arucos_data[TARGET_ID]

        # Calcul de l'erreur par rapport au centre de l'image
        erreur_x = center_x - (frame_width / 2)

        # --- Contrôleur Proportionnel simple ---

        # Rotation (Yaw) : On tourne vers le tag
        k_yaw = 0.002
        yaw_rate = erreur_x * k_yaw

        # Avancement (Vx) : On avance si le tag est à peu près centré
        vx = 0.0
        if abs(erreur_x) < 50:  # Le tag est au centre (+/- 50 pixels)
            if distance_avant > DISTANCE_MIN:
                vx = 0.3  # Avancer à 0.3 m/s
            else:
                vx = 0.0  # Stop, on est devant la fenêtre !

        # Envoi de la commande au Pixhawk
        # vx (avant), vy (droite = 0), vz (bas = 0), yaw_rate
        drone.send_velocity_body(vx, 0, 0, yaw_rate)

    else:
        # L'Aruco est perdu : on s'arrête de bouger en XY, maintien altitude
        drone.send_velocity_body(0, 0, 0, 0)