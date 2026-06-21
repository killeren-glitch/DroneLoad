"""
def run_epreuve1(drone, hw, arucos_data, frame_width):

#S'exécute à chaque tour de boucle (tick).
#aruco_data : dictionnaire {id_aruco: (x, y)}

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
"""

class Epreuve1Task:
    def __init__(self):
        # États possibles : "TAKEOFF", "SEARCH", "CENTER", "LANDED"
        self.state = "TAKEOFF"
        self.takeoff_done = False

    def run(self, drone, hw, arucos_data, frame_width, frame_height):
        TARGET_ID = 0
        ALTITUDE_CIBLE = 0.5 # 50 cm
        
        # ---------------------------------------------------------
        # ETAT 1 : DÉCOLLAGE
        # ---------------------------------------------------------
        if self.state == "TAKEOFF":
            if not self.takeoff_done:
                print(f"Décollage à {ALTITUDE_CIBLE}m...")
                #drone.arm_and_takeoff(ALTITUDE_CIBLE)
                drone.arm_and_takeoff(ALTITUDE_CIBLE)
                self.takeoff_done = True
                self.state = "SEARCH"
                
        # ---------------------------------------------------------
        # ETAT 2 : RECHERCHE (Avancer doucement)
        # ---------------------------------------------------------
        elif self.state == "SEARCH":
            if TARGET_ID in arucos_data:
                print("ArUco détecté ! Passage en mode CENTRAGE.")
                self.state = "CENTER"
            else:
                # On avance doucement (0.15 m/s) à la recherche de l'Aruco
                # vx=0.15, vy=0, vz=0, yaw_rate=0
                drone.send_velocity_body(0.15, 0, 0, 0)
                
        # ---------------------------------------------------------
        # ETAT 3 : CENTRAGE
        # ---------------------------------------------------------
        elif self.state == "CENTER":
            if TARGET_ID in arucos_data:
                center_x, center_y = arucos_data[TARGET_ID]
                
                # Supposons que le haut de l'image (y=0) correspond à l'avant du drone
                err_x_img = center_x - (frame_width / 2)   # Positif si Aruco à droite
                err_y_img = center_y - (frame_height / 2)  # Positif si Aruco en bas
                
                # Si Aruco est en bas de l'image (err_y positif), le drone a "dépassé" la cible
                # Il faut donc reculer (Vx négatif). 
                # Si Aruco à droite de l'image (err_x positif), il faut translater à droite (Vy positif).
                k_p = 0.002
                vx = -err_y_img * k_p
                vy = err_x_img * k_p
                
                # On s'assure de ne pas aller trop vite pendant le centrage
                vx = max(min(vx, 0.2), -0.2)
                vy = max(min(vy, 0.2), -0.2)
                
                # Condition de validation : Si l'Aruco est au centre à +/- 30 pixels
                if abs(err_x_img) < 30 and abs(err_y_img) < 30:
                    print("Cible verrouillée au centre. Atterrissage !")
                    drone.send_velocity_body(0, 0, 0, 0) # On stoppe les moteurs horizontaux
                    drone.land()
                    self.state = "LANDED"
                else:
                    drone.send_velocity_body(vx, vy, 0, 0)
            else:
                # Si on perd l'ArUco de vue, on s'arrête et on repasse en recherche
                print("ArUco perdu ! Arrêt et reprise de la recherche.")
                drone.send_velocity_body(0, 0, 0, 0)
                self.state = "SEARCH"
                
        # ---------------------------------------------------------
        # ETAT 4 : POSÉ
        # ---------------------------------------------------------
        elif self.state == "LANDED":
            # Ne fait plus rien
            pass