import time
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

        self.last_time = time.time()
        self.sum_err_x = 0.0
        self.sum_err_y = 0.0

        # --- Paramètres Caméra (À AJUSTER selon ton matériel) ---
        self.FOV_X_DEG = 70.0 # Angle de vue horizontal de ta caméra (ex: 70°)
        self.FOV_Y_DEG = 55.0 # Angle de vue vertical de ta caméra (ex: 55°)

    def compensate_camera_angles(self, raw_x, raw_y, width, height, roll_rad, pitch_rad, cam_is_down=True):
        """
        Corrige la position (x, y) de la cible en fonction de l'inclinaison du drone.
        Retourne (corrected_x, corrected_y).
        """
        # 1. Calcul du ratio : Combien de pixels représente 1 radian d'inclinaison ?
        px_per_rad_x = width / math.radians(self.FOV_X_DEG)
        px_per_rad_y = height / math.radians(self.FOV_Y_DEG)
        
        if cam_is_down:
            # Caméra regarde le sol
            
            offset_x = roll_rad * px_per_rad_x
            offset_y = pitch_rad * px_per_rad_y
            
            corrected_x = raw_x + offset_x
            corrected_y = raw_y - offset_y # Le signe dépend du repère MAVLink vs OpenCV
            
            return corrected_x, corrected_y
        
        else:
            # Caméra vers l'avant (Plus complexe, gère le lacet)
            # ... (À implémenter compléter !!!!!!!!!!!!!!!!!!!!!!!!!)
            pass

    def run(self, drone, hw, arucos_data, frame_width, frame_height):
        TARGET_ID = 0
        ALTITUDE_CIBLE = 0.25 # 50 cm

        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        # S'assurer qu'on a les derniers angles du drone
        drone.update_attitude()
        
        # ---------------------------------------------------------
        # ETAT 1 : DÉCOLLAGE
        # ---------------------------------------------------------
        """
        if self.state == "TAKEOFF":
            if not self.takeoff_done:
                print(f"Décollage à {ALTITUDE_CIBLE}m...")
                #drone.arm_and_takeoff(ALTITUDE_CIBLE)
                drone.arm_and_takeoff_guided(ALTITUDE_CIBLE)
                self.takeoff_done = True
                self.state = "SEARCH"
        """
        if self.state == "TAKEOFF":
            print(f"Décollage demandé à {ALTITUDE_CIBLE}m (Absolu Lidar)...")
            # On lance l'action une seule fois
            if drone.arm_and_takeoff_guided2(ALTITUDE_CIBLE):
                self.state = "TAKEOFF_MONITORING"
                
        elif self.state == "TAKEOFF_MONITORING":
            # Cette zone est lue en boucle (à chaque frame caméra) sans bloquer le script
            current_alt = drone.get_current_alt_brute() # Via RANGEFINDER
            print(f"\r[TAKEOFF] Lidar: {current_alt:.2f}m | Cible: {ALTITUDE_CIBLE:.2f}m", end="", flush=True)

            # Vérification avec tolérance
            if current_alt >= (ALTITUDE_CIBLE - 0.05):
                print("Altitude atteinte ! Passage à la recherche.")
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
                raw_center_x, raw_center_y = arucos_data[TARGET_ID]
                
                # --- COMPENSATION MAGIQUE ICI ---
                # On corrige la position lue par la caméra avec les angles inertiels
                cam_x, cam_y = self.compensate_camera_angles(
                    raw_center_x, raw_center_y, 
                    frame_width, frame_height, 
                    drone.roll, drone.pitch, 
                    cam_is_down=True
                )
                
                # Le reste du code PI utilise maintenant l'erreur CORRIGÉE
                err_x_img = cam_x - (frame_width / 2)
                err_y_img = cam_y - (frame_height / 2)
                
                self.sum_err_x += err_x_img * dt
                self.sum_err_y += err_y_img * dt
                max_integral = 1000
                self.sum_err_x = max(min(self.sum_err_x, max_integral), -max_integral)
                self.sum_err_y = max(min(self.sum_err_y, max_integral), -max_integral)

                Kp = 0.002
                Ki = 0.0005 
                
                cmd_x = (err_x_img * Kp) + (self.sum_err_x * Ki)
                cmd_y = (err_y_img * Kp) + (self.sum_err_y * Ki)
                
                vx = -cmd_y 
                vy = cmd_x  
                
                vx = max(min(vx, 0.2), -0.2)
                vy = max(min(vy, 0.2), -0.2)
                
                # Pour valider le centrage, on regarde l'erreur stabilisée
                if abs(err_x_img) < 30 and abs(err_y_img) < 30:
                    print("Cible verrouillée au centre. Atterrissage !")
                    drone.send_velocity_body(0, 0, 0, 0)
                    drone.land()
                    self.state = "LANDED"
                else:
                    drone.send_velocity_body(vx, vy, 0, 0)
            else:
                # Si on perd l'ArUco de vue, on s'arrête et on repasse en recherche
                print("ArUco perdu ! Arrêt et reprise de la recherche.")
                self.sum_err_x = 0
                self.sum_err_y = 0
                drone.send_velocity_body(0, 0, 0, 0)
                self.state = "SEARCH"
                
        # ---------------------------------------------------------
        # ETAT 4 : POSÉ
        # ---------------------------------------------------------
        elif self.state == "LANDED":
            # Ne fait plus rien
            pass