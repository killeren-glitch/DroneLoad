import time
import math
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

        self.sum_err_y = 0.0 # Pour le centrage latéral (Axe Y du drone, Axe X de l'image)
        self.sum_err_dist = 0.0

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

    def compensate_camera_forward(self, raw_x, raw_y, width, height, roll_rad, pitch_rad):
        """
        Corrige la position (x, y) pour une caméra regardant vers l'AVANT.
        """
        cx_img = width / 2
        cy_img = height / 2

        # 1. Compensation du PITCH (Le nez se baisse -> L'image monte)
        px_per_rad_y = height / math.radians(self.FOV_Y_DEG)
        # Si pitch > 0 (nez en bas), l'ArUco semble plus haut (Y plus petit)
        # On ajoute l'offset pour redescendre virtuellement l'ArUco à sa vraie place
        offset_y = pitch_rad * px_per_rad_y
        y_pitch_corrected = raw_y + offset_y

        # 2. Compensation du ROLL (Le drone penche -> L'image tourne)
        # On fait une rotation 2D autour du centre de l'image
        dx = raw_x - cx_img
        dy = y_pitch_corrected - cy_img
        
        # Matrice de rotation (On inverse l'angle de Roll)
        cos_r = math.cos(-roll_rad)
        sin_r = math.sin(-roll_rad)
        
        dx_rot = dx * cos_r - dy * sin_r
        dy_rot = dx * sin_r + dy * cos_r
        
        corrected_x = cx_img + dx_rot
        corrected_y = cy_img + dy_rot

        return corrected_x, corrected_y

    def estimate_distance(self, pixel_width, image_width):
        """
        Calcule la distance en mètres grâce au théorème de Thalès.
        """
        REAL_ARUCO_WIDTH_M = 0.20 # 20 cm
        
        # Calcul de la focale en pixels
        focal_length_px = (image_width / 2) / math.tan(math.radians(self.FOV_X_DEG) / 2)
        
        # D = (Taille réelle * Focale) / Taille en pixels
        if pixel_width > 0:
            distance_m = (REAL_ARUCO_WIDTH_M * focal_length_px) / pixel_width
            return distance_m
        return 999.0 # Sécurité si pas de largeur

    def run(self, drone, hw, arucos_data, frame_width, frame_height):
        TARGET_ID = 0
        ALTITUDE_CIBLE = 0.50 # 50 cm
        TARGET_DISTANCE = 0.30 # Arrêt à 30 cm

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
        elif self.state == "APPROACH":
            if TARGET_ID in arucos_data:
                # Récupération des données (On suppose que tu as ajouté la largeur)
                raw_x, raw_y, pixel_width = arucos_data[TARGET_ID]
                
                # 1. Calcul de la Distance
                current_distance = self.estimate_distance(pixel_width, frame_width)
                
                # 2. Compensation des mouvements de la caméra
                cam_x, cam_y = self.compensate_camera_forward(
                    raw_x, raw_y, frame_width, frame_height, 
                    drone.roll, drone.pitch
                )
                
                # 3. Calcul des Erreurs
                err_x_img = cam_x - (frame_width / 2) # Pour le strafe Gauche/Droite
                err_dist = current_distance - TARGET_DISTANCE # Pour l'avance (Positif = trop loin)
                
                # 4. Intégration (PI) avec Anti-Windup
                self.sum_err_y += err_x_img * dt
                self.sum_err_dist += err_dist * dt
                self.sum_err_y = max(min(self.sum_err_y, 500), -500)
                self.sum_err_dist = max(min(self.sum_err_dist, 10), -10)
                
                # 5. Gains PI (À régler en vol)
                # Gain pour le centrage latéral (pixels vers m/s)
                Kp_lat = 0.0015
                Ki_lat = 0.0002
                
                # Gain pour la distance (mètres vers m/s)
                Kp_dist = 0.6
                Ki_dist = 0.1
                
                # 6. Calcul des vitesses
                # L'axe Y du drone (gauche/droite) corrige l'erreur X de l'image
                vy = (err_x_img * Kp_lat) + (self.sum_err_y * Ki_lat)
                
                # L'axe X du drone (avant/arrière) corrige l'erreur de distance
                vx = (err_dist * Kp_dist) + (self.sum_err_dist * Ki_dist)
                
                # Saturation de sécurité (Max 0.3 m/s en approche)
                vx = max(min(vx, 0.3), -0.3)
                vy = max(min(vy, 0.2), -0.2)
                
                print(f"Dist: {current_distance:.2f}m (Err: {err_dist:.2f}m) | Vx: {vx:.2f} Vy: {vy:.2f}")
                
                # 7. Condition d'atterrissage : On est à 30cm (± 5cm) et centré
                if abs(err_dist) < 0.05 and abs(err_x_img) < 40:
                    print("Cible atteinte et verrouillée ! Atterrissage...")
                    drone.send_velocity_body(0, 0, 0, 0)
                    time.sleep(0.5) # Temps de stabilisation
                    drone.land()
                    self.state = "LANDED"
                else:
                    # Envoi des vitesses (vz=0 pour garder l'altitude Lidar gérée par l'EKF)
                    drone.send_velocity_body(vx, vy, 0, 0)
            else:
                print("Cible perdue ! Stationnaire.")
                self.sum_err_y = 0
                self.sum_err_dist = 0
                drone.send_velocity_body(0, 0, 0, 0)
                self.state = "SEARCH"