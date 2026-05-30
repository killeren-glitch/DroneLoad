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

import math

class Epreuve1Task:
    def __init__(self):
        # Nouveaux états pour le test : "TAKEOFF", "TAKEOFF_MONITORING", "WAIT", "LANDED"
        self.state = "TAKEOFF"
        self.takeoff_done = False

        self.last_time = time.time()
        self.sum_err_x = 0.0
        self.sum_err_y = 0.0
        
        # Variable pour chronométrer l'attente
        self.hover_start_time = 0.0

        # --- Paramètres Caméra ---
        self.FOV_X_DEG = 70.0 
        self.FOV_Y_DEG = 55.0 

    def compensate_camera_angles(self, raw_x, raw_y, width, height, roll_rad, pitch_rad, cam_is_down=True):
        """ Code conservé pour plus tard """
        px_per_rad_x = width / math.radians(self.FOV_X_DEG)
        px_per_rad_y = height / math.radians(self.FOV_Y_DEG)
        
        if cam_is_down:
            offset_x = roll_rad * px_per_rad_x
            offset_y = pitch_rad * px_per_rad_y
            corrected_x = raw_x + offset_x
            corrected_y = raw_y - offset_y 
            return corrected_x, corrected_y
        else:
            pass

    def run(self, drone, hw, arucos_data, frame_width, frame_height):
        ALTITUDE_CIBLE = 0.25 # 25 cm

        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        # S'assurer qu'on a les derniers angles du drone
        drone.update_attitude()
        
        # ---------------------------------------------------------
        # ETAT 1 : DÉCOLLAGE
        # ---------------------------------------------------------
        if self.state == "TAKEOFF":
            print(f"Décollage demandé à {ALTITUDE_CIBLE}m (Absolu Lidar)...")
            # On lance l'action une seule fois
            if drone.arm_and_takeoff_guided2(ALTITUDE_CIBLE):
                self.state = "TAKEOFF_MONITORING"
                
        elif self.state == "TAKEOFF_MONITORING":
            # Cette zone est lue en boucle sans bloquer le script
            current_alt = drone.get_current_alt_brute() # Via RANGEFINDER
            print(f"\r[TAKEOFF] Lidar: {current_alt:.2f}m | Cible: {ALTITUDE_CIBLE:.2f}m", end="", flush=True)

            # Vérification avec tolérance
            if current_alt >= (ALTITUDE_CIBLE - 0.05):
                print("\n[OK] Altitude atteinte ! Début du stationnaire de 10 secondes.")
                self.hover_start_time = time.time() # On lance le chrono !
                self.state = "WAIT"

        # ---------------------------------------------------------
        # ETAT 2 : ATTENTE 10 SECONDES (Test)
        # ---------------------------------------------------------
        elif self.state == "WAIT":
            # On envoie une commande de vitesse nulle pour le forcer à tenir sa position
            drone.send_velocity_body(0, 0, 0, 0)
            
            elapsed_time = time.time() - self.hover_start_time
            print(f"\r[WAIT] Attente... {elapsed_time:.1f} / 10.0 s", end="", flush=True)
            
            if elapsed_time >= 10.0:
                print("\n[OK] Temps écoulé. Atterrissage en cours !")
                drone.land()
                self.state = "LANDED"

        # ---------------------------------------------------------
        # ETAT 3 : POSÉ
        # ---------------------------------------------------------
        elif self.state == "LANDED":
            # Ne fait plus rien
            pass
            
        # (J'ai masqué SEARCH et CENTER pour ce test, tu pourras les remettre ensuite)