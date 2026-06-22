from pymavlink import mavutil
import time
import math


class DroneController:
    def __init__(self, connection_string="/dev/ttyAMA0", baudrate=921600):
        # Pour SITL, connection_string sera par ex "udp:127.0.0.1:14550"
        print(f"Connexion au drone sur {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string, baud=baudrate)
        self.master.wait_heartbeat()
        print("Cible connectée !")

    def set_ekf_origin(self):
        """ Injecte une fausse position globale pour débloquer le mode GUIDED sans GPS """
        print("Initialisation de l'origine EKF (Faux GPS)...")
        # Coordonnées arbitraires (Exemple: Paris)
        lat = int(48.8566 * 1e7)
        lon = int(2.3522 * 1e7)
        
        # CORRECTION 1 : Ne jamais mettre 0. (Unité = millimètres)
        alt = 1000 # 1 mètre
        
        # Message 48 : SET_GPS_GLOBAL_ORIGIN
        self.master.mav.set_gps_global_origin_send(
            self.master.target_system,
            lat, lon, alt
        )
        
        # Message 411 : SET_HOME_POSITION
        self.master.mav.set_home_position_send(
            self.master.target_system,
            lat, lon, alt,
            0, 0, 0, 
            [1.0, 0.0, 0.0, 0.0], # Quaternion formaté proprement en floats
            0, 0, 0
        )
        
        # On laisse à l'EKF3 le temps de traiter l'information et de passer au vert
        time.sleep(1.0)

    def arm_and_takeoff(self, target_altitude):
        # 1. Injection de la fausse origine
        self.set_ekf_origin()
        
        # 2. Mode GUIDED
        print("Passage en mode GUIDED...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4)
        time.sleep(0.5)

        # 3. Armement
        print("Armement des moteurs...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

        self.master.motors_armed_wait()
        print("Moteurs armés ! Lancement de la commande TAKEOFF...")

        # 4. Commande de décollage globale
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, target_altitude)
            
        print("Commande acceptée. Surveillance de l'altitude (Spool Up en cours)...")
        
        # 5. BOUCLE DE MAINTIEN (Cruciale avec Pymavlink)
        while True:
            # On garde le script "en vie" en envoyant un Heartbeat à ArduPilot
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                
            # On lit l'altitude locale pour savoir quand on est arrivé
            msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if msg:
                alt = -msg.z  # Inversion de l'axe Z (NED)
                
                # Optionnel : Afficher l'altitude en direct (décommente pour voir la montée)
                # print(f"Montée... Altitude actuelle : {alt:.2f} m")
                
                # Si on a atteint 95% de l'altitude demandée, le décollage est terminé
                if alt >= target_altitude * 0.95:
                    print(f"Altitude cible ({target_altitude}m) atteinte ! Le drone est en vol stationnaire.")
                    break
            
            # On boucle à 10Hz. Rappel : le drone va attendre ~3 secondes avant de commencer à monter.
            time.sleep(0.1)
        
    def arm_and_takeoff_test(self, target_altitude=0.5):
        """ 
        Décollage par commande directe de poussée (Thrust) en GUIDED_NOGPS.
        Ignore totalement la radiocommande.
        """
        # 1. Passage en mode GUIDED_NOGPS (Mode 20)
        print("Passage en mode GUIDED_NOGPS...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            20) 
        time.sleep(1.0)

        print(f"Montée vers {target_altitude}m via Thrust MAVLink...")
        
        while True:
            # --- LECTURE ALTITUDE LIDAR ---
            msg = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=0.2)
            current_alt = 0.0
            
            if msg:
                current_alt = msg.alt # alt est un float directement en mètres
            
            print(f"Altitude EKF : {current_alt:.2f} m")
            
            # --- VÉRIFICATION CIBLE ---
            if current_alt >= target_altitude:
                print(f"Altitude de {target_altitude}m atteinte !")
                break # On sort de la boucle de montée
                
            time.sleep(0.1)


    def arm_and_takeoff_nogps(self, target_altitude=0.5):
        """ 
        Décollage par commande directe de poussée (Thrust) en GUIDED_NOGPS.
        Ignore totalement la radiocommande.
        """
        # 1. Passage en mode GUIDED_NOGPS (Mode 20)
        print("Passage en mode GUIDED_NOGPS...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            20) 
        time.sleep(1.0)

        # 2. Armement des moteurs
        print("Armement des moteurs...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("Moteurs Armés !")

        print(f"Montée vers {target_altitude}m via Thrust MAVLink...")
        
        while True:
            # --- COMMANDE DE POUSSÉE ---
            # Le thrust va de 0.0 (moteurs coupés) à 1.0 (à fond).
            # En général, un drone flotte (Hover) autour de 0.3 ou 0.4.
            # 0.65 garantit une montée franche. (Baisse à 0.55 s'il monte trop vite).
            thrust_value = 0.55 
            
            # Message MAVLink "SET_ATTITUDE_TARGET"
            """
            self.master.mav.set_attitude_target_send(
                0, # time_boot_ms (ignoré)
                self.master.target_system, self.master.target_component,
                0b00000111,    # Masque: Ignore les vitesses de rotation, n'écoute que l'attitude et le Thrust
                [1, 0, 0, 0],  # Quaternion [w, x, y, z] : [1,0,0,0] = Drone maintenu parfaitement plat
                0, 0, 0,       # Roll rate, pitch rate, yaw rate (ignorés par le masque)
                thrust_value   # Poussée des moteurs
            )
            """

            # --- LECTURE ALTITUDE LIDAR ---
            msg = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=0.2)
            current_alt = 0.0
            
            if msg:
                current_alt = msg.alt # alt est un float directement en mètres
            
            print(f"Altitude EKF : {current_alt:.2f} m")
            
            # --- VÉRIFICATION CIBLE ---
            if current_alt >= target_altitude:
                print(f"Altitude de {target_altitude}m atteinte !")
                break # On sort de la boucle de montée
                
            time.sleep(0.1)

        # 3. REPASSAGE EN MODE GUIDED
        # Le mode GUIDED (Mode 4) va stabiliser le drone là où il est (Hover)
        # Et il va de nouveau accepter tes commandes de vitesse (send_velocity_body) pour l'Aruco !
        print("Repassage en mode GUIDED (Mode 4)...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4) 
        time.sleep(1.0)
            
        print("Décollage terminé ! Le drone est prêt pour l'IA.")

    def send_velocity_body(self, vx, vy, vz, yaw_rate):
        """
        Envoie des vitesses au drone relatives à lui-même (avant, droite, bas).
        vx: vitesse vers l'avant (m/s)
        vy: vitesse vers la droite (m/s)
        vz: vitesse vers le bas (m/s) - Négatif pour monter !
        yaw_rate: rotation sur lui-même (rad/s)
        """
        # MAV_FRAME_BODY_NED : l'axe X est toujours l'avant du drone
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b01111111000111,  # Masque pour ignorer la position, n'écouter que la vitesse et yaw_rate
            0, 0, 0,  # Positions x, y, z ignorées
            vx, vy, vz,  # Vitesses
            0, 0, 0,  # Accélérations ignorées
            0, yaw_rate)

    def send_velocity_body_with_stop_time(self, vx, vy, vz, yaw_rate, duration: float = 0.0):
        """
        Envoie des vitesses au drone relatives à lui-même (avant, droite, bas).

        vx       : vitesse vers l'avant (m/s)
        vy       : vitesse vers la droite (m/s)
        vz       : vitesse vers le bas (m/s) - Négatif pour monter !
        yaw_rate : rotation sur lui-même (rad/s)
        duration : durée d'envoi en secondes (0 = envoi unique)
        """
        if duration <= 0:
            # Envoi unique — comportement original
            self.master.mav.set_position_target_local_ned_send(
                0, self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b01111111000111,
                0, 0, 0,
                vx, vy, vz,
                0, 0, 0,
                0, yaw_rate)
        else:
            # Envoi pendant duration secondes
            start = time.time()
            while time.time() - start < duration:
                self.master.mav.set_position_target_local_ned_send(
                    0, self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b01111111000111,
                    0, 0, 0,
                    vx, vy, vz,
                    0, 0, 0,
                    0, yaw_rate)
                time.sleep(0.05)  # 20Hz

            # Stop automatique à la fin
            self.master.mav.set_position_target_local_ned_send(
                0, self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b01111111000111,
                0, 0, 0,
                0, 0, 0,
                0, 0, 0,
                0, 0)

    def keep_position(self, wait_time, yaw):
        for i in range(wait_time):
            self.send_velocity_body(0, 0, 0, yaw)
            time.sleep(1)

    def land(self):
        print("Atterrissage commandé !")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0)

    def rotate_drone(self, angle_deg: float,
                     yaw_rate_deg_s: float = 30.0) -> None:
        """
        Fait tourner le drone sur lui-même d'un angle donné.

        master        : connexion pymavlink
        angle_deg     : angle de rotation en degrés
                        positif = sens horaire
                        négatif = sens antihoraire
        yaw_rate_deg_s: vitesse de rotation en degrés/seconde (défaut 30°/s)
        """
        # Récupérer le yaw actuel depuis le Pixhawk
        msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
        if msg is None:
            print("Impossible de lire l'attitude du drone")
            return

        current_yaw_deg = math.degrees(msg.yaw)  # yaw actuel en degrés
        target_yaw_deg = current_yaw_deg + angle_deg

        # Normaliser entre -180 et 180
        target_yaw_deg = (target_yaw_deg + 180) % 360 - 180
        target_yaw_rad = math.radians(target_yaw_deg)

        # Durée estimée de la rotation
        duration = abs(angle_deg) / yaw_rate_deg_s

        # Envoyer la commande de yaw
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000010111111111,  # uniquement yaw actif
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            target_yaw_rad,  # yaw cible en radians
            0  # yaw_rate
        )

        # Attendre la fin de la rotation
        time.sleep(duration + 0.5)  # +0.5s marge


# Mode sans pixhawk
class DummyMaster:
    """ Imite l'objet master de pymavlink pour que MQTT ne plante pas """
    def arducopter_arm(self):
        print("[MOCK] Commande d'armement reçue")
        
    def arducopter_disarm(self):
        print("[MOCK] Commande de désarmement reçue")
        
    def recv_match(self, type, blocking=False):
        # On simule qu'on ne reçoit aucune info batterie
        return None

class DummyDroneController:
    """ Faux drone pour tester la vision sans matériel """
    def __init__(self):
        print("[MOCK] Faux Pixhawk initialisé (Mode Vision-Only)")
        self.master = DummyMaster()

    def arm_and_takeoff(self, target_altitude):
        print(f"[MOCK] Décollage virtuel à {target_altitude}m")

    def send_velocity_body(self, vx, vy, vz, yaw_rate):
        # Optionnel : Tu peux décommenter le print ci-dessous pour voir 
        # dans le terminal quelles décisions de vol l'IA prend.
        # Mais ça risque de spammer ton terminal à 15 FPS !
        # print(f"[MOCK] Vol -> vx:{vx:.2f}, vy:{vy:.2f}, vz:{vz:.2f}, yaw:{yaw_rate:.3f}")
        pass