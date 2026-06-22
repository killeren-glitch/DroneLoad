import init_function as fa
import capteurs as capt
import time
import epreuve1_function as f
import mavlink
import sensors as s
import gstream

# cap = fa.camera_init()
detector, aruco_dict = fa.aruco_init()
mavlink_command = mavlink.DroneController()
sensors = s.HardwareManager()
video = gstream.VideoManager()
cap = video


def attendre_manoeuvre_manuelle_en_cour():
    pass


def reconnaitre_zone_livraison(zone_annoncee: str):
    """
    Reconnaît la zone de livraison correspondant à celle annoncée par l'arbitre.

    zone_annoncee : numéro/nom de la zone annoncée (ex: "1", "2", "3", "4")

    Retourne True si la zone reconnue correspond, False sinon.
    """
    zone_detectee, confiance = f.classify_delivery_zone(cap)

    if zone_detectee is None:
        print("Aucune zone reconnue — passage en mode manuel")
        return False

    if zone_detectee != zone_annoncee:
        print(f"Zone détectée ({zone_detectee}) ≠ zone annoncée ({zone_annoncee})")
        return False

    print(f"Zone {zone_detectee} confirmée (confiance: {confiance:.2f})")
    return True


def deposer_objet():
    """
    Dépose l'objet sur la zone actuellement survolée.
    """
    deposed = f.drop_object(mavlink_command)
    if not deposed:
        print("Échec dépôt objet, passage en mode manuel")
    return deposed


def recuperer_objet():
    """
    Récupère un objet dans l'aire de stockage (zone B).
    """
    recovered = f.pick_up_object(cap, detector, mavlink_command)
    if not recovered:
        print("Échec récupération objet, passage en mode manuel")
    return recovered


def aligner_robot_sur_aruco(aruco_id: int):
    """
    Guide le robot via UART pour l'aligner devant l'ArUco demandé.

    aruco_id : ID de l'ArUco cible (2 ou 3)

    Retourne True si le robot confirme l'alignement (OUI reçu), False sinon.
    """
    aligned = f.guide_robot_to_aruco(cap, detector, mavlink_command, aruco_id)
    if not aligned:
        print(f"Échec alignement robot sur ArUco {aruco_id}, passage en mode manuel")
    return aligned


def lire_zone_depuis_robot():
    """
    Lit la zone de dépôt transmise par le robot via UART.

    Retourne le numéro de zone (str) ou None en cas d'échec.
    """
    zone = f.read_zone_from_robot(mavlink_command)
    if zone is None:
        print("Aucune zone reçue du robot, passage en mode manuel")
    return zone


def avancer_pendant(temps):
    yaw = 0
    vx = 0.1
    mavlink_command.send_velocity_body(vx, 0.0, 0.0, yaw)
    time.sleep(temps)


def decollage():
    vx = 0
    vy = 0
    vz = -0.1
    yaw = 0
    while True:
        mavlink_command.send_velocity_body(vx, vy, vz, yaw)
        altitude = sensors.get_altitude()
        if altitude >= 1:
            break
        else:
            time.sleep(0.05)

    # Stabilisation 5 secondes
    mavlink_command.keep_position(5, yaw)


def atterissage():
    mavlink_command.land()


def demi_tour() -> None:
    mavlink_command.rotate_drone(180)

def run_epreuve1(drone, hw, arucos_data, frame_width):

        """
        ---------- PHASE 1 : DÉCOLLAGE + DÉPÔT INITIAL ---------
                        Zone A -> Zone de livraison annoncée
        """
        # Décollage avec objet attaché
        decollage()

        # passage auto à manuel
        # l'opérateur positionne le drone face à la zone annoncée par l'arbitre
        attendre_manoeuvre_manuelle_en_cour()

        # on repasse en automatique : reconnaissance + dépose
        zone_annoncee = f.get_announced_zone()  # récupérée via interface arbitre/UI

        if reconnaitre_zone_livraison(zone_annoncee):
            deposer_objet()
        else:
            attendre_manoeuvre_manuelle_en_cour()
            deposer_objet()

        """
        ---------- PHASE 2 : RETOUR STOCKAGE + RÉCUPÉRATION OBJET ---------
                        Zone de livraison -> Zone B (stockage)
        """
        avancer_pendant(0.05)

        # passage auto à manuel vers la zone B
        attendre_manoeuvre_manuelle_en_cour()

        # on repasse en automatique
        recuperer_objet()

        """
        ---------- PHASE 3 : COOPÉRATION AVEC LE ROBOT PULL-E ---------
                        Guidage robot via ArUcos 2 et 3
        """
        avancer_pendant(0.05)

        # passage auto à manuel : positionner le drone au-dessus du robot (ArUco 1)
        attendre_manoeuvre_manuelle_en_cour()

        # on repasse en automatique : alignement sur le premier ArUco
        if aligner_robot_sur_aruco(2):
            pass
        else:
            attendre_manoeuvre_manuelle_en_cour()
            aligner_robot_sur_aruco(2)

        # alignement sur le second ArUco
        if aligner_robot_sur_aruco(3):
            pass
        else:
            attendre_manoeuvre_manuelle_en_cour()
            aligner_robot_sur_aruco(3)

        # lecture de la zone transmise par le robot
        zone_robot = lire_zone_depuis_robot()
        if zone_robot is None:
            attendre_manoeuvre_manuelle_en_cour()
            zone_robot = lire_zone_depuis_robot()

        """
        ---------- PHASE 4 : DÉPÔT FINAL SUR ZONE INDIQUÉE PAR LE ROBOT ---------
        """
        avancer_pendant(0.05)

        # passage auto à manuel vers la zone indiquée par le robot
        attendre_manoeuvre_manuelle_en_cour()

        # on repasse en automatique : reconnaissance + dépose finale
        if reconnaitre_zone_livraison(zone_robot):
            deposer_objet()
        else:
            attendre_manoeuvre_manuelle_en_cour()
            deposer_objet()

        """
        ---------- PHASE 5 : RETOUR ET ATTERRISSAGE ---------
                        Retour vers la zone A
        """
        avancer_pendant(0.05)

        demi_tour()

        avancer_pendant(0.05)

        # passage auto à manuel pour le retour précis sur ArUco 0
        attendre_manoeuvre_manuelle_en_cour()

        # atterrissage sur la zone A
        atterissage()


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