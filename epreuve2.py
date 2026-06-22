import init_function as fa
import capteurs as capt
import time
import epreuve2_function as f
import mavlink
import sensors as s
import gstream

#cap = fa.camera_init()
detector, aruco_dict = fa.aruco_init()
mavlink_command = mavlink.DroneController()
sensors = s.HardwareManager()
video = gstream.VideoManager()
cap = video

def attendre_manoeuvre_manuelle_en_cour():
    pass

def passer_a_travers():

    went_through = f.go_through_window(cap, detector, mavlink_command)
    if not went_through:
        print("Échec traversée fenêtre B, Passage en mode manuel")




def avancer_pendant(temps):
    yaw = 0
    vx = 0.1
    mavlink_command.send_velocity_body(vx, 0.0, 0.0, yaw)
    time.sleep(temps)  # ← laisser le temps à la boucle de tourner

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





def run_epreuve2():
    """
    ---------- PHASE 1 : PASS THROUGH THE FIRST WINDOW ---------
                            WINDOW B
    """
    # Décollage à 1m et attendre d'arriver
    decollage()

    #passage auto a manuel
    #le drone est dirigé manuellement vers la fenetre
    attendre_manoeuvre_manuelle_en_cour()

    #on repasse en automatique et on essaye de traverssé la fenetre.
    # Recherche fenêtre B
    passer_a_travers()
    #Si on arrive la ca veut dire que le drone est passé a travers la fenetre
    #et du coup ce qu'on fait c'est d'avancé un pe ensuite on s'arréte



    """
    ---------- PHASE 2 : PASS THROUGH THE SECOND WINDOW ---------
                            WINDOW C
    """
    # Avancer
    avancer_pendant(0.05)#seconde

    #passage auto a manuel
    #le drone est dirigé manuellement vers la fenetre C
    attendre_manoeuvre_manuelle_en_cour()

    #on repasse en automatique et on essaye de traverssé la fenetre.
    # Traversée
    passer_a_travers()

    #on a traversé la fenetre C
    # Avancer vers la zone E
    avancer_pendant(0.05)

    # Stabilisation 5 secondes
    f.keep_position(5, mavlink_command)

    #On fait demi tour
    demi_tour()

    """
        ---------- PHASE 3 : PASS THROUGH THE SECOND WINDOW ---------
                                WINDOW C-retour vers la zone de départ: zone A
        """
    # Avancer
    avancer_pendant(0.05)  # seconde

    # passage auto a manuel
    # le drone est dirigé manuellement vers la fenetre C
    attendre_manoeuvre_manuelle_en_cour()

    # on repasse en automatique et on essaye de traverssé la fenetre.
    # Traversée
    passer_a_travers()


    """
    ---------- PHASE 4 : PASS THROUGH THE FIRST WINDOW ---------
                            WINDOW B-retour vers la zone de départ: zone A
    """

    # on a traversé la fenetre C
    # Avancer vers la zone E
    avancer_pendant(0.05)

    #passage auto a manuel
    #le drone est dirigé manuellement vers la fenetre
    attendre_manoeuvre_manuelle_en_cour()

    #on repasse en automatique et on essaye de traverssé la fenetre.
    # Recherche fenêtre B
    passer_a_travers()
    #Si on arrive la ca veut dire que le drone est passé a travers la fenetre
    #et du coup ce qu'on fait c'est d'avancé un pe ensuite on s'arréte

    # on a traversé la fenetre C
    # Avancer vers la zone E
    avancer_pendant(0.05)

    #On fait demi tour
    demi_tour()

    # attérisage sur la zone A
    atterissage()









