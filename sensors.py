# Bibliothèques fictives ou génériques pour l'exemple
# Utilise gpiozero ou RPi.GPIO pour les servos, et la lib pimoroni pour le ToF

class HardwareManager:
    def __init__(self):
        print("Initialisation du capteur VL53L5CX et des servos...")
        # TODO: init I2C pour VL53L5CX
        # TODO: init PWM pour crochet et caméra

    def get_forward_distance(self):
        """ Retourne la distance vers l'avant en mètres """
        # TODO: Lire le capteur Pimoroni
        return 2.5  # Mock: retourne 2.5 mètres par défaut

    def set_camera_pitch(self, angle):
        """ Angle: 0 (Avant) à -90 (Sol) """
        print(f"Caméra pivotée à {angle}°")
        # TODO: PWM Servo caméra

    def set_hook(self, open_state):
        print(f"Crochet ouvert : {open_state}")
        # TODO: PWM Servo crochet