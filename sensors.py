# Bibliothèques fictives ou génériques pour l'exemple
# Utilise gpiozero ou RPi.GPIO pour les servos, et la lib pimoroni pour le ToF
import time
import numpy as np
#import vl53l5cx_ctypes as vl53l5cx
#from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE, RANGING_MODE_CONTINUOUS

class VL53L5CXReader:
    """
    Thread dédié à la lecture continue du VL53L5CX en I2C sur Raspberry Pi.
    Expose la dernière distance mesurée sans jamais bloquer l'appelant.
    """

    def __init__(self):
        self._distance_mm: float | None = None

        # Init capteur
        print("Uploading firmware, please wait...")
        self._sensor = vl53l5cx.VL53L5CX()
        print("Done!")
        self._sensor.set_resolution(8 * 8)
   
   ####
        self._sensor.set_ranging_frequency_hz(15) # 15 Hz
        self._sensor.set_ranging_mode(RANGING_MODE_CONTINUOUS)
####
        self._sensor.start_ranging()


    def get_distance(self) -> float | None:
        """
        Retourne la dernière distance estimée en mm.
        Non bloquant, instantané.
        """
        with self._lock:
            return self._distance_mm

    def _run(self):
        data = self._sensor.get_data()

        distances = np.array(data.distance_mm, dtype=float)
        distances = distances[distances > 0]  # éliminer les mesures nulles/invalides

        # Médiane des 64 zones → robuste au bruit et aux fausses détections
        median_mm = float(np.median(distances))


        self._distance_mm = median_mm

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