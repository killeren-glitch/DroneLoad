# Bibliothèques fictives ou génériques pour l'exemple
# Utilise gpiozero ou RPi.GPIO pour les servos, et la lib pimoroni pour le ToF
import numpy
import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE
import threading
from vl53l5cx import VL53L5CX
from vl53l5cx.api import RANGING_MODE_CONTINUOUS

class HardwareManager:
    def __init__(self):
        print("Initialisation du capteur VL53L5CX et des servos...")
        # TODO: init I2C pour VL53L5CX
        # TODO: init PWM pour crochet et caméra

        #init capteur de flux
        print("Uploading firmware, please wait...")
        self.vl53 = self.vl53l5cx.VL53L5CX()
        print("Done!")
        self.vl53.set_resolution(8 * 8)
        self.vl53.enable_motion_indicator(8 * 8)
        # vl53.set_integration_time_ms(50)
        # Enable motion indication at 8x8 resolution
        self.vl53.enable_motion_indicator(8 * 8)
        # Default motion distance is quite far, set a sensible range
        # eg: 40cm to 1.4m
        self.vl53.set_motion_distance(400, 1400)
        self.vl53.start_ranging()

        #initi capteur de distance avant
        self._distance_mm: float | None = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # Init capteur
        self._sensor = VL53L5CX()
        self._sensor.set_resolution(64)  # mode 8x8
        self._sensor.set_ranging_frequency_hz(15)  # 15 Hz
        self._sensor.set_ranging_mode(RANGING_MODE_CONTINUOUS)
        self._sensor.start_ranging()

        # Lancement du thread
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def get_altitude(self):
        """ Retourne la distance vers l'avant en mètres """
        # TODO: Lire le capteur Pimoroni

        distance = 1.9
        if self.vl53.data_ready():
            data = self.vl53.get_data()
            # 2d array of motion data (always 4x4?)
            motion = numpy.flipud(numpy.array(data.motion_indicator.motion[0:16]).reshape((4, 4)))
            # 2d array of distance
            distance = numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8)))
            # 2d array of reflectance
            reflectance = numpy.flipud(numpy.array(data.reflectance).reshape((8, 8)))
            # 2d array of good ranging data
            status = numpy.isin(numpy.flipud(numpy.array(data.target_status).reshape((8, 8))),
                                (STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE))
            print(motion, distance, reflectance, status)

        return distance  # Mock: retourne 2.5 mètres par défaut

    def set_camera_pitch(self, angle):
        """ Angle: 0 (Avant) à -90 (Sol) """
        print(f"Caméra pivotée à {angle}°")
        # TODO: PWM Servo caméra

    def set_hook(self, open_state):
        print(f"Crochet ouvert : {open_state}")
        # TODO: PWM Servo crochet



    def get_forward_distance(self) -> float | None:
        """
        Retourne la dernière distance estimée en mm.
        Non bloquant, instantané.
        """
        with self._lock:
            return self._distance_mm

    def stop(self):
        """Arrête proprement le thread de lecture."""
        self._stop_event.set()
        self._thread.join()
        self._sensor.stop_ranging()

    def _run(self):
        while not self._stop_event.is_set():

            if not self._sensor.data_ready():
                continue                        # pas de sleep, polling non-bloquant

            data = self._sensor.get_data()
            self._sensor.clear_interrupt()

            distances = numpy.array(data.distance_mm, dtype=float)
            distances = distances[distances > 0]  # éliminer les mesures nulles/invalides

            if distances.size == 0:
                continue

            # Médiane des 64 zones → robuste au bruit et aux fausses détections
            median_mm = float(numpy.median(distances))

            with self._lock:
                self._distance_mm = median_mm