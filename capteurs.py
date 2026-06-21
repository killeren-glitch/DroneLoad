import threading
import numpy as np
from vl53l5cx import VL53L5CX
from vl53l5cx.api import RANGING_MODE_CONTINUOUS


class VL53L5CXReader:
    """
    Thread dédié à la lecture continue du VL53L5CX en I2C sur Raspberry Pi.
    Expose la dernière distance mesurée sans jamais bloquer l'appelant.
    """

    def __init__(self):
        self._distance_mm: float | None = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # Init capteur
        self._sensor = VL53L5CX()
        self._sensor.set_resolution(64)           # mode 8x8
        self._sensor.set_ranging_frequency_hz(15) # 15 Hz
        self._sensor.set_ranging_mode(RANGING_MODE_CONTINUOUS)
        self._sensor.start_ranging()

        # Lancement du thread
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def get_distance(self) -> float | None:
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

            distances = np.array(data.distance_mm, dtype=float)
            distances = distances[distances > 0]  # éliminer les mesures nulles/invalides

            if distances.size == 0:
                continue

            # Médiane des 64 zones → robuste au bruit et aux fausses détections
            median_mm = float(np.median(distances))

            with self._lock:
                self._distance_mm = median_mm


# ----------------------------------------------------------------------
# Utilisation
# ----------------------------------------------------------------------

if __name__ == "__main__":
    reader = VL53L5CXReader()

    try:
        while True:
            distance = reader.get_distance()  # non bloquant

            if distance is not None:
                print(f"Distance : {distance:.0f} mm  ({distance/1000:.2f} m)")
            else:
                print("En attente de la première mesure...")

    except KeyboardInterrupt:
        reader.stop()
        print("Arrêt propre")