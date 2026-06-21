
import cv2
import cv2.aruco as aruco
import head as h



def camera_init() -> cv2.VideoCapture:
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Impossible d'ouvrir la caméra (VideoCapture(0)).")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    return cap


def aruco_init() -> tuple:
    """
    Initialise le dictionnaire ArUco et le détecteur une seule fois.
    Retourne (detector, aruco_dict) pour réutilisation dans la boucle.
    """
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    return detector, aruco_dict


def reference_rectifier(x: float, y: float, z: float) -> tuple:
    """Convertit les coordonnées caméra en coordonnées drone."""
    return x + h.DX, y + h.DY, z + h.DZ



