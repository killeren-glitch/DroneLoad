import time
import cv2
import cv2.aruco as aruco
import numpy as np
import head as h
import det_centers as det

class PID:
    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = 0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = time.time()

    def compute(self, error: float) -> float:
        now = time.time()
        dt  = now - self._prev_time
        if dt <= 0:
            dt = 1e-6

        # Proportionnel
        P = self.kp * error

        # Intégral
        self._integral += error * dt
        I = self.ki * self._integral

        # Dérivé
        D = self.kd * (error - self._prev_error) / dt

        self._prev_error = error
        self._prev_time  = now

        output = P + I + D

        # Limiter la sortie
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = time.time()


# Initialisation des PID (valeurs à calibrer !)
pid_x = PID(kp=0.003, ki=0.0001, kd=0.001, output_limit=0.4)
pid_y = PID(kp=0.003, ki=0.0001, kd=0.001, output_limit=0.4)
pid_z = PID(kp=0.005, ki=0.0001, kd=0.002, output_limit=0.3)




def win_center_coordinates(cap: cv2.VideoCapture, detector: aruco.ArucoDetector) -> tuple | None:
    """
    Capture une frame et retourne les coordonnées (x, y, z) du centre de la fenêtre,
    ou None si la détection échoue ou si 'q' est pressé.
    """

    def put_text(frame: np.ndarray, text: str, org: tuple) -> None:
        cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

    #ret, frame = cap.read()
    frame = cap.get_frame()

    if not frame:
        print("Erreur : impossible de lire la frame.")
        return None

    frame = cap.get_frame()
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(frame_gray)

    result = None

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, h.MARKER_SIZE, h.cameraMatrix, h.distCoeffs
        )

        if len(ids) == 4:
            center   = det.window_center_from_known_markers(ids, tvecs)
            win_center = det.get_window_center_px(list(corners), ids)

            if center is not None and win_center is not None:
                x, y, z = center
                cx, cy  = win_center
                put_text(frame, f"Window center: x={x:.2f} y={y:.2f} z={z:.2f}", (20, 30))
                cv2.circle(frame, (int(cx), int(cy)), 10, (255, 0, 0), -1)
                result = (x, y, z)

        elif len(ids) == 2:
            center2    = det.window_center_from_two_vertical_markers(ids, tvecs, {3, 6}, {0, 58})
            win_center2 = det.get_window_center_px_two_markers(list(corners), ids)

            if center2 is not None and win_center2 is not None:
                x, y, z = center2
                cx, cy  = win_center2
                put_text(frame, f"Window center: x={x:.2f} y={y:.2f} z={z:.2f}", (20, 30))
                cv2.circle(frame, (int(cx), int(cy)), 10, (255, 0, 0), -1)
                result = (x, y, z)

    # Un seul imshow/waitKey à la fin, après tous les dessins
    cv2.imshow("Detection ArUco", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        return None

    return result


def go_through_window(cap, detector, mav_commands) -> bool:
    """
    Centre le drone sur la fenêtre et avance dedans.
    Retourne True quand la fenêtre est traversée.
    """

    while True:
        result = win_center_coordinates(cap, detector)

        if result is None or result == "quit":
            # Fenêtre perdue → arrêt
            mav_commands.set_velocity(0, 0, 0)
            return False

        x_3d, y_3d, z_3d = result  # coordonnées 3D en cm

        # Erreurs
        error_x = x_3d   # décalage latéral en cm
        error_y = y_3d   # décalage vertical en cm
        error_z = z_3d   # distance à la fenêtre en cm

        # Calcul PID
        vx = pid_x.compute(error_x)  # vitesse latérale
        vy = pid_y.compute(error_y)  # vitesse verticale
        vz = pid_z.compute(error_z)  # vitesse vers l'avant

        yaw = 0.0
        mav_commands.send_velocity_body(vx, vy, vz, yaw)

        # Fenêtre traversée quand Z < seuil
        if z_3d < 20:
            mav_commands.send_velocity_body( 0, 0, 0 , yaw)
            pid_x.reset()
            pid_y.reset()
            pid_z.reset()
            return True

        time.sleep(0.05)





