import numpy as np

DX = 0
DY = 0
DZ = 0
SPEED = (0 , 0, 0 )
ACC = (0 , 0, 0)
WINDOW_IDS = {3, 6, 0, 53}

# ====== PARAMÈTRES À ADAPTER ======
MARKER_SIZE = 4.3  # taille réelle du marqueur (cm). tvec sera dans la même unité.

# ⚠️ Ces valeurs sont fictives — effectue une calibration caméra réelle avec cv2.calibrateCamera()
cameraMatrix = np.array([
    [800.0,   0.0, 320.0],
    [  0.0, 800.0, 240.0],
    [  0.0,   0.0,   1.0]
], dtype=np.float64)

distCoeffs = np.array([0.1, -0.05, 0.001, 0.0, 0.0], dtype=np.float64)
# ==================================


DISTANCE_C_TO_E = 1