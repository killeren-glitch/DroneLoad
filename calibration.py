import time

import cv2
import numpy as np
import glob
import os

def calibrate_camera(
        images_folder: str,
        chessboard_size: tuple = (9, 6),
        square_size_mm: float = 25.0
) -> np.ndarray | None:
    """
    Calibre la caméra à partir de photos d'un damier et retourne la matrice intrinsèque.

    images_folder  : chemin du dossier contenant les photos .jpg/.png
    chessboard_size: nombre de coins intérieurs (colonnes, lignes) du damier
                     ex: damier 10x7 cases → (9, 6)
    square_size_mm : taille d'une case en mm (défaut 25mm)

    Retourne la cameraMatrix (3x3) ou None si calibration impossible.
    """

    # Préparer les points 3D du damier (z=0 → plan plat)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[
        0:chessboard_size[0],
        0:chessboard_size[1]
    ].T.reshape(-1, 2)
    objp *= square_size_mm

    obj_points = []  # points 3D réels
    img_points = []  # points 2D dans l'image

    # Charger toutes les images du dossier
    images = glob.glob(os.path.join(images_folder, "*.jpg"))
    images += glob.glob(os.path.join(images_folder, "*.png"))

    if len(images) == 0:
        print(f"Aucune image trouvée dans : {images_folder}")
        return None

    print(f"{len(images)} images trouvées — recherche du damier...")

    img_shape = None
    success_count = 0

    for path in images:
        img  = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]  # (width, height)

        # Détecter les coins du damier
        found, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if found:
            # Affiner la position des coins au sous-pixel
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            obj_points.append(objp)
            img_points.append(corners_refined)
            success_count += 1
            print(f"  ✅ Damier détecté : {os.path.basename(path)}")
        else:
            print(f"  ❌ Damier non détecté : {os.path.basename(path)}")

    print(f"\n{success_count}/{len(images)} images utilisées pour la calibration")

    if success_count < 10:
        print("⚠️ Pas assez d'images valides (minimum 10 recommandé)")
        return None

    # Calibration
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_shape, None, None
    )

    print(f"\nErreur de reprojection : {ret:.4f} px")
    if ret > 1.0:
        print("⚠️ Erreur élevée — reprendre les photos avec le damier bien visible")

    print("\ncameraMatrix :")
    print(camera_matrix)

    return camera_matrix

def take_photos(cap: cv2.VideoCapture,
                folder_name: str = "photos",
                n_photos: int = 80,
                delay: float = 0.1) -> str:
    """
    Prend n_photos photos et les enregistre dans un dossier créé automatiquement.

    cap         : VideoCapture OpenCV déjà initialisé
    folder_name : nom du dossier de destination
    n_photos    : nombre de photos à prendre (défaut 80)
    delay       : délai entre chaque photo en secondes (défaut 0.1s)

    Retourne le chemin absolu du dossier créé.
    """

    # Créer le dossier s'il n'existe pas
    os.makedirs(folder_name, exist_ok=True)
    print(f"Dossier créé : {os.path.abspath(folder_name)}")

    count = 0
    while count < n_photos:

        ret, frame = cap.read()
        if not ret:
            print(f"Erreur lecture frame {count + 1} — on réessaie")
            time.sleep(0.05)
            continue

        # Nom du fichier : photo_001.jpg, photo_002.jpg ...
        filename = os.path.join(folder_name, f"photo_{count + 1:03d}.jpg")
        cv2.imwrite(filename, frame)

        print(f"[{count + 1}/{n_photos}] Sauvegardée : {filename}")
        count += 1

        time.sleep(delay)

    print(f"\n{n_photos} photos enregistrées dans : {os.path.abspath(folder_name)}")

    return os.path.abspath(folder_name)

cap = cv2.VideoCapture(0)


# Avec un dossier personnalisé et délai plus long
take_photos(cap, folder_name="photos", n_photos=80, delay=1)

cap.release()


input(" hit to enter ")

# Calibrer avec les 80 photos prises
camera_matrix = calibrate_camera(
    images_folder="photos",
    chessboard_size=(9, 6),   # coins intérieurs du damier
    square_size_mm=25.0       # taille d'une case en mm
)

# Utiliser la matrice dans ton code ArUco
if camera_matrix is not None:
    print("Calibration OK !")
    print(camera_matrix)