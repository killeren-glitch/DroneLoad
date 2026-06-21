import numpy as np
import head as h

def window_center_from_known_markers(ids: np.ndarray, tvecs: np.ndarray) -> tuple | None:
    """
    Calcule le centre de la fenêtre à partir des 4 marqueurs connus.
    Retourne (x, y, z) ou None si les 4 marqueurs ne sont pas détectés.
    """
    if ids is None:
        return None

    pts = []
    for i in range(len(ids)):
        marker_id = int(ids[i][0])
        if marker_id in h.WINDOW_IDS:
            x, y, z = tvecs[i].reshape(3)
            pts.append([x, y, z])

    if len(pts) != 4:
        return None

    pts = np.array(pts, dtype=np.float64)
    center = np.mean(pts, axis=0)
    return tuple(center)

def get_window_center_px(corners: list, ids: np.ndarray) -> tuple[int, int] | None:
    """
    Calcule le centre du carré formé par les 4 marqueurs ArUco de la fenêtre.
    corners: liste des corners retournée par detectMarkers
    ids: tableau des ids retourné par detectMarkers
    Retourne (x, y) en pixels ou None si les 4 marqueurs ne sont pas détectés.
    """
    if ids is None:
        return None

    pts = []
    for i in range(len(ids)):
        marker_id = int(ids[i][0])
        if marker_id in h.WINDOW_IDS:
            c = corners[i].reshape(4, 2)
            cx = np.mean(c[:, 0])
            cy = np.mean(c[:, 1])
            pts.append([cx, cy])

    if len(pts) != 4:
        return None

    pts = np.array(pts)
    cx = int(round(np.mean(pts[:, 0])))
    cy = int(round(np.mean(pts[:, 1])))
    return (cx, cy)

WINDOW_SIZE = 1.0  # taille de la fenêtre en mètres (100x100 cm)

def window_center_from_two_vertical_markers(
    ids: np.ndarray,
    tvecs: np.ndarray,
    window_ids_left: set,
    window_ids_right: set
) -> tuple | None:
    """
    Calcule le centre d'une fenêtre C à partir de 2 ArUcos alignés verticalement
    sur l'un des bords (gauche ou droit) de la fenêtre.

    Les ArUcos ne sont pas au centre de la fenêtre → on décale de WINDOW_SIZE/2
    vers l'intérieur pour atteindre le centre réel.

    ids              : tableau des ids retourné par detectMarkers
    tvecs            : tableau des tvecs retourné par estimatePoseSingleMarkers
    window_ids_left  : set des ids ArUco du bord gauche  (ex: {12, 13})
    window_ids_right : set des ids ArUco du bord droit   (ex: {14, 15})

    Retourne (x, y, z) 3D en cm (même unité que MARKER_SIZE) ou None.
    """
    if ids is None:
        return None

    pts_left  = []
    pts_right = []

    for i in range(len(ids)):
        marker_id = int(ids[i][0])
        x, y, z = tvecs[i].reshape(3)

        if marker_id in window_ids_left:
            pts_left.append([x, y, z])
        elif marker_id in window_ids_right:
            pts_right.append([x, y, z])

    # On choisit le bord détecté (au moins 1 ArUco suffit, 2 c'est mieux)
    if len(pts_left) >= 1:
        pts = np.array(pts_left, dtype=np.float64)
        side = "left"
    elif len(pts_right) >= 1:
        pts = np.array(pts_right, dtype=np.float64)
        side = "right"
    else:
        return None

    # Moyenne des ArUcos du bord détecté → centre du bord
    edge_center = np.mean(pts, axis=0)
    x, y, z = edge_center

    # Décalage horizontal vers l'intérieur de la fenêtre (axe X caméra)
    # Bord gauche → décaler vers la droite (+X)
    # Bord droit  → décaler vers la gauche (-X)
    half_window = (WINDOW_SIZE / 2) * 100  # conversion en cm si MARKER_SIZE en cm

    if side == "left":
        x += half_window
    else:
        x -= half_window

    return (x, y, z)

def get_window_center_px_two_markers(
    corners: list,
    ids: np.ndarray
) -> tuple[int, int] | None:
    """
    Calcule le centre en pixels d'une fenêtre carrée à partir de 2 ArUcos
    alignés verticalement sur l'un de ses bords (gauche ou droit).

    La fenêtre étant carrée, la distance entre les 2 ArUcos (hauteur du bord)
    est égale à la largeur de la fenêtre → on décale de cette distance / 2.

    Retourne (x, y) en pixels ou None si aucun ArUco connu n'est détecté.
    """
    if ids is None:
        return None

    pts_left  = []
    pts_right = []

    for i in range(len(ids)):
        marker_id = int(ids[i][0])
        c = corners[i].reshape(4, 2)
        cx = np.mean(c[:, 0])
        cy = np.mean(c[:, 1])


        pts_left.append([cx, cy])

        pts_right.append([cx, cy])

    # Priorité au bord avec le plus de marqueurs détectés
    if len(pts_left) >= 2 and len(pts_left) >= len(pts_right):
        pts = np.array(pts_left)
        side = "left"
    elif len(pts_right) >= 2:
        pts = np.array(pts_right)
        side = "right"
    else:
        return None  # besoin des 2 ArUcos pour calculer la hauteur

    # Centre du bord et hauteur entre les 2 ArUcos
    edge_cx = np.mean(pts[:, 0])
    edge_cy = np.mean(pts[:, 1])
    height  = abs(pts[0][1] - pts[1][1])  # distance verticale en pixels

    # Fenêtre carrée → largeur = hauteur → décalage = hauteur / 2
    if side == "left":
        cx = edge_cx + height / 2
    else:
        cx = edge_cx - height / 2

    return (int(round(cx)), int(round(edge_cy)))
