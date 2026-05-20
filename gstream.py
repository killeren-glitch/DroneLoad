import cv2
import numpy as np
import threading
import time


class VideoManager:
    def __init__(self, ip_dest="192.168.88.25", port=5000, width=640, height=480):
        self.width = width
        self.height = height

        # --- 1. ENTRÉE VIDÉO ---
        pipeline_in = (
            f"libcamerasrc ! "
            f"video/x-raw, width={width}, height={height}, framerate=15/1 ! "
            f"videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1"
        )
        self.cap = cv2.VideoCapture(pipeline_in, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            print("ERREUR : Impossible d'ouvrir la caméra via GStreamer.")

        # --- 2. GESTION DU THREAD (Buffer de taille 1) ---
        self.current_frame = None
        self.ret = False
        self.running = True
        self.lock = threading.Lock()  # Protège la variable contre les accès simultanés

        # Démarrage du thread en mode "daemon" (il s'arrêtera tout seul quand main.py s'arrêtera)
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        # On attend qu'une première image arrive pour être sûr que la caméra est chaude
        print("Attente de la première image caméra...")
        while self.current_frame is None and self.running:
            time.sleep(0.1)

        # --- 3. SORTIE VIDÉO ---
        pipeline_out = (
            f"appsrc ! videoconvert ! video/x-raw,format=I420 ! "
            f"jpegenc ! rtpjpegpay ! "
            f"udpsink host={ip_dest} port={port} sync=false"
        )
        self.writer = cv2.VideoWriter(pipeline_out, cv2.CAP_GSTREAMER, 0, 15, (width, height), True)

    def _capture_loop(self):
        """
        Fonction exécutée en permanence par le thread secondaire.
        Elle lit la caméra et met à jour l'image partagée.
        """
        while self.running:
            ret, frame = self.cap.read()

            with self.lock:
                self.ret = ret
                if ret:
                    self.current_frame = frame

    def get_frame(self):
        """
        Fonction appelée par ta boucle inference
        Retourne l'image
        """
        with self.lock:
            if self.ret and self.current_frame is not None:
                # On retourne une copie pour éviter que l'IA et la caméra
                # ne modifient/écrasent les mêmes pixels en même temps.
                return self.current_frame.copy()
            return None

    def send_frame(self, frame):
        """ Envoie l'image modifiée via UDP """
        self.writer.write(frame)

    def stop(self):
        """ Arrête proprement le thread et libère la caméra """
        self.running = False
        self.capture_thread.join()
        self.cap.release()
        self.writer.release()
        print("VideoManager arrêté.")