import cv2


# TODO: Décommenter lors de l'installation sur la Raspberry
# from ultralytics import YOLO

class YoloProcessor:
    def __init__(self, model_path="yolov8n.pt"):
        print(f"Initialisation du modèle YOLO : {model_path}")
        # TODO: Initialiser le modèle
        # self.model = YOLO(model_path)

    def process(self, frame, canvas):
        """ Analyse 'frame', dessine les bounding boxes sur 'canvas' """
        detections = []

        # TODO: Remplacer par l'inférence réelle
        # results = self.model(frame, verbose=False)
        # for r in results:
        #     for box in r.boxes:
        #         x1, y1, x2, y2 = map(int, box.xyxy[0])
        #         cls_id = int(box.cls[0])
        #         conf = float(box.conf[0])
        #         name = self.model.names[cls_id]
        #
        #         detections.append({"class": name, "box": (x1, y1, x2, y2), "conf": conf})
        #
        #         cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #         cv2.putText(canvas, f"{name} {conf:.2f}", (x1, y1 - 10),
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return canvas, detections