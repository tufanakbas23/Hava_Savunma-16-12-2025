# detector/yolo_detector.py
from ultralytics import YOLO
import logging

# YOLO loglarını kapat
logging.getLogger("ultralytics").setLevel(logging.ERROR)


class YoloDetector:
    def __init__(self, model_path, device=0, use_half=True):
        self.model = YOLO(model_path)
        self.device = device

    def track_video(self, source, tracker="bytetrack.yaml"):

        return self.model.track(
            source=source,
            stream=True,
            tracker=tracker,
            verbose=False,
            persist=True,
            imgsz=480,          # 640→480: inference %30 hızlanır
            vid_stride=1,       # Her frame'i işle
        )
