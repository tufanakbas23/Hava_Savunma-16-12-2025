# detector/yolo_detector.py
# ─────────────────────────────────────────────────────────────
# Python (Brain) Layer — YOLO Object Detection Module
# İki mod destekler:
#   1. Stream mode: track_video() — OpenCV kaynak ile (mevcut stage pipeline)
#   2. Frame mode:  detect_frame() — tek NumPy frame ile (ZED entegrasyonu)
# ─────────────────────────────────────────────────────────────

from ultralytics import YOLO
import numpy as np
import logging

# Ultralytics loglarını sustur — sadece hatalar görünsün
logging.getLogger("ultralytics").setLevel(logging.ERROR)

logger = logging.getLogger(__name__)


class Detection:
    """
    Tek bir algılama sonucunu temsil eder.
    Otonom sistem terminolojisi kullanır.

    Attributes:
        class_id   (int):   Model sınıf indeksi
        class_name (str):   Sınıf adı (drone, missile, helicopter, f16, balloon)
        confidence (float): Güven skoru [0.0 - 1.0]
        bbox       (tuple): Bounding box (x1, y1, x2, y2) piksel
        center     (tuple): Merkez nokta (cx, cy) piksel
        track_id   (int):   Takip ID'si (tracking aktifse, yoksa -1)
    """
    __slots__ = ("class_id", "class_name", "confidence", "bbox", "center", "track_id")

    def __init__(self, class_id, class_name, confidence, bbox, track_id=-1):
        self.class_id = class_id
        self.class_name = class_name
        self.confidence = confidence
        self.bbox = bbox  # (x1, y1, x2, y2)
        self.center = (
            (bbox[0] + bbox[2]) / 2.0,
            (bbox[1] + bbox[3]) / 2.0,
        )
        self.track_id = track_id

    def __repr__(self):
        return (
            f"Detection({self.class_name}, conf={self.confidence:.2f}, "
            f"center=({self.center[0]:.0f},{self.center[1]:.0f}), id={self.track_id})"
        )


class YoloDetector:
    """
    YOLO tabanlı nesne algılayıcı.

    İki kullanım modu:
      1. track_video(source) → generator  — Mevcut stage pipeline uyumluluğu
      2. detect_frame(frame) → list[Detection]  — ZED frame entegrasyonu

    Args:
        model_path  (str):  .pt model dosya yolu
        device      (int|str): 0 = GPU, "cpu" = CPU
        use_half    (bool): FP16 half precision (GPU üzerinde ~%30 hız artışı)
        conf        (float): Minimum güven eşiği
        imgsz       (int):  Inference boyutu (480 = hız, 640 = doğruluk)
        active_classes (list[str]|None): Sadece bu sınıfları döndür, None = hepsi
    """

    def __init__(
        self,
        model_path: str,
        device=0,
        use_half: bool = True,
        conf: float = 0.35,
        imgsz: int = 480,
        active_classes: list = None,
    ):
        self.model = YOLO(model_path)
        self.device = device
        self.use_half = use_half
        self.conf = conf
        self.imgsz = imgsz

        # Aktif sınıf filtresi — None ise tüm sınıflar geçer
        self.active_classes = active_classes

        # Model sınıf isimlerini al
        self._class_names = self.model.names  # {0: "balon", 1: "drone", ...}

        logger.info(
            f"✅ YOLO yüklendi: {model_path} | "
            f"Device: {device} | FP16: {use_half} | "
            f"Conf: {conf} | ImgSz: {imgsz} | "
            f"Sınıflar: {list(self._class_names.values())} | "
            f"Aktif: {active_classes or 'ALL'}"
        )

    # ─────────────────── Mode 1: Stream (Mevcut Pipeline) ───────────────────

    def track_video(self, source, tracker="bytetrack.yaml"):
        """
        OpenCV video kaynağından stream tabanlı tracking.
        Mevcut stage pipeline ile tam uyumlu.

        Args:
            source: Kamera index veya video dosya yolu
            tracker: Tracker yapılandırma dosyası

        Yields:
            YOLO Result nesneleri (ultralytics format)
        """
        return self.model.track(
            source=source,
            stream=True,
            tracker=tracker,
            verbose=False,
            persist=True,
            imgsz=self.imgsz,
            vid_stride=1,
            conf=self.conf,
            device=self.device,
            half=self.use_half,
        )

    # ─────────────────── Mode 2: Frame-by-Frame (ZED) ───────────────────

    def detect_frame(self, frame: np.ndarray, use_tracking: bool = False) -> list:
        """
        Tek bir NumPy frame üzerinde YOLO inference çalıştırır.

        Args:
            frame: BGR NumPy dizisi (H, W, 3) — ZED'den veya OpenCV'den
            use_tracking: True ise ByteTrack ile çoklu nesne takibi yapar

        Returns:
            list[Detection]: Algılanan nesnelerin yapılandırılmış listesi.
                             Sadece active_classes'taki sınıflar döndürülür.
        """
        # Inference veya tracking
        if use_tracking:
            results = self.model.track(
                source=frame,
                persist=True,
                verbose=False,
                imgsz=self.imgsz,
                conf=self.conf,
                device=self.device,
                half=self.use_half,
                tracker="bytetrack.yaml",
            )
        else:
            results = self.model.predict(
                source=frame,
                verbose=False,
                imgsz=self.imgsz,
                conf=self.conf,
                device=self.device,
                half=self.use_half,
            )

        # Sonuçları Detection nesnelerine dönüştür
        detections = []
        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue

            boxes = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            classes = r.boxes.cls.cpu().numpy().astype(int)

            # Track ID'ler (tracking aktifse)
            track_ids = (
                r.boxes.id.cpu().numpy().astype(int)
                if r.boxes.id is not None
                else [-1] * len(boxes)
            )

            for i in range(len(boxes)):
                cls_id = classes[i]
                cls_name = self._class_names.get(cls_id, f"unknown_{cls_id}")

                # ─── Aktif sınıf filtresi ───
                if self.active_classes is not None:
                    # Model sınıf adı veya standardize isim eşleşmeli
                    if not self._is_active_class(cls_name):
                        continue

                det = Detection(
                    class_id=cls_id,
                    class_name=cls_name,
                    confidence=float(confs[i]),
                    bbox=tuple(boxes[i]),
                    track_id=int(track_ids[i]),
                )
                detections.append(det)

        return detections

    def _is_active_class(self, cls_name: str) -> bool:
        """
        Sınıf adının aktif sınıflar listesinde olup olmadığını kontrol eder.
        Hem model adı (ör. 'balon') hem de standart ad (ör. 'balloon') ile eşleşir.

        Esnek eşleştirme:
          - "balon" → "balloon" ile eşleşir
          - "Balon" → case-insensitive eşleşir
        """
        if self.active_classes is None:
            return True

        name_lower = cls_name.lower()

        # Doğrudan eşleşme
        for ac in self.active_classes:
            ac_lower = ac.lower()
            if name_lower == ac_lower:
                return True
            # Türkçe/İngilizce esnek eşleştirme
            if name_lower in ("balon", "balloon") and ac_lower in ("balon", "balloon"):
                return True
            if name_lower in ("iha", "drone") and ac_lower in ("iha", "drone"):
                return True
            if name_lower in ("fuzesi", "fuze", "missile") and ac_lower in ("fuzesi", "fuze", "missile"):
                return True
            if name_lower in ("helikopter", "helicopter") and ac_lower in ("helikopter", "helicopter"):
                return True

        return False

    @property
    def class_names(self) -> dict:
        """Model sınıf isimleri sözlüğü."""
        return self._class_names
