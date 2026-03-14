# core/detection_engine.py
# ─────────────────────────────────────────────────────────────
# Python (Brain) Layer — Phase 1.1 Detection Engine
#
# Ana orkestratör: ZED 2 kameradan RGB frame alır, YOLO ile
# 5 sınıf üzerinde algılama yapar, sonuçları annotation ile
# frame üzerine çizer ve generator olarak dışarı verir.
#
# Mimari: ZedCamera → YoloDetector → Annotation → yield
# ─────────────────────────────────────────────────────────────

import time
import cv2
import numpy as np
import logging

from config.system_config import SystemConfig
from camera.zed_camera import ZedCamera, ZED_AVAILABLE
from detector.yolo_detector import YoloDetector, Detection

logger = logging.getLogger(__name__)


# ─────────────────── Renk Paleti (sınıf başına) ───────────────────
# Her sınıf için görsel olarak ayrışan BGR renkleri
CLASS_COLORS = {
    "drone":      (0, 165, 255),   # Turuncu
    "missile":    (0, 0, 255),     # Kırmızı
    "helicopter": (255, 100, 0),   # Mavi
    "f16":        (0, 255, 255),   # Sarı
    "balloon":    (0, 255, 0),     # Yeşil
    # Türkçe model isimleri için fallback
    "balon":      (0, 255, 0),
    "iha":        (0, 165, 255),
    "fuze":       (0, 0, 255),
    "helikopter": (255, 100, 0),
}
DEFAULT_COLOR = (200, 200, 200)    # Bilinmeyen sınıflar için gri


class DetectionEngine:
    """
    Phase 1.1 Ana Algılama Motoru.

    ZED 2 kameradan canlı görüntü alır, YOLO ile nesne algılama yapar
    ve sonuçları annotated frame + detection listesi olarak döndürür.

    Generator-tabanlı tasarım: Mevcut GUI/stage pipeline'ına doğrudan
    entegre edilebilir.

    Kullanım:
        engine = DetectionEngine()
        for frame, detections, telemetry in engine.run():
            # frame: annotated BGR image (np.ndarray)
            # detections: list[Detection]
            # telemetry: dict (fps, detection_count, vb.)
            cv2.imshow("Detection", frame)
    """

    def __init__(self, model_path: str = None, use_zed: bool = None):
        """
        Args:
            model_path: YOLO .pt dosya yolu (None ise config'den alınır)
            use_zed: ZED kamera kullanılsın mı? (None ise config'den alınır)
        """
        # ─── Yapılandırma ───
        self.model_path = model_path or SystemConfig.YOLO_MODEL_STAGE1
        self.use_zed = use_zed if use_zed is not None else SystemConfig.USE_ZED_CAMERA

        # ZED SDK yoksa otomatik fallback
        if self.use_zed and not ZED_AVAILABLE:
            logger.warning("ZED SDK bulunamadı → OpenCV fallback moduna geçiliyor.")
            self.use_zed = False

        # ─── YOLO Detector ───
        self.detector = YoloDetector(
            model_path=self.model_path,
            device=SystemConfig.YOLO_DEVICE,
            use_half=SystemConfig.YOLO_HALF,
            conf=SystemConfig.YOLO_CONF_THRESHOLD,
            imgsz=SystemConfig.YOLO_IMGSZ,
            active_classes=SystemConfig.ACTIVE_CLASSES,
        )

        # ─── Kamera ───
        self.zed_camera: ZedCamera | None = None
        self.cv_capture = None

        # ─── Durum ───
        self.should_stop = False
        self._prev_time = time.time()
        self._smooth_fps = 0.0
        self._frame_count = 0

        logger.info(
            f"[DetectionEngine] Model: {self.model_path} | "
            f"Kamera: {'ZED 2' if self.use_zed else 'OpenCV'} | "
            f"Aktif sınıflar: {SystemConfig.ACTIVE_CLASSES}"
        )

    # ─────────────────── Ana Döngü (Generator) ───────────────────

    def run(self):
        """
        Ana algılama döngüsü — generator.

        Yields:
            tuple: (annotated_frame, detections, telemetry)
                - annotated_frame: np.ndarray (BGR, H×W×3) — bounding box + etiketli
                - detections: list[Detection] — yapılandırılmış algılama sonuçları
                - telemetry: dict — FPS, algılama sayısı, kamera modu vb.
        """
        try:
            self._open_camera()

            while not self.should_stop:
                # ─── Frame Al ───
                frame, depth = self._grab_frame()
                if frame is None:
                    continue

                # ─── YOLO Inference ───
                if self.use_zed:
                    # Frame-by-frame mod (ZED)
                    detections = self.detector.detect_frame(frame, use_tracking=True)
                else:
                    # Frame-by-frame mod (OpenCV fallback)
                    detections = self.detector.detect_frame(frame, use_tracking=True)

                # ─── FPS Hesapla ───
                fps = self._calculate_fps()
                self._frame_count += 1

                # ─── Annotation (Bounding Box + Etiket Çizimi) ───
                annotated = self._draw_annotations(frame, detections, fps)

                # ─── Telemetri ───
                telemetry = {
                    "fps": fps,
                    "frame_count": self._frame_count,
                    "detection_count": len(detections),
                    "camera_mode": "ZED 2" if self.use_zed else "OpenCV",
                    "active_classes": SystemConfig.ACTIVE_CLASSES,
                    "detections": detections,  # Yapılandırılmış veri
                }

                yield annotated, detections, telemetry

        finally:
            self._close_camera()

    def stop(self):
        """Algılama döngüsünü durdurur."""
        self.should_stop = True

    # ─────────────────── Kamera Yönetimi ───────────────────

    def _open_camera(self):
        """Yapılandırmaya göre ZED veya OpenCV kamerasını açar."""
        if self.use_zed:
            self.zed_camera = ZedCamera(
                resolution=SystemConfig.ZED_RESOLUTION,
                fps=SystemConfig.ZED_FPS,
                depth_mode=SystemConfig.ZED_DEPTH_MODE,
                max_depth_m=SystemConfig.ZED_MAX_DEPTH_M,
            )
            self.zed_camera.open()
        else:
            self.cv_capture = cv2.VideoCapture(
                SystemConfig.CAMERA_INDEX, cv2.CAP_DSHOW
            )
            self.cv_capture.set(cv2.CAP_PROP_FRAME_WIDTH, SystemConfig.FRAME_WIDTH)
            self.cv_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, SystemConfig.FRAME_HEIGHT)
            self.cv_capture.set(cv2.CAP_PROP_FPS, 30)
            if not self.cv_capture.isOpened():
                raise RuntimeError("OpenCV kamera açılamadı!")
            logger.info(
                f"✅ OpenCV kamera açıldı — "
                f"{SystemConfig.FRAME_WIDTH}x{SystemConfig.FRAME_HEIGHT}"
            )

    def _close_camera(self):
        """Kamerayı güvenli şekilde kapatır."""
        if self.zed_camera is not None:
            self.zed_camera.close()
            self.zed_camera = None
        if self.cv_capture is not None:
            self.cv_capture.release()
            self.cv_capture = None
        logger.info("📷 Kamera kapatıldı.")

    def _grab_frame(self):
        """
        Aktif kameradan bir frame alır.

        Returns:
            (frame, depth): BGR np.ndarray + depth verisi (ZED) veya None
        """
        if self.use_zed and self.zed_camera is not None:
            return self.zed_camera.grab_frame()
        elif self.cv_capture is not None:
            ret, frame = self.cv_capture.read()
            if not ret:
                return None, None
            return frame, None
        return None, None

    # ─────────────────── FPS Hesaplayıcı ───────────────────

    def _calculate_fps(self) -> float:
        """Yumuşatılmış FPS (exponential moving average)."""
        now = time.time()
        dt = now - self._prev_time
        instant_fps = 1.0 / dt if dt > 0 else 0.0
        self._smooth_fps = 0.1 * instant_fps + 0.9 * self._smooth_fps
        self._prev_time = now
        return self._smooth_fps

    # ─────────────────── Annotation (Çizim) ───────────────────

    def _draw_annotations(
        self,
        frame: np.ndarray,
        detections: list,
        fps: float,
    ) -> np.ndarray:
        """
        Algılama sonuçlarını frame üzerine çizer.

        Her tespit için:
          - Bounding box (sınıf renginde)
          - Sınıf adı + güven skoru
          - Merkez noktası

        Ek olarak:
          - FPS göstergesi
          - Algılama sayısı
          - Nişangah (crosshair)
        """
        annotated = frame.copy()
        h, w = annotated.shape[:2]

        # ─── Her tespit için bounding box + etiket ───
        for det in detections:
            color = CLASS_COLORS.get(det.class_name.lower(), DEFAULT_COLOR)
            x1, y1, x2, y2 = map(int, det.bbox)

            # Bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            # Etiket metni: SINIF_ADI  %95  [ID:3]
            label = f"{det.class_name.upper()} {det.confidence:.0%}"
            if det.track_id >= 0:
                label += f" [ID:{det.track_id}]"

            # Etiket arka planı (okunabilirlik)
            (tw, th), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2
            )
            cv2.rectangle(
                annotated,
                (x1, y1 - th - 8),
                (x1 + tw + 6, y1),
                color,
                -1,
            )
            cv2.putText(
                annotated,
                label,
                (x1 + 3, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),  # Siyah metin (kontrast)
                2,
            )

            # Merkez noktası
            cx, cy = int(det.center[0]), int(det.center[1])
            cv2.circle(annotated, (cx, cy), 4, color, -1)

        # ─── HUD Overlay ───
        # FPS + Algılama sayısı
        hud_text = (
            f"FPS: {fps:.0f} | "
            f"Detections: {len(detections)} | "
            f"{'ZED 2' if self.use_zed else 'OpenCV'}"
        )
        cv2.putText(
            annotated, hud_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (0, 255, 0), 2,
        )

        # Aktif sınıflar göstergesi
        classes_text = f"Classes: {', '.join(SystemConfig.ACTIVE_CLASSES)}"
        cv2.putText(
            annotated, classes_text,
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (200, 200, 200), 1,
        )

        # Nişangah (ekran merkezi)
        cx_frame = w // 2
        cy_frame = h // 2
        cross_size = 15
        cv2.line(
            annotated,
            (cx_frame - cross_size, cy_frame),
            (cx_frame + cross_size, cy_frame),
            (0, 255, 0), 1,
        )
        cv2.line(
            annotated,
            (cx_frame, cy_frame - cross_size),
            (cx_frame, cy_frame + cross_size),
            (0, 255, 0), 1,
        )

        return annotated


# ─────────────────── Standalone Test ───────────────────
# Doğrudan çalıştırma: python -m core.detection_engine
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s | %(message)s")

    print("=" * 60)
    print("  Phase 1.1 — Detection Engine Test")
    print("  ZED 2 + YOLO Real-Time Detection")
    print("=" * 60)
    SystemConfig.print_config()

    engine = DetectionEngine()

    try:
        for frame, detections, telemetry in engine.run():
            cv2.imshow("Phase 1.1 - Detection Engine", frame)

            # Tespit detaylarını konsola yazdır (her 30 frame'de)
            if telemetry["frame_count"] % 30 == 0 and detections:
                print(f"\n[Frame {telemetry['frame_count']}] "
                      f"FPS: {telemetry['fps']:.0f} | "
                      f"Detections: {len(detections)}")
                for det in detections:
                    print(f"  → {det}")

            # 'q' ile çıkış
            if cv2.waitKey(1) & 0xFF == ord("q"):
                engine.stop()
                break
    except KeyboardInterrupt:
        engine.stop()
    finally:
        cv2.destroyAllWindows()
        print("\n✅ Detection Engine kapatıldı.")
