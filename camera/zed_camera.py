# camera/zed_camera.py
# ─────────────────────────────────────────────────────────────
# Python (Brain) Layer — ZED 2 Stereo Camera Interface
# Modüler, thread-safe kamera soyutlaması.
# ZED SDK kullanarak RGB frame + depth map sağlar.
# ─────────────────────────────────────────────────────────────

import numpy as np
import logging

logger = logging.getLogger(__name__)

# ZED SDK opsiyonel import — yoksa fallback moduna geçilir
try:
    import pyzed.sl as sl
    ZED_AVAILABLE = True
except ImportError:
    ZED_AVAILABLE = False
    logger.warning("pyzed.sl bulunamadı — ZED kamera devre dışı, OpenCV fallback aktif.")


# ───── Çözünürlük & Derinlik mod mapping ─────
_RESOLUTION_MAP = {
    "HD2K":   sl.RESOLUTION.HD2K   if ZED_AVAILABLE else None,
    "HD1080": sl.RESOLUTION.HD1080 if ZED_AVAILABLE else None,
    "HD720":  sl.RESOLUTION.HD720  if ZED_AVAILABLE else None,
    "VGA":    sl.RESOLUTION.VGA    if ZED_AVAILABLE else None,
}

_DEPTH_MODE_MAP = {
    "NONE":        sl.DEPTH_MODE.NONE        if ZED_AVAILABLE else None,
    "PERFORMANCE": sl.DEPTH_MODE.PERFORMANCE if ZED_AVAILABLE else None,
    "QUALITY":     sl.DEPTH_MODE.QUALITY     if ZED_AVAILABLE else None,
    "ULTRA":       sl.DEPTH_MODE.ULTRA       if ZED_AVAILABLE else None,
}


class ZedCamera:
    """
    ZED 2 stereo kamera sarmalayıcısı (wrapper).

    Sorumlulukları:
      - Kamerayı açma / kapatma (lifecycle)
      - Her frame'de RGB (BGR NumPy) + depth Mat döndürme
      - Context manager desteği (with ... as cam)

    Kullanım:
        with ZedCamera(resolution="HD720", fps=60) as cam:
            while True:
                rgb, depth = cam.grab_frame()
                if rgb is None:
                    continue
                # ... YOLO inference vb.
    """

    def __init__(
        self,
        resolution: str = "HD720",
        fps: int = 60,
        depth_mode: str = "ULTRA",
        max_depth_m: float = 20.0,
    ):
        if not ZED_AVAILABLE:
            raise RuntimeError(
                "ZED SDK (pyzed) yüklü değil. "
                "Lütfen 'pip install pyzed-sl' veya ZED SDK kurulumu yapın."
            )

        self.resolution_key = resolution
        self.fps = fps
        self.depth_mode_key = depth_mode
        self.max_depth_m = max_depth_m

        # ZED nesneleri — open() ile başlatılır
        self._camera: sl.Camera | None = None
        self._image_mat: sl.Mat | None = None
        self._depth_mat: sl.Mat | None = None
        self._runtime_params: sl.RuntimeParameters | None = None
        self._is_open = False

    # ─────────────────── Lifecycle ───────────────────

    def open(self) -> None:
        """ZED kamerasını yapılandırır ve açar."""
        if self._is_open:
            logger.warning("ZED kamera zaten açık.")
            return

        self._camera = sl.Camera()

        init_params = sl.InitParameters()
        init_params.camera_resolution = _RESOLUTION_MAP.get(
            self.resolution_key, sl.RESOLUTION.HD720
        )
        init_params.camera_fps = self.fps
        init_params.depth_mode = _DEPTH_MODE_MAP.get(
            self.depth_mode_key, sl.DEPTH_MODE.ULTRA
        )
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_maximum_distance = self.max_depth_m

        status = self._camera.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED kamera açılamadı: {status}")

        # Tekrar kullanılacak mat nesneleri (bellek tasarrufu)
        self._image_mat = sl.Mat()
        self._depth_mat = sl.Mat()

        # Runtime parametreleri
        self._runtime_params = sl.RuntimeParameters()
        self._runtime_params.confidence_threshold = 50
        self._runtime_params.texture_confidence_threshold = 100

        self._is_open = True

        # Kamera bilgileri
        cam_info = self._camera.get_camera_information()
        res = cam_info.camera_configuration.resolution
        logger.info(
            f"✅ ZED 2 açıldı — {res.width}x{res.height} @ {self.fps}fps | "
            f"Depth: {self.depth_mode_key} | Max: {self.max_depth_m}m"
        )

    def close(self) -> None:
        """ZED kamerasını güvenli bir şekilde kapatır."""
        if self._camera is not None and self._is_open:
            self._camera.close()
            self._is_open = False
            logger.info("📷 ZED kamera kapatıldı.")

    @property
    def is_open(self) -> bool:
        return self._is_open

    # ─────────────────── Frame Grab ───────────────────

    def grab_frame(self):
        """
        Tek bir frame yakalar.

        Returns:
            tuple: (rgb_frame, depth_mat)
                - rgb_frame: np.ndarray (H, W, 3) BGR format — YOLO'ya hazır
                - depth_mat: sl.Mat — ZED depth verisi (Phase 1.2'de kullanılacak)
                Eğer frame alınamazsa (None, None) döner.
        """
        if not self._is_open:
            return None, None

        if self._camera.grab(self._runtime_params) != sl.ERROR_CODE.SUCCESS:
            return None, None

        # Sol kamera görüntüsü (RGB → BGR NumPy)
        self._camera.retrieve_image(self._image_mat, sl.VIEW.LEFT)

        # RGBA → BGR dönüşümü (ZED Mat 4 kanallı döner)
        rgba = self._image_mat.get_data()
        rgb_frame = rgba[:, :, :3].copy()  # copy() → YOLO güvenli bellek

        # Depth map (Phase 1.2 için hazır, şimdilik geçiriyoruz)
        self._camera.retrieve_measure(self._depth_mat, sl.MEASURE.DEPTH)

        return rgb_frame, self._depth_mat

    # ─────────────────── Depth Estimation (Phase 1.2) ───────────────────

    def get_depth_at_bbox(self, depth_mat, x1, y1, x2, y2):
        """
        Bounding box merkezindeki yaklaşık mesafeyi hesaplar.

        ZED depth map üzerinde merkez nokta etrafında 5×5 piksel
        ortalaması alarak kararlı bir mesafe değeri üretir.

        Args:
            depth_mat: sl.Mat — grab_frame()'den dönen depth verisi
            x1, y1, x2, y2: Bounding box koordinatları (piksel)

        Returns:
            float | None: Mesafe (metre), geçersizse None
        """
        if depth_mat is None:
            return None

        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # 5×5 komşuluk — median ile gürültüye dayanıklı
        distances = []
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                err, depth_value = depth_mat.get_value(cx + dx, cy + dy)
                if err == sl.ERROR_CODE.SUCCESS:
                    d = float(depth_value)
                    # NaN/Inf ve aralık dışı kontrolü
                    if not (np.isnan(d) or np.isinf(d)) and 0.1 < d < self.max_depth_m:
                        distances.append(d)

        if distances:
            return float(np.median(distances))
        return None

    @staticmethod
    def get_depth_at_point(depth_mat, cx, cy):
        """
        Tek bir piksel noktasındaki derinliği döndürür.

        Args:
            depth_mat: sl.Mat — ZED depth verisi
            cx, cy: Piksel koordinatları

        Returns:
            float | None: Mesafe (metre)
        """
        if depth_mat is None:
            return None
        err, depth_value = depth_mat.get_value(int(cx), int(cy))
        if err == sl.ERROR_CODE.SUCCESS:
            d = float(depth_value)
            if not (np.isnan(d) or np.isinf(d)) and d > 0.1:
                return d
        return None

    def get_current_fps(self) -> float:
        """ZED SDK'nın raporladığı anlık FPS."""
        if self._is_open:
            return self._camera.get_current_fps()
        return 0.0

    def get_resolution(self):
        """(width, height) tuple döner."""
        if self._is_open:
            cam_info = self._camera.get_camera_information()
            res = cam_info.camera_configuration.resolution
            return res.width, res.height
        return 640, 480  # Varsayılan

    # ─────────────────── Context Manager ───────────────────

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False
