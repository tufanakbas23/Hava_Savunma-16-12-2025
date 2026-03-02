"""
Base Stage Engine - Basit P Kontrol + Lead Prediction + Feedforward
"""

import time
import cv2
import numpy as np
from detector.yolo_detector import YoloDetector
from classify.size_classifier import SizeClassifier
from control.ibvs_controller import IBVSController
from config.system_config import SystemConfig


class BaseStageEngine:
    """
    Tüm stage'lerin base class'ı
    Belgede: Depth-aware IBVS sistemi
    """

    def __init__(self, model_path, arduino, camera_index=0):
        self.detector = YoloDetector(model_path, device=0, use_half=True)
        self.size_classifier = SizeClassifier(large_threshold=10000)
        self.arduino = arduino
        self.camera_index = camera_index

        # ===== KAMERA MODU (SystemConfig'den) =====
        self.camera_on_turret = SystemConfig.CAMERA_ON_TURRET
        
        # ===== Kontrol Parametreleri (SystemConfig'den) =====
        self.LEAD_TIME_S = SystemConfig.LEAD_TIME_S
        self.DEAD_ZONE_NORM = SystemConfig.DEAD_ZONE_NORM
        self.MAX_DELTA_DEG = SystemConfig.MAX_DELTA_DEG
        self.Kp_fixed = SystemConfig.KP_FIXED
        self.DEAD_ZONE_PIX = SystemConfig.DEAD_ZONE_PIX

        # FOV
        self.HFOV_DEG = SystemConfig.HFOV_DEG
        self.VFOV_DEG = SystemConfig.VFOV_DEG

        # ===== Basit P Kontrol =====
        self.ibvs_controller = IBVSController(kp=SystemConfig.KP)

        # Lock
        self.LOCK_TOLERANCE_PIX = SystemConfig.LOCK_TOLERANCE_PIX
        self.LOCK_DURATION_FRAMES = SystemConfig.LOCK_DURATION_FRAMES
        self.lock_frame_counter = 0

        # State
        self.tracked_targets = {}
        self.should_stop = False
        self.prev_time = time.time()
        self.frame_counter = 0

        cam_mode = "HAREKETLİ (IBVS)" if self.camera_on_turret else "SABİT (P kontrol)"
        print(f"[BaseStage] Kamera modu: {cam_mode}")

    def calculate_fps(self):
        now = time.time()
        dt = now - self.prev_time
        instant_fps = 1 / dt if dt > 0 else 0
        # Hareketli ortalama: yumuşak gösterim
        if not hasattr(self, '_smooth_fps'):
            self._smooth_fps = instant_fps
        self._smooth_fps = 0.1 * instant_fps + 0.9 * self._smooth_fps
        self.prev_time = now
        self.frame_counter += 1
        return self._smooth_fps

    def draw_crosshair(self, frame, color=(0, 255, 0), size=3):
        cx, cy = 320, 240
        cv2.line(frame, (cx - size, cy), (cx + size, cy), color, 2)
        cv2.line(frame, (cx, cy - size), (cx, cy + size), color, 2)

    def draw_fps_overlay(self, frame, fps, stage_name):
        cv2.putText(
            frame,
            f"FPS: {fps:.0f} | {stage_name}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )

    def update_turret_ibvs(
        self,
        target_x,
        target_y,
        vx_pix,
        vy_pix,
        bbox_height,
        frame_width,
        frame_height,
        dt,
    ):
        """
        Turret kontrolü - kamera moduna göre farklı algoritma kullanır.

        Args:
            target_x, target_y: Hedefin piksel koordinatları
            vx_pix, vy_pix: Hedefin piksel hızı (Kalman'dan)
            bbox_height: Bounding box yüksekliği (depth için)
            frame_width, frame_height: Frame boyutları
            dt: Delta time

        Returns:
            locked (bool): Hedef kilitli mi?
            depth_m (float): Tahmini derinlik (metre)
        """
        h, w = frame_height, frame_width
        cx, cy = 320, 240  # Kameranın orta noktası
        depth_m = 5.0  # Sabit derinlik (kullanılmasa da dönüş değeri için)
        
        # Hedefin merkeze olan piksel hatası
        error_x = target_x - cx  # Pozitif = hedef sağda
        error_y = target_y - cy  # Pozitif = hedef aşağıda
        
        if self.camera_on_turret:
            # ===== HAREKETLİ KAMERA: P Kontrol + Lead Prediction + Feedforward =====
            
            # Lead Prediction: Balonun ilerisine nişan al
            predicted_x = target_x + vx_pix * self.LEAD_TIME_S
            predicted_y = target_y + vy_pix * self.LEAD_TIME_S
            
            error_x_pred = predicted_x - cx
            error_y_pred = predicted_y - cy
            
            error_x_norm = error_x_pred / (w / 2)
            error_y_norm = error_y_pred / (h / 2)

            # DEAD ZONE kontrolü - merkeze yakınsa hareket etme
            if abs(error_x_norm) < self.DEAD_ZONE_NORM and abs(error_y_norm) < self.DEAD_ZONE_NORM:
                dpan_deg = 0.0
                dtilt_deg = 0.0
            else:
                dpan_cmd, dtilt_cmd = self.ibvs_controller.compute(
                    error_x_norm, error_y_norm
                )

                # FOV ölçekleme: normalize hata → derece/saniye
                dpan_deg = -dpan_cmd * (self.HFOV_DEG / 2) * dt
                dtilt_deg = -dtilt_cmd * (self.VFOV_DEG / 2) * dt
                
                # ===== FEEDFORWARD: Hedefin hızını direkt motor komutuna ekle =====
                # Piksel hızını derece/frame'e çevir ve ekle
                ff_gain = SystemConfig.FEEDFORWARD_GAIN
                if ff_gain > 0 and (abs(vx_pix) > 1.0 or abs(vy_pix) > 1.0):
                    # Piksel hızı → açısal hız (derece/saniye) → derece/frame
                    ff_pan_deg = -(vx_pix * self.HFOV_DEG / w) * dt * ff_gain
                    ff_tilt_deg = -(vy_pix * self.VFOV_DEG / h) * dt * ff_gain
                    dpan_deg += ff_pan_deg
                    dtilt_deg += ff_tilt_deg
                
                # Rate Limiter: Ani hareketleri sınırla
                dpan_deg = max(-self.MAX_DELTA_DEG, min(self.MAX_DELTA_DEG, dpan_deg))
                dtilt_deg = max(-self.MAX_DELTA_DEG, min(self.MAX_DELTA_DEG, dtilt_deg))

            # Arduino'ya gönder (smooth)
            self.arduino.update_angles_smooth(dpan_deg, dtilt_deg)
            
            # Lock için error kullan
            lock_error_x = error_x_pred
            lock_error_y = error_y_pred
            
        else:
            # ===== SABİT KAMERA: Basit Proportional Kontrol =====
            
            depth_m = 5.0  # Sabit değer
            
            # Piksel hatasını açıya çevir
            deg_per_pixel_x = self.HFOV_DEG / w
            deg_per_pixel_y = self.VFOV_DEG / h
            
            # Hedef açı değişimi
            dpan_deg = error_x * deg_per_pixel_x * self.Kp_fixed
            dtilt_deg = error_y * deg_per_pixel_y * self.Kp_fixed
            
            # Dead zone - titreşimi önler
            if abs(error_x) < self.DEAD_ZONE_PIX:
                dpan_deg = 0
            if abs(error_y) < self.DEAD_ZONE_PIX:
                dtilt_deg = 0

            # Arduino'ya gönder
            self.arduino.update_angles(dpan_deg, dtilt_deg)
            
            # Lock için mevcut error kullan
            lock_error_x = error_x
            lock_error_y = error_y

        # Lock kontrolü (her iki mod için ortak)
        locked = (
            abs(lock_error_x) < self.LOCK_TOLERANCE_PIX
            and abs(lock_error_y) < self.LOCK_TOLERANCE_PIX
        )

        if locked:
            self.lock_frame_counter += 1
        else:
            self.lock_frame_counter = 0

        sustained_lock = locked and (
            self.lock_frame_counter >= self.LOCK_DURATION_FRAMES
        )

        return sustained_lock, depth_m

    def filter_shadow(self, crop, threshold=90):
        if crop.size == 0:
            return True
        lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB)
        L = np.mean(lab[:, :, 0])
        return L < threshold

    def draw_target_box(self, frame, box, label, color, thickness=2):
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        cv2.putText(
            frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
        )

    def get_frame_generator(self):
        # Kamera ayarlarını sabitle
        import cv2
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.release()
        
        return self.detector.track_video(
            source=self.camera_index, tracker="bytetrack.yaml"
        )

    def run(self):
        raise NotImplementedError("Her stage kendi run() metodunu implement etmeli!")
