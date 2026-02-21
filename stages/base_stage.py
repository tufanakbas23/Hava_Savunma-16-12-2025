"""
Base Stage Engine - Sadece IBVS (PD kaldırıldı)
Belgede: Bölüm 3.1, 4, 13
"""

import time
import cv2
import numpy as np
from detector.yolo_detector import YoloDetector
from classify.size_classifier import SizeClassifier
from control.ibvs_controller import IBVSController, VelocityTransformer
from control.depth_estimator import DepthEstimator
from control.motion_compensator import MotionCompensator
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
        self.use_full_ibvs = SystemConfig.USE_FULL_IBVS
        
        # ===== Kontrol Parametreleri (SystemConfig'den) =====
        self.LEAD_TIME_S = SystemConfig.LEAD_TIME_S
        self.DEAD_ZONE_NORM = SystemConfig.DEAD_ZONE_NORM
        self.MAX_DELTA_DEG = SystemConfig.MAX_DELTA_DEG
        self.Kp_fixed = SystemConfig.KP_FIXED
        self.DEAD_ZONE_PIX = SystemConfig.DEAD_ZONE_PIX

        # FOV
        self.HFOV_DEG = SystemConfig.HFOV_DEG
        self.VFOV_DEG = SystemConfig.VFOV_DEG

        # ===== IBVS Adaptif PD Controller =====
        self.ibvs_controller = IBVSController(
            lambda_min=SystemConfig.LAMBDA_MIN,
            lambda_max=SystemConfig.LAMBDA_MAX,
            kd_gain=SystemConfig.KD_GAIN
        )

        # ===== Depth Estimator (Belgede: Bölüm 5.3) =====
        self.depth_estimator = DepthEstimator(
            camera_height_m=2.0,
            tilt_singularity_deg=5.0,
            bbox_depth_coeff=5.0,
            depth_min_m=1.0,
            depth_max_m=15.0,
        )

        # ===== Motion Compensator (Belgede: Bölüm 4 - Katman 3) =====
        self.motion_compensator = MotionCompensator(
            hfov_deg=self.HFOV_DEG,
            vfov_deg=self.VFOV_DEG,
            frame_width=1280,  # 720p
            frame_height=720,
            omega_lowpass_alpha=0.3,
        )

        # ===== Velocity Transformer (Belgede: Bölüm 5.4, 5.5) =====
        self.velocity_transformer = VelocityTransformer(
            hfov_deg=self.HFOV_DEG,
            vfov_deg=self.VFOV_DEG,
            frame_width=1280,  # 720p
            frame_height=720,
        )

        # ===== Lead prediction (Belgede: Bölüm 6.4) =====
        self.LEAD_TIME_BASE = 0.15
        self.LOCK_TOLERANCE_PIX = 10
        self.LOCK_DURATION_FRAMES = 10
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
        fps = 1 / (now - self.prev_time) if (now - self.prev_time) > 0 else 0
        self.prev_time = now
        self.frame_counter += 1
        return fps

    def draw_crosshair(self, frame, color=(0, 255, 0), size=15):
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
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
        cx, cy = w / 2, h / 2
        
        # Hedefin merkeze olan piksel hatası
        error_x = target_x - cx  # Pozitif = hedef sağda
        error_y = target_y - cy  # Pozitif = hedef aşağıda
        
        if self.camera_on_turret:
            # ===== HAREKETLİ KAMERA =====
            
            if self.use_full_ibvs:
                # ===== FULL IBVS: Motion Compensation + Lead Prediction =====
                
                # KATMAN 2: Depth Estimation
                tilt_angle_deg = self.arduino.tilt_deg
                depth_m = self.depth_estimator.estimate_depth_hybrid(
                    tilt_angle_deg, bbox_height, h
                )

                # KATMAN 3: Motion Compensation
                omega_pan, omega_tilt = self.arduino.get_omega()
                self.motion_compensator.update_omega_smooth(omega_pan, omega_tilt)
                vx_comp_pix, vy_comp_pix = self.motion_compensator.compensate_velocity(
                    vx_pix, vy_pix, target_x, target_y
                )

                # KATMAN 4: Velocity Transform + Lead Prediction
                omega_h_deg, omega_v_deg = self.velocity_transformer.pixel_velocity_to_angular(
                    vx_comp_pix, vy_comp_pix
                )
                vx_real, vy_real = self.velocity_transformer.angular_to_real_velocity(
                    omega_h_deg, omega_v_deg, depth_m
                )
                x_pix_pred, y_pix_pred = self.velocity_transformer.lead_prediction_pixel(
                    target_x, target_y, vx_real, vy_real, depth_m, self.LEAD_TIME_BASE
                )

                # KATMAN 5: IBVS Control
                error_x_pred = x_pix_pred - cx
                error_y_pred = y_pix_pred - cy
            else:
                # ===== BASİT IBVS + Lead Prediction + Feedforward =====
                depth_m = 5.0  # Sabit değer
                
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
                omega_pan_deg, omega_tilt_deg = self.ibvs_controller.compute_control_velocity(
                    error_x_norm, error_y_norm
                )

                # Pan yönü tersine çevrildi (motor/kamera kurulumuna göre)
                dpan_deg = -omega_pan_deg * dt
                dtilt_deg = omega_tilt_deg * dt
                
                # ===== FEEDFORWARD: Hedefin hızını direkt motor komutuna ekle =====
                # Piksel hızını derece/frame'e çevir ve ekle
                ff_gain = SystemConfig.FEEDFORWARD_GAIN
                if ff_gain > 0 and (abs(vx_pix) > 1.0 or abs(vy_pix) > 1.0):
                    # Piksel hızı → açısal hız (derece/saniye) → derece/frame
                    ff_pan_deg = -(vx_pix * self.HFOV_DEG / w) * dt * ff_gain
                    ff_tilt_deg = (vy_pix * self.VFOV_DEG / h) * dt * ff_gain
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
        # ✅ DÜZELTİLDİ: track_video() + source parametresi
        return self.detector.track_video(
            source=self.camera_index, tracker="bytetrack.yaml"
        )

    def run(self):
        raise NotImplementedError("Her stage kendi run() metodunu implement etmeli!")
