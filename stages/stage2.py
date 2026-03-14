"""
Stage 2: FULLY AUTONOMOUS Swarm Tracking & Engagement
Phase 2.1-2.2: Multi-Object Tracking + Vectorial Direction & Depth Analysis

- Python (Brain) katmanı
- YOLO + ByteTrack ile çoklu nesne takibi
- ZED 2 derinlik verisi ile Z-ekseni (yaklaşma) analizi
- En yüksek öncelikli yaklaşan hedef için Error_X, Error_Y UART paketi

NOTLAR (Stage 2 için kritik):
- Tamamen otonom mod, manuel müdahale yok.
- Renk tabanlı herhangi bir filtreleme YOK (HSV, LAB threshold kullanılmıyor).
- Beklenen sınıflar: Ballistic Missile, UAV, Mini/Micro UAV.
"""

import time
import collections
from typing import Deque, Dict, Tuple, Optional

import cv2
import numpy as np

from stages.base_stage import BaseStageEngine
from config.system_config import SystemConfig
from control.fire_controller import FireController


DepthHistory = Deque[Tuple[float, float]]  # (timestamp, depth_m)


class Stage2Engine(BaseStageEngine):
    """
    Stage 2: Otonom Swarm Takip ve Angajman Motoru.

    - YOLO + ByteTrack ile sağlam çoklu hedef takibi
    - ZED 2 (varsa) ile derinlik tabanlı yaklaşma (Z-ekseni) analizi
    - En kritik yaklaşan hedefin Error_X, Error_Y verisini Arduino'ya UART ile gönderir
    - Turret hizalamasını IBVS üzerinden otomatik yapar
    """

    def __init__(self, model_path, arduino, camera_index=0):
        super().__init__(model_path, arduino, camera_index)

        # Stage 2 için tehdit sınıflarını kullan
        self.detector.active_classes = SystemConfig.STAGE2_ACTIVE_CLASSES
        # Arayüzden kontrol edilebilen otonom takip bayrağı
        self.auto_tracking_enabled: bool = True

        # Hedef başına derinlik geçmişi ve yaklaşma analizi
        self.depth_histories: Dict[int, DepthHistory] = {}
        self.target_meta: Dict[int, dict] = {}

        # Zaman ve yaklaşma parametreleri
        self.min_dist = SystemConfig.STAGE2_MIN_DISTANCE_M
        self.safety_margin = SystemConfig.STAGE2_SAFETY_MARGIN_M
        self.max_engage_dist = SystemConfig.STAGE2_MAX_ENGAGE_DISTANCE_M
        self.min_vz = SystemConfig.STAGE2_MIN_APPROACH_VZ_MPS
        self.approach_window = SystemConfig.STAGE2_APPROACH_WINDOW_SEC

        self.class_priority = SystemConfig.STAGE2_CLASS_PRIORITY

        # Kilitlenme ve atış mantığı (Stage 2 temel ateş kontrolü)
        self.fire_controller = FireController(
            fire_after_lock_s=4.0,
        )

        cam_src = "ZED 2" if self.use_zed else "OpenCV"
        print(f"[Stage2] OTONOM swarm motoru aktif | Kamera: {cam_src}")

        # Stage LED ve otonom mod
        try:
            if hasattr(self.arduino, "send_stage_led"):
                self.arduino.send_stage_led(2)
            if hasattr(self.arduino, "set_mode"):
                self.arduino.set_mode(True)
        except Exception:
            pass

    # ─────────────────── Ana Çalışma Döngüsü ───────────────────

    def run(self):
        """
        Generator: her frame için (frame, telemetry) döndürür.
        Stage 2 TAMAMEN OTONOM moddur.
        """
        if self.use_zed:
            yield from self._run_zed_auto()
        else:
            yield from self._run_opencv_auto()

    # ─────────────────── ZED Frame-by-Frame Modu ───────────────────

    def _run_zed_auto(self):
        try:
            self.open_zed_camera()

            while not self.should_stop:
                frame, depth = self.grab_zed_frame()
                if frame is None:
                    continue

                fps = self.calculate_fps()
                detections = self.detector.detect_frame(frame, use_tracking=True)

                frame, telemetry = self._process_frame_auto(
                    frame, detections, fps, depth_mat=depth
                )
                yield frame, telemetry

        finally:
            self.close_zed_camera()

    # ─────────────────── OpenCV Stream Modu ───────────────────

    def _run_opencv_auto(self):
        results = self.get_frame_generator()

        for r in results:
            if self.should_stop:
                break

            frame = r.orig_img.copy()
            fps = self.calculate_fps()

            # Ultralytics Result → Detection listesi
            from detector.yolo_detector import Detection

            detections = []
            if r.boxes is not None and r.boxes.id is not None:
                ids = r.boxes.id.cpu().numpy().astype(int)
                boxes = r.boxes.xyxy.cpu().numpy()
                confs = r.boxes.conf.cpu().numpy()
                classes = r.boxes.cls.cpu().numpy().astype(int)
                class_names = r.names

                for i in range(len(boxes)):
                    cls_id = classes[i]
                    cls_name = class_names[cls_id]

                    if not self.detector._is_active_class(cls_name):
                        continue

                    det = Detection(
                        class_id=cls_id,
                        class_name=cls_name,
                        confidence=float(confs[i]),
                        bbox=tuple(boxes[i]),
                        track_id=int(ids[i]),
                    )
                    detections.append(det)

            frame, telemetry = self._process_frame_auto(frame, detections, fps)
            yield frame, telemetry

    # ─────────────────── Otonom Frame İşleme ───────────────────

    def _process_frame_auto(self, frame, detections, fps, depth_mat=None):
        """
        Otonom mod frame işleme:
        - Tüm tespitleri işler, derinlik geçmişini günceller
        - Yaklaşan hedefleri belirler
        - En kritik yaklaşan hedef için turret'i IBVS ile hizalar
        - Error_X, Error_Y UART paketini Arduino'ya gönderir

        Returns:
            (annotated_frame, telemetry)
        """
        h, w, _ = frame.shape
        cx_frame = w / 2.0
        cy_frame = h / 2.0
        now = time.time()

        # HUD
        fps_val = fps
        self.draw_fps_overlay(frame, fps_val, "STAGE 2 [AUTO]")
        cv2.putText(
            frame,
            "MODE: AUTO (Swarm Tracking)",
            (10, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
        )
        self.draw_crosshair(frame, color=(0, 0, 255))

        # Mevcut track'ler için meta ve derinlik özetleri
        approaching_targets = []

        # ─── Tüm tespitleri dolaş ───
        for det in detections:
            tid = det.track_id
            x1, y1, x2, y2 = det.bbox
            center_x, center_y = det.center
            bbox_height = y2 - y1

            # Derinlik (varsa ZED, yoksa bbox temelinde kaba tahmin)
            det_depth = None
            if depth_mat is not None and self.zed_camera is not None:
                det_depth = self.zed_camera.get_depth_at_bbox(
                    depth_mat, x1, y1, x2, y2
                )
            else:
                # ZED yoksa, sadece kaba bir 1/z ~ bbox_yüksekliği ilişkisi ile
                # yaklaşma trendini tahmin etmek için orantısal bir derinlik
                if bbox_height > 0:
                    det_depth = min(
                        self.max_engage_dist,
                        max(1.0, 2000.0 / float(bbox_height)),
                    )

            # Hedef meta ve derinlik geçmişi
            if tid not in self.depth_histories:
                self.depth_histories[tid] = collections.deque(maxlen=32)

            if tid not in self.target_meta:
                self.target_meta[tid] = {
                    "class_name": det.class_name,
                    "last_center": det.center,
                    "last_time": now,
                    "vz": 0.0,
                    "is_approaching": False,
                }

            meta = self.target_meta[tid]
            meta["class_name"] = det.class_name

            # Derinlik geçmişini güncelle
            if det_depth is not None and 0.0 < det_depth <= SystemConfig.ZED_MAX_DEPTH_M:
                hist = self.depth_histories[tid]
                hist.append((now, float(det_depth)))

                # Yaklaşma hızı (Z-ekseni) tahmini
                vz, is_approaching = self._estimate_approach(hist)
                meta["vz"] = vz
                meta["is_approaching"] = is_approaching
            else:
                meta["vz"] = 0.0
                meta["is_approaching"] = False

            # 2D hız tahmini (piksel)
            last_cx, last_cy = meta["last_center"]
            dt = max(1.0 / fps_val, now - meta["last_time"]) if fps_val > 0 else now - meta["last_time"]
            vx_pix = (center_x - last_cx) / dt if dt > 1e-3 else 0.0
            vy_pix = (center_y - last_cy) / dt if dt > 1e-3 else 0.0

            meta["last_center"] = (center_x, center_y)
            meta["last_time"] = now

            # Kutu çizimi (tehdit rengine göre)
            depth_str = f"{det_depth:.1f}m" if det_depth is not None else "?"
            vz_str = f"{meta['vz']:+.2f}m/s"

            label = (
                f"{det.class_name.upper()} ID:{tid} "
                f"{det.confidence:.0%} D:{depth_str} Vz:{vz_str}"
            )

            color = (0, 255, 255) if meta["is_approaching"] else (128, 128, 128)
            self.draw_target_box(frame, det.bbox, label, color)

            # Yaklaşan ve angajman bandında olan hedefleri topla
            if meta["is_approaching"] and det_depth is not None:
                if det_depth <= self.max_engage_dist:
                    approaching_targets.append(
                        {
                            "tid": tid,
                            "class_name": det.class_name,
                            "depth_m": det_depth,
                            "vz": meta["vz"],
                            "center": (center_x, center_y),
                            "bbox": det.bbox,
                            "vx_pix": vx_pix,
                            "vy_pix": vy_pix,
                        }
                    )

        # ─── En yüksek öncelikli yaklaşan hedefi seç ───
        primary = self._select_primary_target(approaching_targets, cx_frame, cy_frame)

        have_target = primary is not None
        engaged = False          # Mesafe + yaklaşma bazlı angajman kararı
        locked = False           # IBVS kilidi (BaseStageEngine'den)
        fire_cmd = False         # Arduino'ya gidecek nihai ateş komutu
        depth_m = None

        if primary is not None:
            depth_m = primary["depth_m"]

            # Kutu vurgulama
            depth_str = f"{depth_m:.1f}m"
            primary_label = (
                f"PRIMARY {primary['class_name'].upper()} D:{depth_str} "
                f"Vz:{primary['vz']:+.2f}m/s"
            )
            self.draw_target_box(
                frame,
                primary["bbox"],
                primary_label,
                (0, 255, 0),
                thickness=3,
            )

            # Merkeze göre piksel hatası
            tx, ty = primary["center"]
            error_x = tx - cx_frame
            error_y = ty - cy_frame

            # Normalize hata ([-1, 1])
            error_x_norm = error_x / (w / 2.0)
            error_y_norm = error_y / (h / 2.0)

            if self.auto_tracking_enabled:
                # IBVS tabanlı turret güncellemesi
                dt = 1.0 / fps_val if fps_val > 0 else 0.033
                locked, est_depth = self.update_turret_ibvs(
                    target_x=tx,
                    target_y=ty,
                    vx_pix=primary["vx_pix"],
                    vy_pix=primary["vy_pix"],
                    bbox_height=primary["bbox"][3] - primary["bbox"][1],
                    frame_width=w,
                    frame_height=h,
                    dt=dt,
                )

                # UART: Error_X, Error_Y paketi (PID setpoint için)
                try:
                    if hasattr(self.arduino, "send_error_packet"):
                        self.arduino.send_error_packet(error_x_norm, error_y_norm)
                except Exception:
                    pass

                # Angajman kuralı: kritik mesafeyi aşmadan ÖNCE tetikle
                engaged = self._should_engage(depth_m, primary["vz"])

                # FireController ile Stage 2 ateş kararı
                decision = self.fire_controller.update_stage2(
                    tid=primary["tid"],
                    locked_now=locked,
                    depth_m=depth_m,
                    vz=primary["vz"],
                    have_target=True,
                    now=now,
                    engage_ok=engaged,
                )
                fire_cmd = decision.should_fire

                # Angajman sinyali (örneğin lazer/ateş) – karar Python veya Arduino tarafında olabilir
                try:
                    if hasattr(self.arduino, "send"):
                        # Bu çağrı non-blocking; sadece mevcut açıları gönderir
                        self.arduino.send(fire=fire_cmd)
                except Exception:
                    pass
            else:
                engaged = False
                locked = False
                fire_cmd = False
                self.fire_controller.reset()

            # Angajman durumunu HUD'a yaz
            engage_text = "ENGAGE" if fire_cmd else "TRACKING"
            engage_color = (0, 0, 255) if fire_cmd else (0, 255, 255)
            cv2.putText(
                frame,
                f"{engage_text} | D:{depth_m:.1f}m Vz:{primary['vz']:+.2f}m/s",
                (10, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                engage_color,
                2,
            )

        # ─── Telemetri ───
        telemetry = {
            "fps": fps_val,
            "have_target": have_target,
            "engaged": fire_cmd,
            "locked": locked,
            "lock_duration_s": getattr(
                self.fire_controller, "current_lock_duration_s", 0.0
            ),
            "target_depth_m": depth_m,
            "detection_count": len(detections),
            "approaching_count": len(approaching_targets),
            "camera_mode": "ZED 2" if self.use_zed else "OpenCV",
        }

        if primary is not None:
            telemetry.update(
                {
                    "target_id": primary["tid"],
                    "target_class": primary["class_name"],
                    "target_vz_mps": primary["vz"],
                }
            )
        else:
            telemetry.update(
                {
                    "target_id": None,
                    "target_class": None,
                    "target_vz_mps": None,
                }
            )

        return frame, telemetry

    # ─────────────────── Yardımcı Metodlar ───────────────────

    def _estimate_approach(self, hist: DepthHistory) -> Tuple[float, bool]:
        """
        Derinlik geçmişinden yaklaşıp yaklaşmadığını ve ortalama Z-hızını hesapla.

        Returns:
            (vz_mps, is_approaching)
        """
        if len(hist) < 2:
            return 0.0, False

        # Zaman penceresi içinde kalan örnekleri kullan
        t_now = hist[-1][0]
        window_start = t_now - self.approach_window

        valid = [item for item in hist if item[0] >= window_start]
        if len(valid) < 2:
            return 0.0, False

        t0, d0 = valid[0]
        t1, d1 = valid[-1]
        dt = t1 - t0
        if dt <= 1e-3:
            return 0.0, False

        # Pozitif Vz = hedef yaklaşıyor (mesafe azalıyor)
        vz = (d0 - d1) / dt
        is_approaching = vz > self.min_vz
        return vz, is_approaching

    def _select_primary_target(self, targets, cx: float, cy: float):
        """
        Yaklaşan hedefler arasından en kritik olanı seç.

        Öncelik kriterleri:
          1) Sınıf önceliği (ballistic_missile > uav > mini/micro)
          2) Kritiğe kalan süre (time-to-min-distance)
          3) Görüntü merkezine olan uzaklık (daha merkezde olan)
        """
        if not targets:
            return None

        best = None
        best_score = None

        for t in targets:
            cls = t["class_name"].lower()
            depth = t["depth_m"]
            vz = t["vz"]

            # Sınıf önceliği
            base_prio = self.class_priority.get(cls, 0)

            # Kritiğe kalan süre (saniye)
            time_to_min = self._time_to_min_distance(depth, vz)

            # Merkeze uzaklık (piksel)
            tx, ty = t["center"]
            dist2 = (tx - cx) ** 2 + (ty - cy) ** 2

            # Skor: daha yüksek daha iyi
            # - base_prio: büyük olsun
            # - time_to_min: küçük olsun (yakında çarpışacak)
            # - dist2: küçük olsun (merkeze yakın)
            score = (
                base_prio * 1000.0
                + max(0.0, 5.0 - time_to_min) * 100.0
                + max(0.0, 1.0 - dist2 / (cx * cx + cy * cy)) * 10.0
            )

            if (best is None) or (score > best_score):
                best = t
                best_score = score

        return best

    def _time_to_min_distance(self, depth: float, vz: float) -> float:
        """
        Mevcut derinlik ve yaklaşma hızı ile minimum mesafeye kalan süreyi tahmin et.
        Negatif/sonsuz durumlarda büyük bir değer döndürür.
        """
        if vz <= 1e-3:
            return 1e6

        target_dist = max(self.min_dist, 0.1)
        if depth <= target_dist:
            return 0.0

        return (depth - target_dist) / vz

    def _should_engage(self, depth: float, vz: float) -> bool:
        """
        Angajman kuralı:
        - Hedef gerçekten yaklaşıyor olmalı (vz > min_vz)
        - Hedef kritik mesafe + güvenlik payına yaklaşmış olmalı
          VEYA kısa süre içinde bu banda girecek olmalı.
        """
        if depth is None or depth <= 0.0:
            return False

        if vz <= self.min_vz:
            return False

        engage_band = self.min_dist + self.safety_margin

        # Şimdiden angajman bandında ise
        if depth <= engage_band:
            return True

        # Yakında bandın içine girecekse (öngörü)
        t_to_min = self._time_to_min_distance(depth, vz)
        return t_to_min < 1.0  # 1 saniyeden kısa sürede kritik banda girecekse
