"""
Stage 1: Manuel Mod — Algılama + UI Veri Beslemesi
Phase 1.1-1.3: ZED 2 + YOLO 5-sınıf algılama + depth + UI overlay

Stage 1 TAMAMEN MANUEL moddur.
Görev: YOLO ile nesne algılama, ZED ile mesafe ölçümü yaparak
kullanıcıya zengin durumsal farkındalık (situational awareness) sağlar.
Kullanıcı joystick ile manuel hizalama yapar.

Mimari: Python (Brain) → UART → C++ (Muscles)
"""

import cv2
import numpy as np
from stages.base_stage import BaseStageEngine
from control.kalman_tracker import TargetKalman
from config.system_config import SystemConfig

POSITION_JUMP_THRESHOLD = 150
SHADOW_L_THRESHOLD = 90


class Stage1Engine(BaseStageEngine):
    """
    Stage 1: Manuel Algılama Motoru.

    - YOLO ile nesne algılama (5 sınıf)
    - ZED ile bounding box merkezinde mesafe ölçümü
    - Kalman filtre ile yumuşak pozisyon/hız tahmini
    - Tüm bilgileri frame üzerine çizerek UI'ye iletir
    - Otonom takip veya ateş kontrolü YOKTUR (Stage 2+'da)
    """

    def __init__(self, model_path, arduino, camera_index=0):
        super().__init__(model_path, arduino, camera_index)

        cam_src = "ZED 2" if self.use_zed else "OpenCV"
        print(f"[Stage1] Manuel algılama motoru aktif | Kamera: {cam_src}")

    def run(self):
        """
        Generator: her frame için (frame, telemetry) döndürür.
        ZED kamera aktifse frame-by-frame mod, değilse OpenCV stream modu.
        """
        if self.use_zed:
            yield from self._run_zed()
        else:
            yield from self._run_opencv()

    # ─────────────────── ZED Frame-by-Frame Modu ───────────────────

    def _run_zed(self):
        """ZED kamera + detect_frame() ile çalışır."""
        try:
            self.open_zed_camera()

            while not self.should_stop:
                frame, depth = self.grab_zed_frame()
                if frame is None:
                    continue

                fps = self.calculate_fps()
                detections = self.detector.detect_frame(frame, use_tracking=True)

                frame, telemetry = self._process_frame(
                    frame, detections, fps, depth_mat=depth
                )
                yield frame, telemetry

        finally:
            self.close_zed_camera()

    # ─────────────────── OpenCV Stream Modu ───────────────────

    def _run_opencv(self):
        """OpenCV stream + track_video() ile çalışır (geriye uyumluluk)."""
        results = self.get_frame_generator()

        for r in results:
            if self.should_stop:
                break

            frame = r.orig_img.copy()
            fps = self.calculate_fps()
            detections = self._convert_yolo_results(r)

            frame, telemetry = self._process_frame(frame, detections, fps)
            yield frame, telemetry

    def _convert_yolo_results(self, r):
        """Ultralytics Result → Detection listesi dönüştürücü."""
        from detector.yolo_detector import Detection

        detections = []
        if r.boxes is None or r.boxes.id is None:
            return detections

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

        return detections

    # ─────────────────── Ana Frame İşleme ───────────────────

    def _process_frame(self, frame, detections, fps, depth_mat=None):
        """
        Manuel mod frame işleme:
        Tüm algılamaları çizer, en yakın hedefi vurgular,
        otonom kontrol YAPMAZ.

        Returns:
            (annotated_frame, telemetry)
        """
        h, w, _ = frame.shape
        cx_frame = w / 2.0
        cy_frame = h / 2.0

        # ===== HUD OVERLAY =====
        self.draw_fps_overlay(frame, fps, "STAGE 1 [MANUEL]")

        cv2.putText(
            frame, "MODE: MANUAL (Joystick)",
            (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
            (0, 200, 255), 2,
        )

        self.draw_crosshair(frame)

        # ===== TÜM ALGILAMALARI İŞLE =====
        closest_target = None

        for det in detections:
            tid = det.track_id
            x1, y1, x2, y2 = det.bbox
            center_x, center_y = det.center
            bbox_height = y2 - y1

            # ─── ZED Depth ───
            det_depth = None
            if depth_mat is not None and self.zed_camera is not None:
                det_depth = self.zed_camera.get_depth_at_bbox(
                    depth_mat, x1, y1, x2, y2
                )

            # Gölge filtresi
            crop = frame[
                max(0, int(y1)) : min(h, int(y2)),
                max(0, int(x1)) : min(w, int(x2)),
            ]
            if self.filter_shadow(crop, SHADOW_L_THRESHOLD):
                continue

            # ─── Kalman Takip (pozisyon yumuşatma) ───
            if tid not in self.tracked_targets:
                kalman = TargetKalman(center_x, center_y)
                self.tracked_targets[tid] = {
                    "kalman": kalman,
                    "bbox_height": bbox_height,
                    "last_position": (center_x, center_y),
                    "class_name": det.class_name,
                    "depth_m": det_depth,
                }
                continue

            target = self.tracked_targets[tid]
            dt = 1.0 / fps if fps > 0 else 0.033
            target["kalman"].predict(dt)
            filt_pos, filt_vel = target["kalman"].correct(center_x, center_y)
            target["bbox_height"] = bbox_height
            target["class_name"] = det.class_name
            target["depth_m"] = det_depth

            # Jump kontrolü
            last_x, last_y = target["last_position"]
            pos_distance = np.sqrt(
                (center_x - last_x) ** 2 + (center_y - last_y) ** 2
            )
            if pos_distance > POSITION_JUMP_THRESHOLD:
                target["last_position"] = (center_x, center_y)
                continue

            target["last_position"] = filt_pos

            # ─── Box Çizimi — Sınıf + Güven + Mesafe ───
            speed = np.sqrt(filt_vel[0] ** 2 + filt_vel[1] ** 2)
            depth_str = f"{det_depth:.1f}m" if det_depth is not None else "?"
            label = (
                f"{det.class_name.upper()} ID:{tid} "
                f"{det.confidence:.0%} D:{depth_str}"
            )
            self.draw_target_box(frame, det.bbox, label, (128, 128, 128))

            # ─── En yakın hedefi seç (merkeze en yakın) ───
            fx, fy = filt_pos
            dist2 = (fx - cx_frame) ** 2 + (fy - cy_frame) ** 2

            if closest_target is None or dist2 < closest_target["dist2"]:
                closest_target = {
                    "dist2": dist2,
                    "tid": tid,
                    "filt_pos": filt_pos,
                    "filt_vel": filt_vel,
                    "box": det.bbox,
                    "bbox_height": bbox_height,
                    "class_name": det.class_name,
                    "confidence": det.confidence,
                    "depth_m": det_depth,
                }

        # ===== EN YAKIN HEDEF VURGULAMA =====
        have_target = closest_target is not None
        depth_m = None

        if closest_target is not None:
            depth_m = closest_target["depth_m"]

            depth_str = f"{depth_m:.1f}m" if depth_m is not None else "?"
            primary_label = (
                f"NEAREST {closest_target['class_name'].upper()} "
                f"{closest_target['confidence']:.0%} D:{depth_str}"
            )
            self.draw_target_box(
                frame,
                closest_target["box"],
                primary_label,
                (0, 255, 0),
                thickness=3,
            )

        # ===== ALT BİLGİ ÇİZGİSİ =====
        if have_target:
            depth_display = f"{depth_m:.2f}m" if depth_m is not None else "N/A"
            info_text = (
                f"ZED Depth: {depth_display} | "
                f"Targets: {len(detections)}"
            )
            cv2.putText(
                frame, info_text,
                (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 0), 1,
            )

        # ===== TELEMETRI =====
        telemetry = {
            "fps": fps,
            "have_target": have_target,
            "target_id": closest_target["tid"] if closest_target else None,
            "target_class": closest_target["class_name"] if closest_target else None,
            "target_depth_m": depth_m,
            "detection_count": len(detections),
            "camera_mode": "ZED 2" if self.use_zed else "OpenCV",
        }

        return frame, telemetry
