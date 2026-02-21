"""
Stage 1: Hareketli Hedef Takibi (IBVS ile)
Belgede: Bölüm 4 - Tüm katmanlar entegre
"""

import cv2
import numpy as np
from stages.base_stage import BaseStageEngine
from control.kalman_tracker import TargetKalman

POSITION_JUMP_THRESHOLD = 150
SHADOW_L_THRESHOLD = 90


class Stage1Engine(BaseStageEngine):
    def __init__(self, model_path, arduino, camera_index=0):
        super().__init__(model_path, arduino, camera_index)

        # Stage1 state
        self.tracking_enabled = False
        self.fire_requested = False
        self.cycle_armed = True

        print("[Stage1] IBVS kontrol sistemi aktif")

    def run(self):
        """Generator: her frame için (frame, telemetry) döndürür."""
        results = self.get_frame_generator()

        for r in results:
            if self.should_stop:
                break

            frame = r.orig_img.copy()
            fps = self.calculate_fps()
            h, w, _ = frame.shape
            cx_frame = w / 2.0
            cy_frame = h / 2.0

            # ===== OVERLAY =====
            self.draw_fps_overlay(frame, fps, "STAGE 1 [IBVS]")
            status_color = (0, 255, 0) if self.tracking_enabled else (0, 0, 255)
            status_text = "TRACK: ON" if self.tracking_enabled else "TRACK: OFF"
            cv2.putText(
                frame,
                status_text,
                (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                status_color,
                2,
            )

            cycle_text = "ARMED" if self.cycle_armed else "WAIT"
            cycle_color = (0, 255, 0) if self.cycle_armed else (255, 0, 0)
            cv2.putText(
                frame,
                f"CYCLE: {cycle_text}",
                (10, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                cycle_color,
                2,
            )

            self.draw_crosshair(frame)

            # ===== HEDEF SEÇİMİ =====
            best_candidate = None
            have_target = False

            if r.boxes is not None and r.boxes.id is not None:
                ids = r.boxes.id.cpu().numpy().astype(int)
                boxes = r.boxes.xyxy.cpu().numpy()
                classes = r.boxes.cls.cpu().numpy().astype(int)
                class_names = r.names

                for i, box in enumerate(boxes):
                    x1, y1, x2, y2 = box
                    tid = ids[i]
                    detected_class = class_names[classes[i]]

                    if "balon" not in detected_class:
                        continue

                    center_x = (x1 + x2) / 2.0
                    center_y = (y1 + y2) / 2.0
                    bbox_height = y2 - y1

                    # Gölge filtresi
                    crop = frame[
                        max(0, int(y1)) : min(h, int(y2)),
                        max(0, int(x1)) : min(w, int(x2)),
                    ]
                    if self.filter_shadow(crop, SHADOW_L_THRESHOLD):
                        continue

                    size = self.size_classifier.classify(box)

                    # Yeni hedef
                    if tid not in self.tracked_targets:
                        kalman = TargetKalman(center_x, center_y)
                        self.tracked_targets[tid] = {
                            "kalman": kalman,
                            "size": size,
                            "bbox_height": bbox_height,
                            "last_position": (center_x, center_y),
                        }
                        continue

                    # Kalman güncelleme
                    target = self.tracked_targets[tid]
                    dt = 1.0 / fps if fps > 0 else 0.033
                    target["kalman"].predict(dt)
                    filt_pos, filt_vel = target["kalman"].correct(center_x, center_y)
                    target["size"] = size
                    target["bbox_height"] = bbox_height

                    # Jump kontrolü
                    last_x, last_y = target["last_position"]
                    pos_distance = np.sqrt(
                        (center_x - last_x) ** 2 + (center_y - last_y) ** 2
                    )

                    if pos_distance > POSITION_JUMP_THRESHOLD:
                        target["last_position"] = (center_x, center_y)
                        cv2.rectangle(
                            frame,
                            (int(x1), int(y1)),
                            (int(x2), int(y2)),
                            (0, 255, 255),
                            2,
                        )
                        continue

                    target["last_position"] = filt_pos

                    # Box çiz
                    speed = np.sqrt(filt_vel[0] ** 2 + filt_vel[1] ** 2)
                    self.draw_target_box(
                        frame, box, f"ID:{tid} {size} v={speed:.1f}", (128, 128, 128)
                    )

                    # En iyi adayı seç (önce small, sonra merkeze yakın)
                    fx, fy = filt_pos
                    error_x = fx - cx_frame
                    error_y = fy - cy_frame
                    dist2 = error_x**2 + error_y**2
                    size_priority = 0 if size == "small" else 1
                    score = (size_priority, dist2)

                    if (best_candidate is None) or (score < best_candidate["score"]):
                        best_candidate = {
                            "score": score,
                            "tid": tid,
                            "filt_pos": filt_pos,
                            "filt_vel": filt_vel,
                            "box": box,
                            "size": size,
                            "bbox_height": bbox_height,
                        }

            # ===== PRIMARY HEDEF =====
            locked = False
            depth_m = 5.0

            if best_candidate is not None:
                have_target = True
                fx, fy = best_candidate["filt_pos"]
                vx_pix, vy_pix = best_candidate["filt_vel"]
                bbox_height = best_candidate["bbox_height"]

                self.draw_target_box(
                    frame,
                    best_candidate["box"],
                    f"PRIMARY {best_candidate['size']}",
                    (0, 255, 0),
                    thickness=4,
                )

                # ===== IBVS KONTROL =====
                if self.tracking_enabled and self.cycle_armed:
                    dt = 1.0 / fps if fps > 0 else 0.033
                    locked, depth_m = self.update_turret_ibvs(
                        fx, fy, vx_pix, vy_pix, bbox_height, w, h, dt
                    )

                    laser_on = bool(self.fire_requested and locked)

                    if laser_on:
                        self.cycle_armed = False
                        self.tracking_enabled = False
                        self.fire_requested = False

                    if locked:
                        cv2.putText(
                            frame,
                            "LOCKED",
                            (int(cx_frame) - 40, int(cy_frame) - 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 0),
                            2,
                        )

                    self.arduino.send(fire=laser_on)
                else:
                    self.arduino.send(fire=False)
            else:
                self.arduino.send(fire=False)

            # Debug info
            if have_target:
                info_text = f"Depth: {depth_m:.1f}m | Lock: {self.lock_frame_counter}/{self.LOCK_DURATION_FRAMES}"
                cv2.putText(
                    frame,
                    info_text,
                    (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    1,
                )

            # ===== TELEMETRI =====
            telemetry = {
                "fps": fps,
                "locked": locked,
                "have_target": have_target,
                "tracking": self.tracking_enabled,
                "cycle_armed": self.cycle_armed,
                "target_id": best_candidate["tid"] if best_candidate else None,
                "target_size": best_candidate["size"] if best_candidate else None,
            }

            yield frame, telemetry
