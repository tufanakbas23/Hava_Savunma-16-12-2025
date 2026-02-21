"""
Stage 2: Dost/Düşman Ayrımı (IBVS ile)
Belgede: Bölüm 4 - Tüm katmanlar entegre
"""

import cv2
import numpy as np
from collections import Counter
from stages.base_stage import BaseStageEngine
from control.kalman_tracker import TargetKalman
from classify.lab_color import LabColorBalloon

# Sabitler
INTERVAL_LEARNING = 1
INTERVAL_CONFIRMED = 30
INTERVAL_STABLE = 60
POSITION_JUMP_THRESHOLD = 150
CONFIDENCE_FRAMES = 5
CONFIDENCE_THRESHOLD = 3
SHADOW_L_THRESHOLD = 90
ENEMY_COLORS = ["red", "kirmizi"]
FRIEND_COLORS = ["blue", "mavi"]


class Stage2Engine(BaseStageEngine):
    def __init__(self, model_path, arduino, camera_index=0):
        super().__init__(model_path, arduino, camera_index)

        # Renk sınıflandırıcı
        self.color_classifier = LabColorBalloon()

        # Stage2 state
        self.auto_tracking_enabled = True
        self.last_fire_time = None
        self.FIRE_COOLDOWN = 0.5

        print("[Stage2] IBVS + Renk sınıflandırma aktif")

    def confirm_target_color(self, target, color, frame_counter):
        """Hedefin rengini onaylama mantığı"""
        if not target["confirmed"]:
            target["color_history"].append(color)
            if len(target["color_history"]) >= CONFIDENCE_FRAMES:
                counts = Counter(target["color_history"])
                most_common, count = counts.most_common(1)[0]
                if count >= CONFIDENCE_THRESHOLD and most_common != "unknown":
                    target["confirmed"] = True
                    target["color"] = most_common
            if len(target["color_history"]) > CONFIDENCE_FRAMES:
                target["color_history"].pop(0)
        else:
            # Renk değişimi kontrolü
            if color != target["color"] and color != "unknown":
                target["confirmed"] = False
                target["color_history"] = [color]
                target["stable_count"] = 0
            else:
                target["stable_count"] += 1

    def run(self):
        """Generator: her frame için (frame, telemetry) döndürür."""
        results = self.get_frame_generator()

        for r in results:
            if self.should_stop:
                break

            frame = r.orig_img.copy()
            fps = self.calculate_fps()
            now = self.prev_time
            h, w, _ = frame.shape
            cx_frame = w / 2.0
            cy_frame = h / 2.0

            # ===== OVERLAY =====
            self.draw_fps_overlay(frame, fps, "STAGE 2 [IBVS]")
            cv2.putText(
                frame,
                "MODE: AUTO",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
            )
            self.draw_crosshair(frame, size=10)

            # ===== HEDEF SEÇİMİ =====
            best_enemy = None
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

                    # YOLO sınıf adından renk çıkar (kirmizi_balon → kirmizi)
                    yolo_color = "unknown"
                    if "kirmizi" in detected_class:
                        yolo_color = "kirmizi"
                    elif "mavi" in detected_class:
                        yolo_color = "mavi"

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
                        # YOLO sınıf adından renk varsa onu kullan, yoksa LAB
                        if yolo_color != "unknown":
                            color = yolo_color
                            confirmed = True  # YOLO rengi güvenilir
                        else:
                            color = self.color_classifier.classify(frame, box)
                            confirmed = False
                        kalman = TargetKalman(center_x, center_y)
                        self.tracked_targets[tid] = {
                            "kalman": kalman,
                            "size": size,
                            "bbox_height": bbox_height,
                            "color": color,
                            "color_history": [color],
                            "confirmed": confirmed,
                            "last_check": self.frame_counter,
                            "last_position": (center_x, center_y),
                            "stable_count": 0,
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
                        color = self.color_classifier.classify(frame, box)
                        target["color"] = color
                        target["color_history"] = [color]
                        target["confirmed"] = False
                        target["stable_count"] = 0
                        target["last_check"] = self.frame_counter
                        target["last_position"] = (center_x, center_y)
                        continue

                    target["last_position"] = filt_pos

                    # Renk onay mantığı
                    if not target["confirmed"]:
                        interval = INTERVAL_LEARNING
                    elif target["stable_count"] > 100:
                        interval = INTERVAL_STABLE
                    else:
                        interval = INTERVAL_CONFIRMED

                    frames_since = self.frame_counter - target["last_check"]
                    if frames_since >= interval:
                        color = self.color_classifier.classify(frame, box)
                        self.confirm_target_color(target, color, self.frame_counter)
                        target["last_check"] = self.frame_counter

                    final_color = target.get("color", "unknown")
                    confirmed = target["confirmed"]
                    is_enemy = confirmed and final_color in ENEMY_COLORS
                    is_friend = confirmed and final_color in FRIEND_COLORS

                    # Kutu rengi
                    if is_enemy:
                        box_color = (0, 0, 255)
                        thickness = 4
                    elif is_friend:
                        box_color = (255, 0, 0)
                        thickness = 2
                    else:
                        box_color = (128, 128, 128)
                        thickness = 2

                    # Label
                    label = f"ID:{tid} {size}"
                    if confirmed:
                        label += f" [{final_color.upper()}]"
                    else:
                        label += " [?]"

                    if is_enemy:
                        label += " [ENEMY]"
                    elif is_friend:
                        label += " [FRIEND]"

                    speed = np.sqrt(filt_vel[0] ** 2 + filt_vel[1] ** 2)
                    label += f" v={speed:.1f}"

                    self.draw_target_box(frame, box, label, box_color, thickness)

                    # PRIMARY düşman seç
                    if is_enemy:
                        fx, fy = filt_pos
                        error_x = fx - cx_frame
                        error_y = fy - cy_frame
                        dist2 = error_x**2 + error_y**2
                        size_priority = 0 if size == "small" else 1
                        score = (size_priority, dist2)

                        if (best_enemy is None) or (score < best_enemy["score"]):
                            best_enemy = {
                                "score": score,
                                "tid": tid,
                                "filt_pos": filt_pos,
                                "filt_vel": filt_vel,
                                "box": box,
                                "size": size,
                                "color": final_color,
                                "bbox_height": bbox_height,
                            }

            # ===== PRIMARY DÜŞMAN VURGULA =====
            locked = False
            fired = False
            depth_m = 5.0

            if best_enemy is not None:
                have_target = True
                fx, fy = best_enemy["filt_pos"]
                vx_pix, vy_pix = best_enemy["filt_vel"]
                bbox_height = best_enemy["bbox_height"]

                self.draw_target_box(
                    frame,
                    best_enemy["box"],
                    f"PRIMARY ENEMY {best_enemy['size']}",
                    (0, 255, 0),
                    thickness=3,
                )

                # ===== OTONOM IBVS KONTROL =====
                if self.auto_tracking_enabled:
                    dt = 1.0 / fps if fps > 0 else 0.033
                    locked, depth_m = self.update_turret_ibvs(
                        fx, fy, vx_pix, vy_pix, bbox_height, w, h, dt
                    )

                    can_fire = locked and (
                        self.last_fire_time is None
                        or now - self.last_fire_time > self.FIRE_COOLDOWN
                    )

                    if can_fire:
                        self.arduino.send(fire=True)
                        self.last_fire_time = now
                        fired = True
                    else:
                        self.arduino.send(fire=False)
                else:
                    self.arduino.send(fire=False)

                if locked:
                    cv2.putText(
                        frame,
                        "LOCK",
                        (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2,
                    )
            else:
                self.arduino.send(fire=False)

            # Debug info
            if have_target:
                info_text = f"Depth: {depth_m:.1f}m | Lock: {self.lock_frame_counter}/{self.LOCK_DURATION_FRAMES}"
                cv2.putText(
                    frame,
                    info_text,
                    (10, 120),
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
                "auto_enabled": self.auto_tracking_enabled,
                "target_id": best_enemy["tid"] if best_enemy else None,
                "target_size": best_enemy["size"] if best_enemy else None,
                "target_color": best_enemy["color"] if best_enemy else None,
                "fired": fired,
            }

            yield frame, telemetry
