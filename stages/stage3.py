"""
Stage 3: Angajman Sistemi (IBVS ile)
Belgede: Bölüm 4 - Tüm katmanlar entegre
"""

import time
import cv2
import numpy as np
from stages.base_stage import BaseStageEngine
from classify.sticker_lab_color import StickerLabClassifier
from control.kalman_tracker import TargetKalman

# Angajman ROI
ENGAGE_ROI = (100, 50, 400, 200)

# Platform açıları (Belgede: Bölüm 6.2)
PLATFORM_PAN_LEFT_DEG = -30.0
PLATFORM_PAN_RIGHT_DEG = +30.0
PLATFORM_TILT_DEG = 0.0

# Tracking parametreleri
STABLE_FRAMES_REQUIRED = 10
FIRE_DURATION_FRAMES = 20


class Stage3Engine(BaseStageEngine):
    def __init__(self, model_path, arduino=None, camera_index=0):
        super().__init__(model_path, arduino, camera_index)

        # Sticker renk sınıflandırıcı
        self.color_classifier = StickerLabClassifier()

        # State machine
        self.state = "IDLE"

        # Angajman bilgisi
        self.engagement = {
            "letter": None,
            "direction": None,
            "shape": None,
            "color": None,
        }

        # Tracking
        self.kalman = None
        self.target_id = None
        self.stable_frame_count = 0
        self.fire_frame_count = 0

        # OCR
        self.ocr_pending = False
        self.last_ocr_time = 0.0
        self.ocr_period = 0.5

        print("[Stage3] IBVS + Angajman sistemi aktif")

    def start_cycle(self):
        """Tek butonla döngüyü başlat"""
        if self.state == "IDLE":
            self.state = "READING"
            self._reset_engagement()

    def _reset_engagement(self):
        """Angajman bilgilerini sıfırla"""
        self.engagement = {
            "letter": None,
            "direction": None,
            "shape": None,
            "color": None,
        }
        self.kalman = None
        self.target_id = None
        self.stable_frame_count = 0
        self.fire_frame_count = 0
        self.ocr_pending = False

    def _is_engagement_complete(self):
        """Angajman okuma tamamlandı mı?"""
        return (
            self.engagement["letter"] is not None
            and self.engagement["shape"] is not None
            and self.engagement["color"] is not None
        )

    def _turn_to_platform(self):
        if not self.arduino:
            return

        target_pan = (
            PLATFORM_PAN_LEFT_DEG
            if self.engagement["direction"] == "LEFT"
            else PLATFORM_PAN_RIGHT_DEG
        )
        self.arduino.pan_deg = target_pan
        self.arduino.tilt_deg = PLATFORM_TILT_DEG
        self.arduino.send(fire=False)
        time.sleep(1.0)

    def submit_ocr_result(self, letter: str, conf: float):
        """OCR thread'den sonuç gelince"""
        if self.state == "READING" and self.engagement["letter"] is None:
            if letter in ("A", "B"):
                self.engagement["letter"] = letter
                self.engagement["direction"] = "LEFT" if letter == "A" else "RIGHT"
                self.ocr_pending = False

    def run(self):
        """Ana döngü"""
        results = self.get_frame_generator()

        for r in results:
            if self.should_stop:
                break

            frame = r.orig_img.copy()
            fps = self.calculate_fps()
            now = time.time()
            h, w, _ = frame.shape

            # Overlay
            self.draw_fps_overlay(frame, fps, f"STAGE 3 [IBVS] | {self.state}")

            # Angajman ROI çiz
            x1, y1, x2, y2 = ENGAGE_ROI
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.circle(frame, (w // 2, h // 2), 5, (0, 255, 255), -1)

            ocr_roi = None
            locked = False
            depth_m = 5.0

            # ===== STATE MACHINE =====
            if self.state == "READING":
                # Angajman bilgilerini oku
                if not self._is_engagement_complete():
                    # Harf oku (OCR ile)
                    if self.engagement["letter"] is None:
                        if not self.ocr_pending and (
                            now - self.last_ocr_time >= self.ocr_period
                        ):
                            roi = frame[y1:y2, x1:x2]
                            if roi.size > 0:
                                ocr_roi = roi.copy()
                                self.ocr_pending = True
                                self.last_ocr_time = now

                    # Şekil + renk oku (YOLO + LAB)
                    if r.boxes is not None and len(r.boxes) > 0:
                        boxes = r.boxes.xyxy.cpu().numpy()
                        classes = r.boxes.cls.cpu().numpy().astype(int)
                        class_names = r.names

                        for i, box in enumerate(boxes):
                            x1b, y1b, x2b, y2b = box
                            detected_class = class_names[int(classes[i])]
                            cx_box = (x1b + x2b) / 2
                            cy_box = (y1b + y2b) / 2

                            # Şekil ROI içindeyse
                            if detected_class in ["kare", "ucgen", "daire"]:
                                if x1 <= cx_box <= x2 and y1 <= cy_box <= y2:
                                    if self.engagement["shape"] is None:
                                        self.engagement["shape"] = detected_class

                                    if self.engagement["color"] is None:
                                        color = self.color_classifier.classify(
                                            frame, box
                                        )
                                        if color != "unknown":
                                            self.engagement["color"] = color
                else:
                    # Tüm bilgiler tamam, platforma dön
                    self.state = "TURNING"
                    self._turn_to_platform()

            elif self.state == "TURNING":
                # Dönüş tamamlandı
                self.state = "SEARCHING"

            elif self.state == "SEARCHING":
                # Angajmana uygun hedef ara
                if r.boxes is not None and len(r.boxes) > 0:
                    ids = (
                        r.boxes.id.cpu().numpy().astype(int)
                        if r.boxes.id is not None
                        else None
                    )
                    boxes = r.boxes.xyxy.cpu().numpy()
                    classes = r.boxes.cls.cpu().numpy().astype(int)
                    class_names = r.names

                    target_shape = f"balon_{self.engagement['shape']}"
                    target_color = self.engagement["color"]

                    candidates = []
                    for i, box in enumerate(boxes):
                        detected_class = class_names[int(classes[i])]
                        if not detected_class.startswith("balon"):
                            continue

                        size = self.size_classifier.classify(box)
                        color = self.color_classifier.classify(frame, box)

                        if (
                            detected_class == target_shape
                            and color == target_color
                            and size == "small"
                        ):
                            tid = int(ids[i]) if ids is not None else i
                            candidates.append(
                                {
                                    "tid": tid,
                                    "box": box,
                                    "class": detected_class,
                                    "color": color,
                                }
                            )

                    # Merkeze en yakın adayı seç
                    if candidates:

                        def dist_to_center(item):
                            x1b, y1b, x2b, y2b = item["box"]
                            cx = (x1b + x2b) / 2
                            cy = (y1b + y2b) / 2
                            return (cx - w / 2) ** 2 + (cy - h / 2) ** 2

                        best = min(candidates, key=dist_to_center)
                        self.target_id = best["tid"]

                        # Kalman başlat
                        x1b, y1b, x2b, y2b = best["box"]
                        init_x = (x1b + x2b) / 2
                        init_y = (y1b + y2b) / 2
                        self.kalman = TargetKalman(init_x, init_y)
                        self.state = "TRACKING"
                        self.stable_frame_count = 0

            elif self.state == "TRACKING":
                # Hedefi takip et
                found = False
                if r.boxes is not None and r.boxes.id is not None:
                    ids = r.boxes.id.cpu().numpy().astype(int)
                    boxes = r.boxes.xyxy.cpu().numpy()

                    for i, tid in enumerate(ids):
                        if tid == self.target_id:
                            found = True
                            box = boxes[i]
                            x1b, y1b, x2b, y2b = box
                            raw_cx = (x1b + x2b) / 2
                            raw_cy = (y1b + y2b) / 2
                            bbox_height = y2b - y1b

                            # Kalman güncelle
                            dt = 1.0 / fps if fps > 0 else 0.033
                            self.kalman.predict(dt)
                            (smooth_cx, smooth_cy), (vx, vy) = self.kalman.correct(
                                raw_cx, raw_cy
                            )

                            # ===== IBVS KONTROL =====
                            locked, depth_m = self.update_turret_ibvs(
                                smooth_cx, smooth_cy, vx, vy, bbox_height, w, h, dt
                            )

                            # Lock kontrolü
                            if locked:
                                self.stable_frame_count += 1
                                if self.stable_frame_count >= STABLE_FRAMES_REQUIRED:
                                    self.state = "FIRING"
                                    self.fire_frame_count = 0
                            else:
                                self.stable_frame_count = 0

                            # Hedef box çiz
                            cv2.rectangle(
                                frame,
                                (int(x1b), int(y1b)),
                                (int(x2b), int(y2b)),
                                (0, 255, 0),
                                4,
                            )
                            break

                if not found:
                    # Hedef kayboldu
                    self.state = "SEARCHING"
                    self.kalman = None

            elif self.state == "FIRING":
                # Ateş et
                self.arduino.send(fire=True)
                self.fire_frame_count += 1
                cv2.putText(
                    frame,
                    "FIRING!",
                    (w // 2 - 50, h // 2 - 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    (0, 0, 255),
                    3,
                )

                if self.fire_frame_count >= FIRE_DURATION_FRAMES:
                    self.arduino.send(fire=False)
                    self.state = "DONE"

            elif self.state == "DONE":
                # Döngü tamamlandı
                cv2.putText(
                    frame,
                    "DONE - Manuel tahta donun",
                    (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                time.sleep(2.0)
                self.state = "IDLE"
                self._reset_engagement()

            # ===== BİLGİ OVERLAY =====
            info_text = f"Letter:{self.engagement['letter']} Shape:{self.engagement['shape']} Color:{self.engagement['color']}"
            cv2.putText(
                frame,
                info_text,
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
            )

            # Debug
            debug_text = f"Depth: {depth_m:.1f}m | Lock: {self.lock_frame_counter}/{self.LOCK_DURATION_FRAMES}"
            cv2.putText(
                frame,
                debug_text,
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                1,
            )

            # ===== TELEMETRI =====
            telemetry = {
                "fps": fps,
                "state": self.state,
                "letter": self.engagement["letter"],
                "direction": self.engagement["direction"],
                "shape": self.engagement["shape"],
                "color": self.engagement["color"],
                "locked": locked,
                "ocr_pending": self.ocr_pending,
                "ocr_roi": ocr_roi,
            }

            yield frame, telemetry
