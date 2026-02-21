"""
Sticker Renk Sınıflandırıcı - JSON kalibrasyon desteği
"""

import cv2
import numpy as np
import json
import os


class StickerLabClassifier:
    """Stage 3 için sticker renk sınıflandırıcı"""

    def __init__(self, calibration_path="color_calibration.json"):
        # Varsayılan değerler
        self.red_center = np.array([30.0, 20.0])
        self.max_dist_red = 15.0

        self.blue_center = np.array([-20.0, -25.0])
        self.max_dist_blue = 15.0

        self.green_center = np.array([-30.0, 30.0])
        self.max_dist_green = 15.0

        # JSON'dan yükle
        self.calibration_path = calibration_path
        self.load_calibration()

    def load_calibration(self):
        """color_calibration.json dosyasından yükle"""
        if not os.path.exists(self.calibration_path):
            print(
                f"[WARN] {self.calibration_path} bulunamadı, varsayılan değerler kullanılıyor"
            )
            return

        try:
            with open(self.calibration_path, "r") as f:
                cal = json.load(f)

            if "red" in cal:
                self.red_center = np.array(
                    [cal["red"]["A_center"], cal["red"]["B_center"]]
                )
                self.max_dist_red = cal["red"]["radius_ab"]

            if "blue" in cal:
                self.blue_center = np.array(
                    [cal["blue"]["A_center"], cal["blue"]["B_center"]]
                )
                self.max_dist_blue = cal["blue"]["radius_ab"]

            if "green" in cal:
                self.green_center = np.array(
                    [cal["green"]["A_center"], cal["green"]["B_center"]]
                )
                self.max_dist_green = cal["green"]["radius_ab"]

            print(f"[INFO] Sticker renk kalibrasyonu yüklendi")

        except Exception as e:
            print(f"[ERROR] JSON yükleme hatası: {e}")

    def classify(self, frame, box):
        """Aynı mantık LabColorBalloon ile"""
        x1, y1, x2, y2 = map(int, box)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)

        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            return "unknown"

        lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB)
        A_mean = np.mean(lab[:, :, 1])
        B_mean = np.mean(lab[:, :, 2])
        ab_vec = np.array([A_mean, B_mean])

        dist_red = np.linalg.norm(ab_vec - self.red_center)
        dist_blue = np.linalg.norm(ab_vec - self.blue_center)
        dist_green = np.linalg.norm(ab_vec - self.green_center)

        min_dist = min(dist_red, dist_blue, dist_green)

        if min_dist == dist_red and dist_red < self.max_dist_red:
            return "red"
        elif min_dist == dist_blue and dist_blue < self.max_dist_blue:
            return "blue"
        elif min_dist == dist_green and dist_green < self.max_dist_green:
            return "green"
        else:
            return "unknown"
