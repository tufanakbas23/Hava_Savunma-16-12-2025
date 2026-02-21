"""
LAB Renk Sınıflandırıcı - JSON kalibrasyon desteği eklendi
"""

import cv2
import numpy as np
import json
import os


class LabColorBalloon:
    """
    LAB renk uzayında balon rengi sınıflandırıcı
    color_calibration.json dosyasından kalibrasyon yükler
    """

    def __init__(self, calibration_path="color_calibration.json"):
        # Varsayılan değerler (JSON yoksa kullanılacak)
        self.red_center = np.array([30.0, 20.0])  # A, B
        self.max_dist_red = 15.0

        self.blue_center = np.array([-20.0, -25.0])
        self.max_dist_blue = 15.0

        self.green_center = np.array([-30.0, 30.0])
        self.max_dist_green = 15.0

        # JSON'dan yükle
        self.calibration_path = calibration_path
        self.load_calibration()

    def load_calibration(self):
        """color_calibration.json dosyasından kalibrasyon yükle"""
        if not os.path.exists(self.calibration_path):
            print(
                f"[WARN] {self.calibration_path} bulunamadı, varsayılan değerler kullanılıyor"
            )
            return

        try:
            with open(self.calibration_path, "r") as f:
                cal = json.load(f)

            # Red
            if "red" in cal:
                self.red_center = np.array(
                    [cal["red"]["A_center"], cal["red"]["B_center"]]
                )
                self.max_dist_red = cal["red"]["radius_ab"]

            # Blue
            if "blue" in cal:
                self.blue_center = np.array(
                    [cal["blue"]["A_center"], cal["blue"]["B_center"]]
                )
                self.max_dist_blue = cal["blue"]["radius_ab"]

            # Green
            if "green" in cal:
                self.green_center = np.array(
                    [cal["green"]["A_center"], cal["green"]["B_center"]]
                )
                self.max_dist_green = cal["green"]["radius_ab"]

            print(f"[INFO] Renk kalibrasyonu yüklendi: {self.calibration_path}")
            print(
                f"  Red:   A={self.red_center[0]:.1f}, B={self.red_center[1]:.1f}, R={self.max_dist_red:.1f}"
            )
            print(
                f"  Blue:  A={self.blue_center[0]:.1f}, B={self.blue_center[1]:.1f}, R={self.max_dist_blue:.1f}"
            )
            print(
                f"  Green: A={self.green_center[0]:.1f}, B={self.green_center[1]:.1f}, R={self.max_dist_green:.1f}"
            )

        except Exception as e:
            print(f"[ERROR] JSON yükleme hatası: {e}, varsayılan değerler kullanılıyor")

    def reload_calibration(self):
        """Kalibrasyon dosyasını yeniden yükle (runtime'da güncellemek için)"""
        self.load_calibration()

    def classify(self, frame, box):
        """
        Balonun rengini LAB uzayında sınıflandır

        Args:
            frame: BGR görüntü
            box: [x1, y1, x2, y2]

        Returns:
            "red", "blue", "green", "unknown"
        """
        x1, y1, x2, y2 = map(int, box)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)

        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            return "unknown"

        # LAB'a çevir
        lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB)
        L_mean = np.mean(lab[:, :, 0])
        A_mean = np.mean(lab[:, :, 1])
        B_mean = np.mean(lab[:, :, 2])

        # A, B vektörü
        ab_vec = np.array([A_mean, B_mean])

        # Her renge uzaklık
        dist_red = np.linalg.norm(ab_vec - self.red_center)
        dist_blue = np.linalg.norm(ab_vec - self.blue_center)
        dist_green = np.linalg.norm(ab_vec - self.green_center)

        # En yakın renk
        min_dist = min(dist_red, dist_blue, dist_green)

        if min_dist == dist_red and dist_red < self.max_dist_red:
            return "red"
        elif min_dist == dist_blue and dist_blue < self.max_dist_blue:
            return "blue"
        elif min_dist == dist_green and dist_green < self.max_dist_green:
            return "green"
        else:
            return "unknown"
