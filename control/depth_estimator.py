"""
Depth Estimator - Hybrid Tilt+Bbox
Belgede: Bölüm 5.3, 6.4
"""

import numpy as np


class DepthEstimator:
    """
    Monocular kamera için depth tahmini.
    Hybrid yaklaşım: tilt açısı + bounding box yüksekliği.
    """

    def __init__(
        self,
        camera_height_m=2.0,
        tilt_singularity_deg=5.0,
        bbox_depth_coeff=5.0,
        depth_min_m=1.0,
        depth_max_m=15.0,
    ):
        """
        Args:
            camera_height_m: Kamera yüksekliği (zemine göre, metre)
            tilt_singularity_deg: Tilt < bu değerde bbox fallback (derece)
            bbox_depth_coeff: k katsayısı (depth = k / bbox_height_norm)
            depth_min_m: Minimum depth clip (metre)
            depth_max_m: Maximum depth clip (metre)
        """
        self.camera_height_m = camera_height_m
        self.tilt_singularity_deg = tilt_singularity_deg
        self.bbox_depth_coeff = bbox_depth_coeff
        self.depth_min_m = depth_min_m
        self.depth_max_m = depth_max_m

    def estimate_depth_tilt(self, tilt_angle_deg):
        """
        Tilt açısından depth tahmini (aerial target için).

        Belgede: Bölüm 5.3 - Yaklaşım 1
        depth_tilt = camera_height / tan(tilt_angle)

        Varsayım: Balon kamera hizasında (~1m vertical offset)
        Hata payı: %30-50

        Args:
            tilt_angle_deg: Tilt açısı (derece, pozitif yukarı)

        Returns:
            depth_m: Tahmini depth (metre)
        """
        if abs(tilt_angle_deg) < 0.1:
            # Singularity: çok düz bakınca sonsuz olur
            return self.depth_max_m

        tilt_rad = np.deg2rad(tilt_angle_deg)
        depth_m = self.camera_height_m / np.tan(abs(tilt_rad))

        # Clip
        depth_m = np.clip(depth_m, self.depth_min_m, self.depth_max_m)
        return depth_m

    def estimate_depth_bbox(self, bbox_height, frame_height):
        """
        Bounding box yüksekliğinden depth tahmini.

        Belgede: Bölüm 5.3 - Yaklaşım 2
        depth_bbox = k / (bbox_height_normalized)

        k: Kalibrasyon sabiti (balon bilinen çap 30cm)
        Hata payı: %40-60

        Args:
            bbox_height: Bounding box yüksekliği (piksel)
            frame_height: Frame yüksekliği (piksel)

        Returns:
            depth_m: Tahmini depth (metre)
        """
        bbox_height_norm = bbox_height / frame_height

        if bbox_height_norm < 0.001:
            # Çok küçük hedef, çok uzak
            return self.depth_max_m

        depth_m = self.bbox_depth_coeff / bbox_height_norm

        # Clip
        depth_m = np.clip(depth_m, self.depth_min_m, self.depth_max_m)
        return depth_m

    def estimate_depth_hybrid(self, tilt_angle_deg, bbox_height, frame_height):
        """
        Hybrid depth estimation: tilt veya bbox yöntemini seç.

        Belgede: Bölüm 5.3 - Hibrit seçim
        if abs(tilt) >= 5°:
            depth = depth_tilt
        else:
            depth = depth_bbox

        Args:
            tilt_angle_deg: Tilt açısı (derece)
            bbox_height: Bounding box yüksekliği (piksel)
            frame_height: Frame yüksekliği (piksel)

        Returns:
            depth_m: Tahmini depth (metre)
        """
        if abs(tilt_angle_deg) >= self.tilt_singularity_deg:
            # Tilt-based güvenilir
            depth_m = self.estimate_depth_tilt(tilt_angle_deg)
        else:
            # Bbox-based fallback
            depth_m = self.estimate_depth_bbox(bbox_height, frame_height)

        return depth_m
