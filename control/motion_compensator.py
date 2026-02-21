"""
Motion Compensator - Kamera hareketi kompanzasyonu
Belgede: Bölüm 4 - Katman 3
"""

import numpy as np


class MotionCompensator:
    """
    Kamera açısal hareketinden kaynaklanan piksel hareketini hesaplar ve telafi eder.
    Image Jacobian (L) ile camera-induced velocity bulunur.
    """

    def __init__(
        self,
        hfov_deg=60.0,
        vfov_deg=45.0,
        frame_width=640,
        frame_height=480,
        omega_lowpass_alpha=0.3,
    ):
        """
        Args:
            hfov_deg: Horizontal FOV (derece)
            vfov_deg: Vertical FOV (derece)
            frame_width: Frame genişliği (piksel)
            frame_height: Frame yüksekliği (piksel)
            omega_lowpass_alpha: Low-pass filter alpha (0-1)
        """
        self.hfov_deg = hfov_deg
        self.vfov_deg = vfov_deg
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.omega_lowpass_alpha = omega_lowpass_alpha

        # Smooth omega tracking
        self.omega_pan_smooth = 0.0
        self.omega_tilt_smooth = 0.0

    def update_omega_smooth(self, omega_pan_deg, omega_tilt_deg):
        """
        Açısal hızı low-pass filtre ile yumuşat.

        Belgede: Bölüm 6.4 - OMEGA_LOWPASS_ALPHA

        Args:
            omega_pan_deg: Pan açısal hızı (deg/s)
            omega_tilt_deg: Tilt açısal hızı (deg/s)
        """
        alpha = self.omega_lowpass_alpha
        self.omega_pan_smooth = (
            alpha * omega_pan_deg + (1 - alpha) * self.omega_pan_smooth
        )
        self.omega_tilt_smooth = (
            alpha * omega_tilt_deg + (1 - alpha) * self.omega_tilt_smooth
        )

    def compute_camera_induced_velocity(self, x_pix, y_pix):
        """
        Kamera hareketi nedeniyle hedefe eklenen görsel hızı hesapla.

        Belgede: Bölüm 4 - Katman 3
        v_cam = L(x,y) * [omega_tilt, omega_pan]

        Args:
            x_pix: Hedefin X pikseli
            y_pix: Hedefin Y pikseli

        Returns:
            vx_cam_pix: Kamera kaynaklı X hızı (piksel/s)
            vy_cam_pix: Kamera kaynaklı Y hızı (piksel/s)
        """
        # Normalize koordinatlar
        cx = self.frame_width / 2
        cy = self.frame_height / 2
        x_norm = (x_pix - cx) / (self.frame_width / 2)
        y_norm = (y_pix - cy) / (self.frame_height / 2)

        # Image Jacobian (pan-tilt için)
        # L = [[-1-x², -xy],
        #      [xy, -(1+y²)]]
        L = np.array(
            [[-(1 + x_norm**2), -x_norm * y_norm], [x_norm * y_norm, -(1 + y_norm**2)]]
        )

        # Omega vektörü (rad/s)
        omega_tilt_rad = np.deg2rad(self.omega_tilt_smooth)
        omega_pan_rad = np.deg2rad(self.omega_pan_smooth)
        omega = np.array([omega_tilt_rad, omega_pan_rad])

        # Feature velocity (normalized frame/s)
        v_norm = L @ omega

        # Normalized → piksel
        vx_cam_pix = v_norm[0] * (self.frame_width / 2)
        vy_cam_pix = v_norm[1] * (self.frame_height / 2)

        return vx_cam_pix, vy_cam_pix

    def compensate_velocity(self, vx_measured_pix, vy_measured_pix, x_pix, y_pix):
        """
        Ölçülen piksel hızından kamera hareketini çıkar.

        Belgede: Bölüm 4 - Katman 3
        v_compensated = v_measured - v_cam

        Args:
            vx_measured_pix: Ölçülen X hızı (piksel/s, Kalman'dan)
            vy_measured_pix: Ölçülen Y hızı (piksel/s)
            x_pix: Hedefin X pikseli
            y_pix: Hedefin Y pikseli

        Returns:
            vx_comp_pix: Kompanze edilmiş X hızı (piksel/s)
            vy_comp_pix: Kompanze edilmiş Y hızı (piksel/s)
        """
        vx_cam, vy_cam = self.compute_camera_induced_velocity(x_pix, y_pix)

        vx_comp_pix = vx_measured_pix - vx_cam
        vy_comp_pix = vy_measured_pix - vy_cam

        return vx_comp_pix, vy_comp_pix
