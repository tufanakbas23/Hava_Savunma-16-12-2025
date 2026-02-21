"""
IBVS Controller - Image-Based Visual Servoing
Belgede: Bölüm 5.1, 5.2
"""

import numpy as np


class IBVSController:
    """
    Pan-tilt için IBVS kontrol kuralı.
    Adaptif PD kontrol: Hata büyük → yüksek gain, hata küçük → düşük gain
    """

    def __init__(self, lambda_min=0.5, lambda_max=3.0, kd_gain=0.5):
        """
        Args:
            lambda_min: Minimum P gain (hedef yakınken)
            lambda_max: Maximum P gain (hedef uzakken)
            kd_gain: D terimi (salınım önleme)
        """
        self.lambda_min = lambda_min
        self.lambda_max = lambda_max
        self.kd_gain = kd_gain
        
        # Önceki hata (D terimi için)
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

    def compute_image_jacobian(self, x_norm, y_norm):
        """
        Pan-Tilt için Image Jacobian hesapla.
        """
        L = np.array(
            [[-(1 + x_norm**2), -x_norm * y_norm], [x_norm * y_norm, -(1 + y_norm**2)]]
        )
        return L
    
    def compute_adaptive_lambda(self, error_magnitude):
        """
        Adaptif gain hesapla: Hata büyük → yüksek gain, hata küçük → düşük gain
        
        Args:
            error_magnitude: Normalize hata büyüklüğü (0-1 arası)
        
        Returns:
            lambda: Adaptif gain değeri
        """
        # Lineer interpolasyon: error=0 → lambda_min, error=1 → lambda_max
        # Clamp: 0 ile 1 arası
        error_clamped = np.clip(error_magnitude, 0.0, 1.0)
        
        # Adaptif gain
        lambda_adaptive = self.lambda_min + (self.lambda_max - self.lambda_min) * error_clamped
        
        return lambda_adaptive

    def compute_control_velocity(self, error_x_norm, error_y_norm):
        """
        Adaptif IBVS PD kontrol
        - Uzaktayken: Hızlı hareket (yüksek gain)
        - Yakındayken: Yavaş ve hassas (düşük gain, salınım yok)

        Returns:
            omega_tilt: Tilt angular velocity (deg/s)
            omega_pan: Pan angular velocity (deg/s)
        """
        x_norm = -error_x_norm
        y_norm = -error_y_norm

        # Image Jacobian
        L = self.compute_image_jacobian(x_norm, y_norm)

        try:
            L_inv = np.linalg.pinv(L)
        except:
            return 0.0, 0.0

        error = np.array([error_x_norm, error_y_norm])
        
        # Hata büyüklüğü (Euclidean norm)
        error_magnitude = np.sqrt(error_x_norm**2 + error_y_norm**2)
        
        # Adaptif gain hesapla
        lambda_adaptive = self.compute_adaptive_lambda(error_magnitude)
        
        # D terimi: hata değişim hızı
        d_error_x = error_x_norm - self.prev_error_x
        d_error_y = error_y_norm - self.prev_error_y
        
        # Önceki hatayı güncelle
        self.prev_error_x = error_x_norm
        self.prev_error_y = error_y_norm

        # P terimi: Adaptif IBVS
        velocity_p = -lambda_adaptive * L_inv @ error
        
        # D terimi: salınım önleme
        velocity_d = np.array([-self.kd_gain * d_error_x, -self.kd_gain * d_error_y])
        
        # Toplam komut
        velocity_command = velocity_p + velocity_d

        omega_tilt_rad = velocity_command[0]
        omega_pan_rad = velocity_command[1]

        omega_tilt_deg = np.rad2deg(omega_tilt_rad)
        omega_pan_deg = np.rad2deg(omega_pan_rad)

        return omega_tilt_deg, omega_pan_deg


class VelocityTransformer:
    """
    Piksel hızını → Açısal hız → Gerçek hız (m/s) dönüşümü
    Belgede: Bölüm 5.4, 5.5
    """

    def __init__(self, hfov_deg=60.0, vfov_deg=45.0, frame_width=640, frame_height=480):
        """
        Args:
            hfov_deg: Horizontal FOV (derece)
            vfov_deg: Vertical FOV (derece)
            frame_width: Frame genişliği (piksel)
            frame_height: Frame yüksekliği (piksel)
        """
        self.hfov_deg = hfov_deg
        self.vfov_deg = vfov_deg
        self.frame_width = frame_width
        self.frame_height = frame_height

        # Derece/piksel oranı
        self.deg_per_pixel_h = hfov_deg / frame_width
        self.deg_per_pixel_v = vfov_deg / frame_height

    def pixel_velocity_to_angular(self, vx_pix, vy_pix):
        """
        Piksel hızını açısal hıza çevir.

        Belgede: Bölüm 5.4
        omega_h = vx_pix * deg_per_pixel_h (deg/s)

        Args:
            vx_pix: X yönünde piksel hızı (piksel/s)
            vy_pix: Y yönünde piksel hızı (piksel/s)

        Returns:
            omega_h_deg: Horizontal açısal hız (deg/s)
            omega_v_deg: Vertical açısal hız (deg/s)
        """
        omega_h_deg = vx_pix * self.deg_per_pixel_h
        omega_v_deg = vy_pix * self.deg_per_pixel_v
        return omega_h_deg, omega_v_deg

    def angular_to_real_velocity(self, omega_h_deg, omega_v_deg, depth_m):
        """
        Açısal hızı gerçek hıza çevir (small angle approximation).

        Belgede: Bölüm 5.4
        vx_real = depth * tan(omega_h * π/180) ≈ depth * omega_h * π/180

        Args:
            omega_h_deg: Horizontal açısal hız (deg/s)
            omega_v_deg: Vertical açısal hız (deg/s)
            depth_m: Derinlik (metre)

        Returns:
            vx_real: Gerçek X hızı (m/s)
            vy_real: Gerçek Y hızı (m/s)
        """
        # Small angle: tan(θ) ≈ θ (radyan)
        omega_h_rad = np.deg2rad(omega_h_deg)
        omega_v_rad = np.deg2rad(omega_v_deg)

        vx_real = depth_m * np.tan(omega_h_rad)
        vy_real = depth_m * np.tan(omega_v_rad)

        return vx_real, vy_real

    def lead_prediction_pixel(
        self, x_pix, y_pix, vx_real, vy_real, depth_m, lead_time_s
    ):
        """
        Lead prediction: gelecekteki pozisyonu tahmin et ve piksele çevir.

        Belgede: Bölüm 5.5

        Args:
            x_pix: Şu anki X piksel
            y_pix: Şu anki Y piksel
            vx_real: Gerçek X hızı (m/s)
            vy_real: Gerçek Y hızı (m/s)
            depth_m: Derinlik (metre)
            lead_time_s: Lead time (saniye)

        Returns:
            x_pix_pred: Tahmini X piksel
            y_pix_pred: Tahmini Y piksel
        """
        # 1) Şu anki piksel → açı
        cx = self.frame_width / 2
        cy = self.frame_height / 2
        x_angle_deg = (x_pix - cx) * self.deg_per_pixel_h
        y_angle_deg = (y_pix - cy) * self.deg_per_pixel_v

        # 2) Açı → gerçek pozisyon (metre)
        x_real = depth_m * np.tan(np.deg2rad(x_angle_deg))
        y_real = depth_m * np.tan(np.deg2rad(y_angle_deg))

        # 3) Gelecekteki gerçek pozisyon
        x_real_pred = x_real + vx_real * lead_time_s
        y_real_pred = y_real + vy_real * lead_time_s

        # 4) Gerçek → açı → piksel
        x_angle_pred_deg = np.rad2deg(np.arctan2(x_real_pred, depth_m))
        y_angle_pred_deg = np.rad2deg(np.arctan2(y_real_pred, depth_m))

        x_pix_pred = cx + x_angle_pred_deg / self.deg_per_pixel_h
        y_pix_pred = cy + y_angle_pred_deg / self.deg_per_pixel_v

        return x_pix_pred, y_pix_pred
