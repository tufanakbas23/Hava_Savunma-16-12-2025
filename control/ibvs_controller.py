"""
Basit P Kontrol - Pan-Tilt Takip Sistemi
Sadece Kp * error. Başka hiçbir şey yok.
"""


class IBVSController:
    """Sabit Kp ile oransal kontrol."""

    def __init__(self, kp=1.5):
        self.kp = kp

    def compute(self, error_x_norm, error_y_norm):
        """
        Args:
            error_x_norm: Normalize X hatası (-1 ile +1)
            error_y_norm: Normalize Y hatası (-1 ile +1)
        Returns:
            dpan, dtilt: Komut çıktıları
        """
        return self.kp * error_x_norm, self.kp * error_y_norm
