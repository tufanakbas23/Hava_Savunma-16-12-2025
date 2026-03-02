"""
Sistem Konfigürasyonu
Tüm ayarlanabilir parametreler burada.
"""


class SystemConfig:
    """Tüm sistem parametrelerini tutan config sınıfı"""

    # ========== YOLO Model Paths ==========
    YOLO_MODEL_STAGE1 = r"C:\Users\Tufan\Desktop\yolo8_proje_goruntu\balon_baska.pt"
    YOLO_MODEL_STAGE2 = r"C:\Users\Tufan\Desktop\yolo8_proje_goruntu\balon_baska.pt"
    YOLO_MODEL_STAGE3 = r"C:\Users\Tufan\Desktop\yolo8_proje_goruntu\balon_baska.pt"

    # ========== IBVS Kontrol ==========
    CAMERA_ON_TURRET = True     # True = Hareketli, False = Sabit kamera

    # P Kontrol
    KP = 1.5                    # Düşük tut → salınım yok, hız Arduino'dan gelecek

    # Lead Prediction
    LEAD_TIME_S = 0.50          # Sistem gecikmesi telafisi

    # Dead Zone & Rate Limiter
    DEAD_ZONE_NORM = 0.04       # Normalize dead zone
    MAX_DELTA_DEG = 5.0         # Maks açı değişimi/frame

    # Feedforward
    FEEDFORWARD_GAIN = 1.0   # Hedefin hızını motor komutuna ekle

    # Sabit Kamera
    KP_FIXED = 0.1
    DEAD_ZONE_PIX = 20

    # ========== FOV ==========
    HFOV_DEG = 78.0
    VFOV_DEG = 50.0

    # ========== Tracking ==========
    LOCK_TOLERANCE_PIX = 20
    LOCK_DURATION_FRAMES = 5

    # Kalman Filter
    PROCESS_NOISE =5e-1        # 0.1→0.3 daha reaktif hız tahmini
    MEASUREMENT_NOISE = 1e-1

    # ========== Arduino ==========
    ARDUINO_PORT = "COM4"
    ARDUINO_BAUDRATE = 115200
    ARDUINO_TIMEOUT = 0.1

    # ========== Kamera ==========
    CAMERA_INDEX = 0
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480

    # ========== Stage 3 Platform Açıları ==========
    PLATFORM_PAN_LEFT_DEG = -45.0
    PLATFORM_PAN_RIGHT_DEG = 45.0
    PLATFORM_TILT_DEG = 15.0

    # ========== Size Classification ==========
    LARGE_THRESHOLD = 10000

    @classmethod
    def print_config(cls):
        """Konfigürasyonu ekrana yazdır"""
        print("=" * 50)
        print("SİSTEM KONFIGÜRASYONU")
        print("=" * 50)
        for key, value in cls.__dict__.items():
            if not key.startswith("_") and key.isupper():
                print(f"{key:30s} = {value}")
        print("=" * 50)
