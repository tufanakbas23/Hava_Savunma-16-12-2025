"""
Sistem Konfigürasyonu
Tüm ayarlanabilir parametreler burada.
"""
import os

# Proje kök dizini
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class SystemConfig:
    """Tüm sistem parametrelerini tutan config sınıfı"""

    # ========== ZED 2 Kamera ==========
    USE_ZED_CAMERA = True               # False → OpenCV fallback
    ZED_RESOLUTION = "HD720"            # HD720 | HD1080 | HD2K
    ZED_FPS = 60                        # 15 | 30 | 60
    ZED_DEPTH_MODE = "ULTRA"            # NONE | PERFORMANCE | QUALITY | ULTRA
    ZED_MAX_DEPTH_M = 20.0              # Maks derinlik mesafesi (metre)

    # ========== YOLO Inference ==========
    YOLO_CONF_THRESHOLD = 0.35          # Güven eşiği
    YOLO_IMGSZ = 480                    # Inference boyutu (480→hız, 640→doğruluk)
    YOLO_DEVICE = 0                     # 0 = GPU, "cpu" = CPU
    YOLO_HALF = True                    # FP16 half precision (GPU only)

    # ========== Object Classes (5 Sınıf) ==========
    # Otonom sistem terminolojisi ile sınıf tanımları
    CLASS_NAMES = ["drone", "missile", "helicopter", "f16", "balloon"]

    # Aktif sınıflar — sadece bunlar işlenir, geri kalanı filtrelenir
    # Stage 1 demo modeli için varsayılan: sadece balon
    ACTIVE_CLASSES = ["balloon"]

    # Stage 2 için tehdit sınıfları (Swarm Attack senaryosu)
    # Not: Buradaki isimler YOLO modelindeki sınıf isimleri ile eşleşmelidir.
    STAGE2_ACTIVE_CLASSES = [
        "ballistic_missile",
        "uav",
        "mini_uav",
        "micro_uav",
    ]

    # ========== YOLO Model Paths ==========
    YOLO_MODEL_STAGE1 = os.path.join(PROJECT_ROOT, "balon_baska.pt")
    YOLO_MODEL_STAGE2 = os.path.join(PROJECT_ROOT, "balon_baska.pt")
    YOLO_MODEL_STAGE3 = os.path.join(PROJECT_ROOT, "balon_baska.pt")

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

    # ========== Stage 2: Otonom Swarm Takip ==========
    # Yaklaşan hedef analizi + angajman mesafeleri
    STAGE2_MIN_DISTANCE_M = 8.0          # Kritik minimum mesafe (angajman sınırı)
    STAGE2_SAFETY_MARGIN_M = 2.0         # Sınırdan önce angajman için güvenlik payı
    STAGE2_MAX_ENGAGE_DISTANCE_M = 18.0  # Bu mesafeden öte hedefler öncelik dışı

    # Yaklaşma (Z-ekseni) analizi
    STAGE2_MIN_APPROACH_VZ_MPS = 0.5     # m/s, pozitif = yaklaşan
    STAGE2_APPROACH_WINDOW_SEC = 0.7     # Ortalama hız hesabı için zaman penceresi

    # Hedef önceliklendirme (sınıf bazlı)
    STAGE2_CLASS_PRIORITY = {
        "ballistic_missile": 3,
        "missile": 3,          # Alternatif isimlendirme desteği
        "uav": 2,
        "mini_uav": 1,
        "micro_uav": 1,
    }

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
