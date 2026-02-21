"""
Sistem Konfigürasyonu - Belgede: Bölüm 6
Tüm ayarlanabilir parametreler burada.
"""


class SystemConfig:
    """Tüm sistem parametrelerini tutan config sınıfı"""

    # ========== YOLO Model Paths (HER STAGE İÇİN) ==========
    YOLO_MODEL_STAGE1 = r"C:\Users\Tufan\Desktop\yolo8_proje_goruntu\balon_baska.pt"
    YOLO_MODEL_STAGE2 = r"C:\Users\Tufan\Desktop\yolo8_proje_goruntu\balon_baska.pt"
    YOLO_MODEL_STAGE3 = (
        r"C:\Users\Tufan\Desktop\yolo8_proje_goruntu\balon_baska.pt"
    )

    # ========== 6.1 Kalibrasyon (Arduino tarafı) ==========
    # Belgede: Bölüm 6.1
    PULSES_PER_DEG_PAN = 13.3  # Başlangıç, sahada ölçülecek
    PULSES_PER_DEG_TILT = 13.3  # Başlangıç, sahada ölçülecek
    MAX_SPEED = 4000  # pulses/s
    ACCELERATION = 8000  # pulses/s²

    # ========== 6.2 IBVS Kontrol Parametreleri ==========
    # ⚡ AYARLANACAK ANA PARAMETRELER BURADA ⚡
    
    # Kamera Modu
    CAMERA_ON_TURRET = True     # True = Hareketli kamera (IBVS), False = Sabit kamera
    USE_FULL_IBVS = False       # True = Motion Comp + Lead, False = Basit IBVS
    
    # Adaptif PD Controller
    LAMBDA_MIN = 0.3            # Yakında düşük gain → salınım önlenir
    LAMBDA_MAX = 2.0            # Uzakta yüksek gain → hızlı yakalama
    KD_GAIN = 2.0               # Azı fren → hızlı takipte geri kalmaz
    
    # Lead Prediction
    LEAD_TIME_S = 0.50          # Tahmin süresi (saniye)
    
    # Dead Zone & Rate Limiter
    DEAD_ZONE_NORM = 0.08       # Artırıldı → salınım bölgesi genişledi
    MAX_DELTA_DEG = 3.0         # Maks açı değişimi/frame
    
    # Feedforward: Hedefin hızını motor komutuna direkt ekle
    FEEDFORWARD_GAIN = 0.7      # Artırıldı → hedefin hızına daha iyi yetişir
    
    # Sabit Kamera P Kontrol
    KP_FIXED = 0.1              # Sabit kamera Kp değeri
    DEAD_ZONE_PIX = 20          # Sabit kamera dead zone (piksel)
    
    # Diğer Parametreler
    CAMERA_HEIGHT_M = 2.0       # Kamera yüksekliği (metre)
    PAN_GAIN = 1.0              # Pan fine-tuning kazancı
    TILT_GAIN = 1.0             # Tilt fine-tuning kazancı

    # ========== 6.3 FOV Parametreleri ==========
    HFOV_DEG = 78.0             # Horizontal FOV (derece)
    VFOV_DEG = 50.0             # Vertical FOV (derece)

    # ========== 6.4 Tracking Parametreleri ==========

    # Depth Estimation
    DEPTH_MIN_M = 1.0
    DEPTH_MAX_M = 15.0
    TILT_SINGULARITY_DEG = 5.0
    BBOX_DEPTH_COEFF = 5.0

    # Motion Compensation
    OMEGA_LOWPASS_ALPHA = 0.3

    # Lead Prediction (Full IBVS)
    LEAD_TIME_BASE = 0.15
    LEAD_ADAPTIVE = False

    # Lock Criteria
    LOCK_TOLERANCE_PIX = 20     # Balonun hedef bölgesi (piksel) - artırıldı
    LOCK_DURATION_FRAMES = 5    # Düşürüldü - daha hızlı kilit

    # Kalman Filter
    PROCESS_NOISE = 1e-1        # Artırıldı → Kalman hız tahmini daha reaktif
    MEASUREMENT_NOISE = 1e-1

    # ========== Titreşim Önleme ==========
    ANGLE_SMOOTH_ALPHA = 0.4
    MAX_DELTA_DEG_PER_FRAME = 2.0
    LOCK_TOLERANCE_DEG = 1.0

    # ========== Arduino Seri Port ==========
    ARDUINO_PORT = "COM3"  # Windows: COM3
    ARDUINO_BAUDRATE = 115200
    ARDUINO_TIMEOUT = 0.1

    # ========== Kamera ==========
    CAMERA_INDEX = 0  # 0 = varsayılan kamera
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480

    # ========== Stage 3 Platform Açıları ==========
    # Belgede: Bölüm 10 - Adım 6
    PLATFORM_PAN_LEFT_DEG = -45.0  # Sol platform açısı
    PLATFORM_PAN_RIGHT_DEG = 45.0  # Sağ platform açısı
    PLATFORM_TILT_DEG = 15.0  # Angajman tilt açısı

    # ========== Size Classification ==========
    LARGE_THRESHOLD = 10000  # Piksel alanı eşiği (büyük/küçük)

    @classmethod
    def print_config(cls):
        """Konfigürasyonu ekrana yazdır (debug için)"""
        print("=" * 60)
        print("SİSTEM KONFIGÜRASYONU")
        print("=" * 60)
        for key, value in cls.__dict__.items():
            if not key.startswith("_") and key.isupper():
                print(f"{key:30s} = {value}")
        print("=" * 60)
