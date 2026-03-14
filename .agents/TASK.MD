# Hava Savunma Sistemi — Görev Listesi (Birleştirilmiş)

## PHASE 1: Core Infrastructure & Hardware Interfaces

### 1.1 Logitech Camera Wrapper (`camera/logitech_camera.py`) [YENİ]
- [ ] `LogitechCamera` sınıfı oluştur
- [ ] `open()` metodu — `cv2.VideoCapture(1, cv2.CAP_DSHOW)`, 1280×720
- [ ] `close()` ve `grab_frame()` metodları
- [ ] `is_open` property
- [ ] `CAP_PROP_BUFFERSIZE = 1` ayarla (bayat kare önleme)
- [ ] Thread-safe erişim (lock mekanizması)

### 1.2 Benewake TF03 LiDAR Parser (`hardware/lidar_interface.py`) [YENİ]
- [ ] `LidarInterface` sınıfı oluştur
- [ ] Serial bağlantı: COM3, 115200 baud, 8-N-1
- [ ] 9-byte binary paket parse:
  - [ ] Frame header doğrulama (0x59, 0x59)
  - [ ] `distance_cm = Dist_H * 256 + Dist_L`
  - [ ] Checksum doğrulama (ilk 8 byte toplamı, düşük 8 bit)
  - [ ] Sinyal gücü (strength) okuma
  - [ ] Sıcaklık okuma (byte 6-7)
- [ ] `poll_distance() → float | None` metodu (metre cinsinden döndür)
- [ ] Arka plan thread ile sürekli polling (latest-value buffer)
- [ ] Sinyal gücü eşik değeri altında → `None` döndür
- [ ] Serial timeout/hata yönetimi
- [ ] COM port `system_config.py`'den okunacak

### 1.3 Donanım Atış Sistemi Entegrasyonu
- [ ] Arduino C++ (`arduınoya_atılacak.cpp`): Pin 9 lazer/tetik HIGH/LOW döngüsü
- [ ] `arduino_interface.py`: UART `fire(duration_ms)` fonksiyonu

### 1.4 Matematiksel Dönüşümler (`base_stage.py`'ye ekle)
- [ ] `calculate_parallax_shift(depth_m, offset_x_mm, offset_y_mm) → (shift_x_px, shift_y_px)`
- [ ] Formül: `shift = (offset / Z) * focal_length_px`
- [ ] Başlangıç: `O_x ≈ 200mm`, `O_y ≈ 0mm`, `system_config.py`'den okunsun
- [ ] Açısal hız dönüşüm fonksiyonu:
  - [ ] `omega_x = (vx_px / frame_w) * ZED_HFOV_DEG`
  - [ ] `omega_y = (vy_px / frame_h) * ZED_VFOV_DEG`
  - [ ] ZED FOV sabitleri `system_config.py`'den (HFOV ≈ 110°, VFOV ≈ 70°)

### 1.5 YOLO Model Yöneticisi (Engine Manager)
- [ ] Batch=2 engine yükle: `balon_baska.engine` — Tüm stage'ler
- [ ] `balon_baska_tek.engine` (batch=1) artık kullanılmıyor

### 1.6 `system_config.py` Konfigürasyonları
- [ ] Port/Sensor: `LIDAR_PORT="COM3"`, `LIDAR_BAUDRATE=115200`
- [ ] Parallax: `PARALLAX_OFFSET_X_MM=200.0`, `PARALLAX_OFFSET_Y_MM=0.0`
- [ ] Atış süresi: `FIRE_DURATION_MS = 200`
- [ ] ZED FOV: `ZED_HFOV_DEG=110.0`, `ZED_VFOV_DEG=70.0`
- [ ] Stage bazlı sınıf listeleri:
  - [ ] `STAGE1_CLASSES = ["missile", "helicopter", "f16", "uav"]`
  - [ ] `STAGE2_CLASSES = ["missile", "uav"]`
  - [ ] `STAGE3_CLASSES = ["missile", "helicopter", "f16", "uav"]`
- [ ] Range Gating: F16 (10–15m), Missile (10–15m), Mini/Micro UAV (0–15m), Helicopter (5–15m)
- [ ] `balloon` sınıfı tüm listelerden çıkarıldı mı kontrol
- [ ] Timing parametreleri:
  - [ ] `HANDOVER_TIMEOUT_FRAMES = 30`
  - [ ] `PID_REACQUIRE_FRAMES = 15`
  - [ ] `COLOR_VERIFY_TIMEOUT_FRAMES = 30`
  - [ ] `COLOR_CONFIRM_RATIO = (3, 5)`
  - [ ] `POST_ENGAGEMENT_DELAY_S = 0.5`
  - [ ] `SLEW_STABILIZE_MS = 100`
  - [ ] `LIDAR_FALLBACK_TIMEOUT_MS = 500`
  - [ ] `LIDAR_FALLBACK_ZED_MARGIN_M = 1.0`

---

## PHASE 2: Stage 1 — Manuel Mod (Statik Hedefler)

### 2.1 Stage 1 Engine (`stage1.py` yeniden yaz)
- [ ] ZED + Logitech birlikte açılacak (batch=2)
- [ ] YOLO: `balon_baska.engine` (batch=2)
- [ ] ZED depth bilgisi overlay'de gösterilecek
- [ ] Aktif sınıflar: `missile`, `helicopter`, `f16`, `uav`
- [ ] LiDAR her frame okunacak
- [ ] Otomatik nişan/ateş YAPILMAYACAK

### 2.2 Kırmızı Piksel Hedefleme
- [ ] Bbox ROI kırp
- [ ] HSV veya LAB renk filtreleme ile kırmızı piksel maskesi
- [ ] Kırmızı piksel kümesinin merkez noktasını hesapla
- [ ] Bu merkezi nişan noktası olarak overlay'e çiz

### 2.3 Overlay
- [ ] Bounding Box + Sınıf adı + Confidence
- [ ] ZED Depth (m)
- [ ] LiDAR mesafesi (m)
- [ ] Kırmızı hedef noktası (nişan)
- [ ] FPS

### 2.4 Manuel Fire Workflow
- [ ] `frame_ready` signal ile UI'a frame gönder
- [ ] Arduino Manuel mod (`M0`)
- [ ] Joystick ile kullanıcı hizalar
- [ ] UI "Fire" butonu → `arduino.fire(duration_ms)`

---

## PHASE 3: Stage 2 — Otonom Swarm Takibi

> **3 koldan 3 hedef** gelecek (her kolda 1 tane). Mesafeye bakılmaksızın imha edilecek.

### 3.1 Donanım Başlatma
- [ ] ZED + Logitech + LiDAR aç (LiDAR telemetri/overlay için aktif)
- [ ] YOLO: `balon_baska.engine` (batch=2)
- [ ] Arduino Otonom mod (`M1`)
- [ ] Aktif sınıflar: `missile`, `uav`

### 3.2 Dış Döngü — Spotter (ZED)
- [ ] Her frame: ZED + Logitech → YOLO batch=2
- [ ] ZED: ByteTrack (`model.track(tracker="bytetrack.yaml")`)
- [ ] Her `track_id` için depth geçmişi (`deque(timestamp, depth_m)`)
- [ ] Yaklaşma hızı: `vz = Δdepth / Δtime`
- [ ] Uzaklaşan hedefleri filtrele (`vz < 0`)
- [ ] Açısal hız: `omega = (vx_px / W) * ZED_HFOV`
- [ ] Önceliklendirme: (1) sınıf önceliği, (2) kritik mesafeye ETA, (3) merkeze yakınlık

### 3.3 Spotter → Tracker Handover
- [ ] ZED primary target seç — N frame onay
- [ ] ZED piksel → pan/tilt açı hesapla
- [ ] `arduino.set_absolute_position(pan, tilt)` — slew komutu
- [ ] **Post-slew stabilization (~100ms)** bekle
- [ ] **Logitech frame buffer temizle** (bayat kareler at)
- [ ] **Taze frame al** (yeni grab)
- [ ] Logitech'te hedef ara: sınıf + bbox boyut + merkeze yakınlık
- [ ] **30 frame timeout** — bulunamazsa spotter'a dön
- [ ] Engine swap YOK (batch=2 sürekli çalışıyor)

### 3.4 İç Döngü — PID Takip (Tight Loop)
- [ ] Frame al: ZED + Logitech → YOLO batch=2
- [ ] ZED: güncel depth + açısal hız güncelle (feedforward)
- [ ] Logitech: hedef hala var mı kontrolü
- [ ] Pixel error + parallax shift + feedforward → `E{errX},Y{errY}` → Arduino
- [ ] **Hedef kaybı → Re-acquire:**
  - [ ] Son Arduino komutu dondur
  - [ ] 15 frame boyunca yeniden ara
  - [ ] Bulunamazsa → spotter dış döngüsüne dön
- [ ] **LOCKED:** `|error| < threshold` N consecutive frame
- [ ] **LOCKED → hemen FIRE!** (mesafe kontrolü yok, range gate yok)
- [ ] LiDAR/ZED depth overlay ve telemetride gösterilmeye devam eder

### 3.5 Post-Engagement Auto-Scan
- [ ] FIRE sonrası ~0.5s bekleme
- [ ] Otomatik spotter fazına dön
- [ ] Sonraki en kritik hedefi seç
- [ ] Hedef kalmadıysa → idle tarama

---

## PHASE 4: Stage 3 — Otonom + Dost/Düşman Ayrımı

> Stage 3 = Stage 2 + LAB Renk Doğrulama + Balon Rengi Maskeleme

### 4.1 Farklar (Stage 2'ye ek)
- [ ] Aktif sınıflar: `missile`, `helicopter`, `f16`, `uav` (4 sınıf)
- [ ] LAB renk sınıflandırıcı yükle (`StickerLabClassifier`)
- [ ] Dost Blacklist (`set()`) başlat

### 4.2 Balon Hedefleme & Renk Maskeleme (YARIŞMA DUYURUSU BEKLENİYOR)
- [ ] Sınıf bazlı balon pozisyonu (bbox içinde normalize bölge):
  - [ ] F16: kokpit (üst-orta, 0.2–0.5)
  - [ ] Helicopter: kokpit (üst-orta, 0.2–0.5)
  - [ ] UAV: altında (alt kısım, 0.7–1.0)
  - [ ] Missile: ucunda (üst uç, 0.0–0.3)
- [ ] Kırmızı piksel arama sadece ilgili bölgede yapılsın
- [ ] Dost/düşman renk tespiti balon pikselleri dışlanarak yapılsın
- [ ] ⚠️ **Balon patladığında hedef düşmez/kaybolmaz** — vuruş onay mekanizması yarışma duyurusunda netleşecek

### 4.3 LAB Renk Doğrulama
- [ ] `color_calibration.json` ile kalibrasyon yükle
- [ ] LAB A/B kanalları: Kırmızı=DÜŞMAN, Mavi=DOST, Yeşil=TBD
- [ ] Sınıflandırma sadece Logitech bbox ROI içinde

### 4.4 Spotter + Dost Blacklist
- [ ] Stage 2 spotter mantığı (Task 3.2)
- [ ] Hedef seçerken blacklist'teki `track_id`'leri atla

### 4.5 Renk Doğrulama Fazı (Handover sonrası, PID öncesi)
- [ ] Handover confirmed → renk doğrulama döngüsüne gir
- [ ] Her frame: bbox ROI → `classify(frame, box)`
- [ ] **Mavi (DOST)** → `track_id` blacklist'e → spotter'a dön
- [ ] **Kırmızı (DÜŞMAN)** → 3/5 frame onay → PID'e geç
- [ ] **Belirsiz** → tekrar dene
- [ ] **30 frame timeout** → hedefi atla, spotter'a dön

### 4.6 PID + Range Gate + Post-Engagement
- [ ] Stage 2 ile birebir aynı (Task 3.4, 3.5)
- [ ] 15 frame re-acquire, range gate split, LiDAR fallback dahil

### 4.7 OCR Kaldırma
- [ ] `stages/ocr_worker.py` dosyasını sil
- [ ] `arayuz.py`'den `OcrWorker` import ve referanslarını kaldır
- [ ] `stage3.py`'den OCR çağrılarını kaldır

---

## PHASE 5: Resilience & Error Handling (En Son)

### 5.1 Try/Catch Sarmalama
- [ ] Kamera açma/grab hataları → otomatik yeniden bağlanma
- [ ] Arduino seri port kopması → reconnect loop + UI uyarı
- [ ] LiDAR seri port kopması → fallback moduna geçiş
- [ ] YOLO model yükleme hatası → anlamlı hata mesajı, graceful stop
- [ ] Frame işleme hataları → frame atla, döngüye devam

### 5.2 Auto-Recovery
- [ ] Sistem hiçbir durumda çökmemeli
- [ ] Kritik donanım hatası → UI'da kırmızı uyarı + log
- [ ] Geçici hatalar → sessizce atla, devam et
- [ ] Hata sonrası tekrar çalışmaya hazır olmalı

---

## PHASE 6: UI — C# WinForms Arayüzü (`arayuz/`)

> Arayüz: `arayuz/` klasöründeki C# WinForms projesi (.NET 6.0). Python backend ile **Memory-Mapped File (MMF)** üzerinden haberleşir. Arayüz sonra güncellenecek.

### 6.1 Mevcut C# Arayüz Yapısı
- [ ] `Form1.cs` + `Form1.Designer.cs` — Ana form
- [ ] `Resources/` — Buton PNG'leri (normal/hover/tıklanmış)
- [ ] Stage geçişleri UI butonlarıyla (otomatik geçiş YOK)

### 6.2 MMF Shared Memory Protokolü (Python ↔ C#)

**C# → Python (Komut):**
- [ ] `"SystemMode"` (10 byte) — `"1"`=Otonom, `"2"`=Manuel, `"3"`=YarıOto
- [ ] `"FireDataMemory"` (1 byte bool) — Ateş komutu
- [ ] `"FireCommandMemory"` (1 byte bool) — Ateşe devam
- [ ] `"EngagementDataMemory"` (1 byte bool) — Angajman kabul
- [ ] `"AngleDataMemory"` (2048 byte) — Yasaklı alan `"min,max"` açı

**Python → C# (Görüntü):**
- [ ] `"raw_frame"` (W×H×3 byte) — Ham kamera görüntüsü
- [ ] `"processed_frame"` (W×H×3 byte) — YOLO overlay'li görüntü

### 6.3 Python Tarafı MMF Implementasyonu
- [ ] `mmap` veya `multiprocessing.shared_memory` ile shared memory yaz/oku
- [ ] Her frame: annotated görüntüyü `processed_frame` MMF'e yaz
- [ ] `SystemMode` oku → aktif stage belirle
- [ ] `FireDataMemory` oku → ateş komutu varsa `arduino.fire()` tetikle

### 6.4 Overlay (Python tarafında frame'e çizilecek)
- [ ] Stage 1: BBox + Sınıf + Conf + ZED Depth + LiDAR(m) + Kırmızı hedef + FPS
- [ ] Stage 2/3: BBox + Sınıf + Conf + ZED Depth + LiDAR + Track ID + Lock durumu

