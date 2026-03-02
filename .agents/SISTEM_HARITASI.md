# 🛡️ Hava Savunma Sistemi — Dosya ve Modül Haritası

Bu belge, projedeki **her dosyanın ne iş yaptığını**, modüller arası bağlantıları ve veri akışını açıklar.

---

## Genel Mimari

```
Kamera → YOLO Algılama → Kalman Takip → P Kontrol → Arduino → Step Motorlar
  ↑                                                                    ↓
  └────────────────── Görsel Geri Besleme (Closed Loop) ◄──────────────┘
```

```
┌─────────────────────────────────────────────────────┐
│                  arayuz.py (GUI)                    │
│  Stage1Worker / Stage2Worker / Stage3Worker          │
│  ColorCalibrationWidget                              │
└───────┬──────────────┬──────────────┬───────────────┘
        │              │              │
   ┌────▼───┐    ┌─────▼───┐    ┌────▼────┐
   │stage1.py│    │stage2.py│    │stage3.py│
   └────┬────┘    └────┬────┘    └────┬────┘
        │              │              │
   ┌────▼──────────────▼──────────────▼────┐
   │          base_stage.py                 │
   │  (YOLO + Kalman + IBVS + Arduino)     │
   └──┬─────────┬──────────┬───────────┬───┘
      │         │          │           │
┌─────▼──┐ ┌───▼────┐ ┌───▼──────┐ ┌──▼──────────────┐
│ YOLO   │ │Kalman  │ │ IBVS     │ │arduino_interface│
│Detector│ │Tracker │ │Controller│ │    (Serial)     │
└────────┘ └────────┘ └──────────┘ └──────┬──────────┘
                                          │ UART
                                   ┌──────▼──────────┐
                                   │ Arduino C++     │
                                   │ (Step Motorlar) │
                                   └─────────────────┘
```

---

## 📁 Dosya Bazında Detaylı Açıklama

### Kök Dizin

| Dosya | Satır | Açıklama |
|-------|-------|----------|
| `arayuz.py` | ~1050 | **Ana uygulama.** PyQt6 GUI, kamera thread'leri, tüm stage worker'ları barındırır |
| `arduınoya_atılacak.cpp` | 318 | **Arduino firmware.** Step motor kontrolü, joystick, seri haberleşme |
| `zed.py` | ~100 | ZED 2 stereo kamera + YOLO entegrasyonu (ayrı bir çalışma) |
| `color_calibration.json` | — | Renk sınıflandırma kalibrasyon verileri (LAB A/B merkez + yarıçap) |
| `joystick_test.cpp` | — | Joystick test kodu (debug amaçlı) |

#### YOLO Model Dosyaları (.pt)

| Dosya | Boyut | Kullanım |
|-------|-------|----------|
| `balon_baska.pt` | 6 MB | **Ana model** — Stage 1/2/3 balon tespiti |
| `balon_yolo.pt` | 22 MB | Alternatif büyük model |
| `insan.pt` | 6 MB | İnsan tespiti modeli |
| `yolo8n.pt` / `yolov8n.pt` | 6 MB | Genel YOLOv8 nano modeller |
| `yolo10s.pt` | 49 MB | YOLOv10 small modeli |

---

### 📂 `config/` — Sistem Konfigürasyonu

#### `system_config.py`
**Tüm ayarlanabilir parametrelerin tek merkezden yönetildiği dosya.**

`SystemConfig` sınıfı şu parametreleri tutar:

| Parametre Grubu | Örnekler |
|-----------------|----------|
| YOLO model yolları | `YOLO_MODEL_STAGE1`, `YOLO_MODEL_STAGE2`, `YOLO_MODEL_STAGE3` |
| IBVS kontrol | `KP=1.5`, `LEAD_TIME_S=0.50`, `DEAD_ZONE_NORM=0.04`, `MAX_DELTA_DEG=5.0` |
| Feedforward | `FEEDFORWARD_GAIN=1.0` |
| FOV | `HFOV_DEG=78.0`, `VFOV_DEG=50.0` |
| Kalman | `PROCESS_NOISE=0.5`, `MEASUREMENT_NOISE=0.1` |
| Arduino | `ARDUINO_PORT="COM4"`, `ARDUINO_BAUDRATE=115200` |
| Kamera | `CAMERA_INDEX=0`, `FRAME_WIDTH=640`, `FRAME_HEIGHT=480` |
| Stage 3 | `PLATFORM_PAN_LEFT_DEG=-45.0`, `PLATFORM_PAN_RIGHT_DEG=45.0` |

---

### 📂 `detector/` — Nesne Algılama

#### `yolo_detector.py` (25 satır)
- **Sınıf:** `YoloDetector`
- YOLO modelini yükler ve `track_video()` ile frame-by-frame algılama yapar
- ByteTrack tracker kullanır (çoklu hedef takibi)
- Inference boyutu: 480px (hız optimizasyonu)
- GPU (`device=0`) ve half precision desteği

---

### 📂 `control/` — Kontrol Algoritmaları

#### `ibvs_controller.py` (22 satır)
- **Sınıf:** `IBVSController`
- Saf P (Proportional) kontrol: `dpan = Kp × error_x`, `dtilt = Kp × error_y`
- Girdi: normalize hata (-1 ile +1 arası)
- Çıktı: pan ve tilt açı değişimi

#### `kalman_tracker.py` (50 satır)
- **Sınıf:** `TargetKalman`
- 4 durumlu Kalman filtre: `[x, y, vx, vy]`
- `predict(dt)` → Bir sonraki konumu ve hızı tahmin eder
- `correct(x, y)` → YOLO ölçümü ile filtreyi günceller
- Çıktıları base_stage'de feedforward ve lead prediction için kullanılır

---

### 📂 `hardware/` — Donanım Arayüzü

#### `arduino_interface.py` (444 satır)
- **Sınıf:** `ArduinoInterface`
- Seri port üzerinden Arduino ile iletişim (115200 baud)
- **Ana metotlar:**

| Metot | İşlev |
|-------|-------|
| `update_angles(d_pan, d_tilt)` | Delta açı gönder |
| `update_angles_smooth(d_pan, d_tilt, alpha)` | Yumuşatılmış açı güncellemesi |
| `set_absolute_position(pan, tilt)` | Mutlak pozisyon ayarla |
| `send(fire)` | Hedef açıyı Arduino'ya gönder |
| `go_home()` | (0°, 0°) pozisyonuna git |
| `set_mode(autonomous)` | Otonom/Manuel mod geçişi |
| `get_status()` | Durum sorgulama |
| `fire(duration_ms)` | Lazeri belirli süre aktif et |
| `set_gains(pan_gain, tilt_gain)` | Kalibrasyon kazançları |
| `set_smoothing(alpha)` | Yumuşatma katsayısı |
| `get_omega()` | Açısal hız (motion compensation için) |

**Protokol:** `P{pan×10},T{tilt×10},F{0|1}` formatı (örn: `P450,T-150,F0`)

---

### 📂 `stages/` — Aşama Motorları

Tüm stage'ler `BaseStageEngine`'den türer ve `run()` generator'ı ile çalışır.

#### `base_stage.py` (237 satır)
- **Sınıf:** `BaseStageEngine` — Tüm stage'lerin ortak atasıdır
- **İçerdikleri:**
  - YOLO detector başlatma
  - Kamera açma / frame okuma
  - Nişangah çizimi, FPS hesaplama, overlay
  - `update_turret_ibvs()` → Ana kontrol döngüsü:
    1. Lead prediction (hedefin önüne nişan al)
    2. Normalize hata hesaplama
    3. Dead zone filtresi
    4. P kontrol → açı delta
    5. Feedforward (hedef hızını motor komutuna ekle)
    6. Rate limiter (maks 5°/frame)
    7. Arduino'ya gönder
  - Size classifier entegrasyonu

#### `stage1.py` (237 satır)
- **Sınıf:** `Stage1Engine(BaseStageEngine)`
- **Görev:** Hareketli hedef takibi
- YOLO ile balon algılama → Kalman filtre ile takip
- Tracking açıp kapatılabilir (buton ile)
- Gölge filtresi (LAB L kanalı)
- Pozisyon sıçraması filtreleme (150px eşik)
- **LOCKED** durumu: hedef nişangahın 20px yakınında × 5 frame

#### `stage2.py` (323 satır)
- **Sınıf:** `Stage2Engine(BaseStageEngine)`
- **Görev:** Dost/düşman ayrımı
- Stage 1'in tüm özellikleri + LAB renk sınıflandırma
- Kırmızı = düşman, Mavi = dost
- Renk onaylama: 5 frame içinde 3 kez aynı renk → onaylanmış
- Birden fazla hedef: en büyük düşman seçilir

#### `stage3.py` (362 satır)
- **Sınıf:** `Stage3Engine(BaseStageEngine)`
- **Görev:** Angajman sistemi
- Sticker renk sınıflandırma (sıralama bilgisi)
- OCR ile harf okuma (A/B)
- Platform açılarına dönme (sol/sağ platform)
- Ateş sekansı: Kilit → OCR → Platform dön → Ateş → Sonraki hedef
- Çoklu hedef sıralama (merkeze yakınlık)

#### `ocr_worker.py` (67 satır)
- **Sınıf:** `OcrWorker(QThread)`
- Ayrı thread'de Tesseract OCR çalıştırır
- Balon üzerindeki A/B harflerini okur
- Ön işleme: Grayscale → 2× büyüt → GaussianBlur → Otsu threshold
- Sonucu `result_ready` sinyali ile Stage3'e bildirir

---

### 📂 `classify/` — Sınıflandırma Modülleri

#### `lab_color.py` (127 satır)
- **Sınıf:** `LabColorBalloon`
- **Kullanım:** Stage 2'de balon rengi tespiti
- LAB renk uzayında A-B kanallarıyla sınıflandırma
- `color_calibration.json`'dan kalibrasyon yükler
- Desteklenen renkler: Red, Blue, Green, Unknown
- Merkez + yarıçap bazlı Öklid mesafe ile karar

#### `sticker_lab_color.py` (93 satır)
- **Sınıf:** `StickerLabClassifier`
- **Kullanım:** Stage 3'te sticker renk okuma
- `LabColorBalloon` ile aynı mantık, sticker'a uyarlanmış

#### `size_classifier.py` (47 satır)
- **Sınıf:** `SizeClassifier`
- BBox alan eşiği ile büyük/küçük ayrımı (varsayılan: 10000px²)
- `calibrate_threshold()` ile otomatik eşik ayarı

---

### 🎮 `arayuz.py` — Ana GUI (Detay)

**~1050 satır, PyQt6 tabanlı.**

| Bileşen | Satır Aralığı | İşlev |
|---------|---------------|-------|
| `Stage1Worker` | 105-143 | Stage 1 thread (QThread) |
| `Stage2Worker` | 146-173 | Stage 2 thread |
| `Stage3Worker` | 176-223 | Stage 3 thread + OCR entegrasyonu |
| `ColorCalibrationWidget` | 229-371 | Renk kalibrasyon arayüzü |
| `MainWindow` | 372+ | Ana pencere, video paneli, kontrol butonları |

**GUI Özellikleri:**
- Canlı video + overlay (FPS, stage, hedef bilgisi, nişangah)
- Tab ile Stage 1/2/3 geçişi
- Başlat/Durdur/Track/Fire butonları
- Telemetri paneli
- Log paneli
- Klavye kısayolları: S(Start), F(Fire), N(Next), Enter
- Renk kalibrasyon tab'ı (tıkla-örnekle-kaydet)

---

### ⚡ Arduino Firmware (`arduınoya_atılacak.cpp`)

**318 satır, AccelStepper kütüphanesi.**

| Bölüm | İşlev |
|-------|-------|
| Pin tanımları | Pan: 5/6/7, Tilt: 2/3/4, Joy: A0/A1/D8, Lazer: D9 |
| Kalibrasyon | Pan: 26.67 pulse/°, Tilt: 8.89 pulse/° |
| `setup()` | Motor init, joystick init |
| `loop()` | Mod kontrolü → otonom veya manuel |
| `checkSerial()` | Seri komut parse (char buffer, String yok) |
| `processCmd()` | Komut işleme: ?, H, M0/M1, P...T...F... |
| `runAutonomous()` | `moveTo()` ile pozisyon kontrolü |
| `runManual()` | `readAxis()` + `runSpeed()` ile joystick kontrolü |
| `checkButton()` | Joystick butonu ile motor enable/disable |

**Seri Protokol:**
```
Python → Arduino:  P450,T-150,F0  |  M1  |  M0  |  H  |  ?
Arduino → Python:  OK,P450,T-150  |  S,1,450,-150,0,1  |  OK,M1
```

---

## 🔄 Veri Akışı (Bir Frame'in Yolculuğu)

```
1. Kamera → frame (640×480 BGR)
2. YoloDetector.track_video() → [id, x1, y1, x2, y2, conf, class]
3. TargetKalman.predict(dt) → tahmini (x, y) ve (vx, vy)
4. TargetKalman.correct(x_meas, y_meas) → düzeltilmiş konum/hız
5. BaseStageEngine.update_turret_ibvs():
   a. Lead prediction: target_x + vx × LEAD_TIME_S
   b. Error: (predicted - center) / frame_size  → normalize
   c. Dead zone filtresi
   d. IBVSController.compute() → dpan, dtilt
   e. Feedforward ekleme
   f. Rate limiter (MAX_DELTA_DEG)
   g. ArduinoInterface.update_angles_smooth(dpan, dtilt)
6. Arduino → Step motorlar hareket eder
7. Sonraki frame → 1'e dön (closed loop)
```

---

## 🚀 Çalıştırma

```bash
# 1. Arduino'ya firmware yükle (Arduino IDE)
# 2. Arduino IDE'yi kapat (port serbest kalsın)
# 3. Python venv aktif et
# 4. Çalıştır:
python arayuz.py
```

**Port ayarı:** `config/system_config.py` → `ARDUINO_PORT = "COM4"`
