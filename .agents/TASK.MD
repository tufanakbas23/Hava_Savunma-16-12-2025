# Hava Savunma Sistemi — Görev Listesi (Birleştirilmiş)

## PHASE 1: Core Infrastructure & Hardware Interfaces

### 1.1 Logitech Camera Wrapper (`camera/logitech_camera.py`) 
- [ ] `LogitechCamera` sınıfı oluştur
- [ ] `open()` metodu — `cv2.VideoCapture(CAMERA_LOGITECH_ID, cv2.CAP_DSHOW)`, 1280×720
- [ ] `close()` ve `grab_frame()` metodları
- [ ] `is_open` property
- [ ] `CAP_PROP_BUFFERSIZE = 1` ayarla (bayat kare önleme)
- [ ] Thread-safe erişim (lock mekanizması)

### 1.2 ZED Camera Wrapper (`camera/zed_camera.py`) 
- [ ] Hali hazırda ana dizindeki dağınık `zed.py` yerine, Logitech ile aynı API yapısına sahip bir wrapper sınıf (`ZedCamera`) oluşturulacak.
- [ ] `open()`, `close()`, `grab_frame()` metodları bulunacak. ZED API üzerinden alınacak RGB + Depth eşzamanlı dönecek.
- [ ] `is_open` state takibi yapılacak. Her 3 aşamada Logitech ve ZED her an aktif kalmaya devam edecek.

### 1.3 Benewake TF03 LiDAR Parser (`hardware/lidar_interface.py` İPTAL - ARDUINO ÜZERİNDEN OKUNACAK)
- [ ] Daha önce doğrudan PC'ye bağlı olarak planlanan LiDAR parser kodları tamamen iptal edildi.
- [ ] LiDAR, Arduino'ya bağlıdır ve okumalar donanım seviyesinde (`Gömülü/stnm_lidar.cpp`) yapılmaktadır.
- [ ] Python tarafı (PC), LiDAR mesafe verisini doğrudan Arduino'nun gönderdiği `StnmTelemetryPacket_t` paketinden okuyacaktır (Bkz: Phase 7).
- [ ] Arka plan thread ile COM port polling işlemi sadece ana UART bağlantısı üzerinden tek kanaldan yürütülecektir.

### 1.4 Donanım Atış Sistemi Entegrasyonu
- [ ] Arduino C++ (`Gömülü/STNM_PTU_System.ino`): `fireWeapon()` fonksiyonunda relay/solenoid/lazer tetik uygulaması tamamlanacak (şu an sadece state=FIRING yapıyor, yorum satırı: `// future: trigger relay / solenoid / laser`)
- [ ] Ateşleme `STNM_CMD_FIRE` komutu ile tetiklenir → `StnmCommandPacket_t.fire = 1`
- [ ] `arduino_interface.py`: Binary paket ile `fire` bayrağını set et (`StnmCommandPacket_t` üzerinden)
- [ ] ~~`arduınoya_atılacak.cpp` ARTIK KULLANILMIYOR~~ — Yeni firmware: `Gömülü/STNM_PTU_System.ino`

### 1.5 Matematiksel Dönüşümler (`base_stage.py`'ye ekle)
- [ ] `calculate_parallax_shift(depth_m, offset_x_mm, offset_y_mm) → (shift_x_px, shift_y_px)`
- [ ] Formül: `shift = (offset / Z) * focal_length_px`
- [ ] Başlangıç: `O_x ≈ 200mm`, `O_y ≈ 0mm`, `system_config.py`'den okunsun
- [ ] Açısal hız dönüşüm fonksiyonu (Zaman tabanlı):
  - [ ] `omega_x = ((vx_px / frame_w) * ZED_HFOV_DEG) / d_time`
  - [ ] `omega_y = ((vy_px / frame_h) * ZED_VFOV_DEG) / d_time` (d_time = iki kare arası geçen süre)
  - [ ] ZED FOV sabitleri `system_config.py`'den (HFOV ≈ 110°, VFOV ≈ 70°)
- [ ] Kamera-Taret Piksel/Derece Dönüşümü (Kinematik):
  - [ ] `pixels_to_degrees(x_px, y_px) → (pan_deg, tilt_deg)` dönüşümü yazılacak.
  - [ ] ZED kamerası taret sisteminin yatay ekseninden yaklaşık **30 cm aşağıda** konumlandırılacağı için, tilt açısı hesaplanırken bu dikey ofset (vertical offset) kinematik modele yansıtılacak (Parallax_Y değişkeni ile trigonometrik düzeltme).

### 1.6 YOLO Model Yöneticisi (Engine Manager)
- [ ] Mevcut olan batch=2 ayarlı engine modelini yükle: `yolo/balon_baska.engine` — Tüm stage'ler

### 1.7 `system_config.py` Konfigürasyonları
- [ ] Bağlantı: `ARDUINO_PORT="COM4"`, `ARDUINO_BAUDRATE=115200` 
- [ ] Kamera ID'leri: `CAMERA_LOGITECH_ID=1`, `CAMERA_ZED_ID=0`
- [ ] Parallax: `PARALLAX_OFFSET_X_MM=200.0`, `PARALLAX_OFFSET_Y_MM=0.0`
- [ ] Atış süresi: `FIRE_DURATION_MS = 200`
- [ ] ZED FOV: `ZED_HFOV_DEG=110.0`, `ZED_VFOV_DEG=70.0`
- [ ] Stage bazlı sınıf listeleri:
  - [ ] `STAGE1_CLASSES = ["missile", "helicopter", "f16", "uav"]`
  - [ ] `STAGE2_CLASSES = ["missile", "uav"]`
  - [ ] `STAGE3_CLASSES = ["missile", "helicopter", "f16", "uav"]`
- [ ] Range Gating: F16 (10–15m), Missile (10–15m), Mini/Micro UAV (0–15m), Helicopter (5–15m)
- [ ] `balloon` sınıfı tüm listelerden çıkarıldı mı kontrol
- [ ] Timing parametreleri (Frame bazlı DEĞİL, Zaman / MS bazlı kullanılmalı):
  - [ ] `HANDOVER_TIMEOUT_MS = 1000` (Eski 30 frame yerine)
  - [ ] `PID_REACQUIRE_TIMEOUT_MS = 500` (Eski 15 frame yerine)
  - [ ] `COLOR_VERIFY_TIMEOUT_MS = 1000` (Eski 30 frame yerine, ancak içindeki karar mekanizması frame sayısına bağlı)
  - [ ] `COLOR_CONFIRM_RATIO = (3, 5)` (Son 5 karenin en az 3'ünde hedef kırmızıyken onay ver)
  - [ ] `POST_ENGAGEMENT_DELAY_MS = 500`
  - [ ] `SLEW_STABILIZE_MS = 100`
  - [ ] (Derinlik tespitinde ZED her zaman ana güven kaynağıdır, LiDAR versin veya vermesin varsayılan ZED'dir)

---

## PHASE 2: Stage 1 — Manuel Mod (Statik Hedefler)

### 2.1 Stage 1 Engine (`stage1.py` yeniden yaz)
- [ ] ZED + Logitech birlikte açılacak (batch=2)
- [ ] YOLO: `yolo/balon_baska.engine` (batch=2)
- [ ] ZED depth bilgisi overlay'de gösterilecek
- [ ] Aktif sınıflar: `missile`, `helicopter`, `f16`, `uav`
- [ ] LiDAR her frame okunacak
- [ ] Otomatik nişan/ateş YAPILMAYACAK

### 2.2 Kırmızı Piksel Hedefleme
- [ ] Bbox ROI kırp
- [ ] CIELAB renk filtreleme ile kırmızı piksel maskesi
- [ ] Kırmızı piksel kümesinin merkez noktasını hesapla
- [ ] Bu merkezi nişan noktası olarak overlay'e çiz

### 2.3 Overlay
- [ ] Bounding Box + Sınıf adı + Confidence
- [ ] ZED Depth (m)
- [ ] LiDAR mesafesi (m)
- [ ] Kırmızı hedef noktası (nişan)
- [ ] FPS

### 2.4 Manuel Fire Workflow
- [ ] Akış kontrolü `frame_ready` sinyali ile UI'ya gönderilecek
- [ ] Joystick USB üzerinden **PC'ye** bağlanacak ve **PyGame** (`pygame.joystick`) ile okunacak (COM port değil — COM port joystick diye bir şey yok)
- [ ] Arayüzde Joystick eksenli veri yorumlanacak (Hız / Yön) → Python `STNM_CMD_SET_PANTILT` binary paketi ile Arduino'ya gönderir
- [ ] Arduino Manuel mod: `STNM_CMD_SET_MODE` + `mode = STNM_MODE_MANUAL (1)` (`M0` ASCII komutu artık **geçersiz** — yeni Gömülü protokolünde yok)
- [ ] Kullanıcı joystick ile pan/tilt açısını Python üzerinden ayarlar (Gömülü'de joystick okuması YOK, Python okur ve binary paket gönderir)
- [ ] Tetikleme: joystick butonu veya UI "Fire" butonu → `STNM_CMD_FIRE` paketi gönderilir (`fire=1`); `FIRE_DURATION_MS` Python tarafında sinyal süresi olarak kullanılır (Arduino'da süre uygulaması `fireWeapon()` içinde yapılacak — şu an eksik)

---

## PHASE 3: Stage 2 — Otonom Swarm Takibi

> **3 koldan 3 hedef** gelecek (her kolda 1 tane). Mesafeye bakılmaksızın imha edilecek.

### 3.1 Donanım Başlatma
- [ ] ZED + Logitech aç + Arduino Serial bağlantı başlat (LiDAR telemetrisi Arduino'dan `StnmTelemetryPacket_t` ile gelecek)
- [ ] YOLO: `yolo/balon_baska.engine` (batch=2 yükle)
- [ ] Arduino Otonom mod: `STNM_CMD_SET_MODE` + `mode = STNM_MODE_AUTONOMOUS (2)` binary paket ile (`M1` ASCII komutu artık **geçersiz**)
- [ ] Aktif sınıflar: `missile`, `uav`

### 3.2 Dış Döngü — Spotter (ZED)
- [ ] Her frame: ZED + Logitech → YOLO batch=2
- [ ] ZED: ByteTrack (`model.track(tracker="bytetrack.yaml")`)
- [ ] Her `track_id` için depth geçmişi (`deque(timestamp, depth_m)`)
- [ ] Yaklaşma hızı: `vz = Δdepth / Δtime`
  - [ ] Ölçüm dalgalanmasını/sensör gürültüsünü engellemek için son 10 karelik (frame) verinin Hareketli Ortalaması (Moving Average) kullanılacak (`DEPTH_HISTORY_SIZE = 10`). (*Not: Algoritma yavaşlamazsa bu sayı korunacak, hız düşerse 5'e çekilecek*)
- [ ] Uzaklaşan hedefleri filtrele (`vz < 0`)
- [ ] Açısal hız: `omega = ((vx_px / W) * ZED_HFOV) / dt`
- [ ] Önceliklendirme: (1) sınıf önceliği, (2) kritik mesafeye ETA, (3) merkeze yakınlık

### 3.3 Spotter → Tracker Handover
- [ ] ZED primary target seç — N ms onay
- [ ] ZED piksel → pan/tilt açı hesapla
- [ ] `arduino.set_absolute_position(pan, tilt)` — slew komutu
- [ ] **Post-slew stabilization (~100ms)** bekle
- [ ] **Logitech frame buffer temizle** (bayat kareler at)
- [ ] **Taze frame al** (yeni grab)
- [ ] Logitech'te hedef ara: sınıf + bbox boyut + merkeze yakınlık
- [ ] **HANDOVER_TIMEOUT_MS süre sınırı** — bulunamazsa spotter'a dön
- [ ] Engine swap YOK (batch=2 sürekli çalışıyor)

### 3.4 İç Döngü — P+Feedforward Takip (Tight Loop)
> ⚠️ Sistem PID **değil**, P kontrol + Feedforward kullanıyor (`ibvs_controller.py`). Ki ve Kd yoktur.
- [ ] Frame al: ZED + Logitech → YOLO batch=2
- [ ] ZED: güncel depth + açısal hız güncelle (feedforward)
- [ ] Logitech: hedef hala var mı kontrolü
- [ ] Pixel error + parallax shift + feedforward → `STNM_CMD_SET_PANTILT` binary paketi → Arduino
- [ ] **Hedef kaybı → Re-acquire:**
  - [ ] Son Arduino komutu dondur
  - [ ] `PID_REACQUIRE_TIMEOUT_MS` bitene kadar yeniden ara
  - [ ] Bulunamazsa → spotter dış döngüsüne dön
- [ ] **LOCKED:** `|error| < threshold` N consecutive ms
- [ ] **LOCKED → hemen FIRE!** → `STNM_CMD_FIRE` paketi gönder (`fire=1`); Arduino `STNM_STATE_FIRING` durumuna geçer
- [ ] LiDAR: Arduino'dan gelen `lidar_distance` (cm cinsinden, `/100.0` ile metreye çevir) overlay ve telemetride gösterile

### 3.5 Post-Engagement Auto-Scan
- [ ] FIRE sonrası ~0.5s bekleme
- [ ] Otomatik spotter fazına dön
- [ ] Hedef vurulduktan veya gözden kaybedildikten sonra Taret (Pan/Tilt motorları) **MERKEZE (Home pozisyonuna) DÖNDÜRÜLMEYECEK**, ZED o anki açısında (son pozisyonunda) taramaya devam edecek.
- [ ] Sonraki en kritik hedefi seç
- [ ] Hedef kalmadıysa → bulunduğu konumdan idle tarama (sağa sola devriye/sweep) başlat

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
- [ ] Renk sınır değerleri donanım tarafında doğrudan koda gömülecek (json dosyası veya konfigürasyon dosyası KULLANILMAYACAK).
- [ ] LAB A/B kanalları: Kırmızı=DÜŞMAN, Mavi=DOST 
- [ ] Sınıflandırma sadece Logitech bbox ROI içinde

### 4.4 Spotter + Dost Blacklist
- [ ] Stage 2 spotter mantığı (Task 3.2)
- [ ] Hedef seçerken blacklist'teki `track_id`'leri atla

### 4.5 Renk Doğrulama Fazı (Handover sonrası, PID öncesi)
- [ ] Handover confirmed → renk doğrulama döngüsüne gir
- [ ] Her ms: bbox ROI → `classify(frame, box)`
- [ ] **Mavi (DOST)** → `track_id` blacklist'e → spotter'a dön
- [ ] **Kırmızı (DÜŞMAN)** → (3/5 ms orantısı) onay → PID'e geç
- [ ] **Belirsiz** → tekrar dene
- [ ] **COLOR_VERIFY_TIMEOUT_MS** → hedefi atla, spotter'a dön

### 4.6 P Kontrol + Range Gate + Post-Engagement
- [ ] P+Feedforward kontrol yapısı Stage 2 ile birebir aynı (Task 3.4, 3.5).
- [ ] **Stage 3 Range Gate Zorunluluğu:** Atış (`STNM_CMD_FIRE`) verilmeden ÖNCE hedefin mesafesi Range Gate sınırları dahilinde olup olmadığı onaylanacak (ZED depth primary kaynak)
- [ ] Stage 3 angajman başlarken `STNM_CMD_SET_MODE` + `STNM_STATE_TRACKING` Arduino'ya bildirilecek (telemetri akışında state doğru görünür)
- [ ] **Önemli:** Mesafe okumasında HER ZAMAN ZED kamerası önceliklidir ve LiDAR veri versin veya vermesin güven kaynağı ZED olacaktır.
- [ ] LiDAR verisi: `StnmTelemetryPacket_t.lidar_distance` cm cinsindendir → Python'da `/100.0` ile metreye çevir
- [ ] ms bazlı re-acquire ve range gate split dinamikleri uygulanacaktır.

### 4.7 OCR Modülünün ve Kalıntılarının Tamamen Temizlenmesi
- [ ] `stages/ocr_worker.py` (veya varsa benzer dosyalar) tamamen silinecek.
- [ ] `arayuz_v5.py` gibi tüm arayüz dosyalarından `OcrWorker` importları kaldırılacak.
- [ ] Ekranda skor, yazı veya OCR'den dönen herhangi bir metni basan UI elementleri (Label vb.) silinecek.
- [ ] Model, tesseract kütüphanesi ve loglama mekanizmalarından OCR kısmı %100 temizlenecek.

---

## PHASE 5: Resilience & Error Handling (En Son)

### 5.1 Try/Catch Sarmalama
- [ ] Kamera açma/grab hataları → otomatik yeniden bağlanma
- [ ] Arduino seri port kopması → reconnect loop + UI uyarı
- [ ] LiDAR artık Arduino'ya bağlı, PC'ye değil → LiDAR kopması Python'u değil Arduino'yu etkiler; Arduino `lidar_distance=0` gönderir, Python bunu "okuma yok" olarak yorumlar
- [ ] YOLO model yükleme hatası → anlamlı hata mesajı, graceful stop
- [ ] Frame işleme hataları → frame atla, döngüye devam
- [ ] Binary paket CRC hatası → paketi at, bir sonraki paketi bekle (parse atla)

### 5.2 Auto-Recovery
- [ ] Sistem hiçbir durumda çökmemeli
- [ ] Kritik donanım hatası → UI'da kırmızı uyarı + log
- [ ] Geçici hatalar → sessizce atla, devam et
- [ ] Hata sonrası tekrar çalışmaya hazır olmalı

---

## PHASE 6: UI — PySide6 Arayüzü (`arayuz_v5.py`)

> Arayüz: C# WinForms projesi **iptal edildi**. Yerine PySide6 ile geliştirilen `arayuz_v5.py` kullanılacak.

### 6.1 PySide6 Arayüz Yapısı
- [ ] `MainWindow` sınıfı etrafında şekillenen, sekmeli (Konrol, Kalibrasyon, PD Ayarları) yapı.
- [ ] ZED ve Logitech kameralarının görüntülerini eşzamanlı gösteren çift video etiketi.
- [ ] `DualCameraWorker` (QThread tabanlı) ile arka planda YOLO inference ve kamera okuma.
- [ ] UI güncellemeleri için Qt sinyal-slot mimarisi (`_frame_signal`, `_log_signal`).

### 6.2 Sistem Kontrolleri ve Telemetri
- [ ] Mod seçimi (Manuel, Sürü, Otonom) butonlarla (`on_stage_btn`).
- [ ] START/STOP butonları işlevsel hale getirilecek.
- [ ] Lidar mesafe ve Sistem Durumu telemetri etiketlerine gerçek zamanlı yansıtılacak.
- [ ] Hata ve durum mesajları için Log Konsolu (QPlainTextEdit) etkin kullanılacak.

---

## PHASE 7: Seri Haberleşme Optimizasyonu (Python ↔ Arduino)

> Amaç: Standart, yavaş ve bloke edici text-tabanlı haberleşmeyi; asenkron, hızlı ve güvenilir binary haberleşmeye çevirmek. `arayuz_v5.py` ve `hardware/arduino_interface.py` (ve Arduino C++ kodları) güncellenecek.

### 7.1 PySide6 Mimarisi (Arka Plan QThread vs UI)
- [ ] Sistem sadece arayüz üzerinden çalışacak (`arayuz_v5.py`), headless çalıştırılma düşünülmüyor.
- [ ] `hardware/arduino_interface.py` çağrıları `QThread` üzerinden yapılacak, Arayüz bloke edilmeyecek.
- [ ] Arka planda çalışan `DualCameraWorker` komutları (hata hesaplamaları, atış vb.) QThread üzerinden doğrudan ana birime iletilecek.
    - [ ] Python tarafı `hardware/arduino_interface.py` içerisinde iletişim için `StnmCommandPacket_t` ve `StnmTelemetryPacket_t` (`Gömülü/inc/stnm_protocol_types.h` dosyasındaki yapılara) tam uyumlu `struct` mantığı kurgulanacak.
    - [ ] Telemetry parse: her 100ms'de Arduino binary paket gönderir → Python `StnmTelemetryPacket_t` parse eder → `pan_current`, `tilt_current`, `lidar_distance`, `system_state` alanları arayüze/gözlemciye aktarılır.

### 7.2 İletişim Hızı (Baud Rate)
- [ ] Baud hızı artırımı YAPILMAYACAK. Varsayılan hız 115200 olarak kalacak.
- [ ] Arduino C++ tarafında `STNM_PTU_System.ino` içerisinde `Serial.begin(115200);` olarak kalacak.

### 7.3 Veri Formatı: Gömülü Struct Yapısı (STNM Protocol)
- [ ] ASCII string gönderimini (Örn: `P100,T200,F0\n`) terk et.
- [ ] Python tarafında `struct` kütüphanesini kullanarak `Gömülü/inc/stnm_protocol_types.h` ile 1:1 eşleşen (SOP=0xAA) paketleri oluştur.

**Paket boyutları (`#pragma pack(1)` ile):**

| Paket | Format | Boyut |
|-------|--------|-------|
| `StnmCommandPacket_t` | `'<BBBBBhhBH'` | **12 byte** |
| `StnmTelemetryPacket_t` | `'<BBBhhHBH'` | **12 byte** |
| `StnmAckPacket_t` | `'<BBBBBH'` | **7 byte** |

- `StnmCommandPacket_t` → `struct.pack('<BBBBBhhBH', SOP, TYPE, SEQ, CMD, MODE, PAN, TILT, FIRE, CRC)`
  - `pan`, `tilt` = `int16_t` → `h` (signed short, **derece cinsinden**)
  - CRC: `serial.read(12)` ile sabit boyutlu oku
- `StnmTelemetryPacket_t` (Arduino→Python) → `serial.read(12)` → `struct.unpack('<BBBhhHBH', data)`
  - `(sop, type, seq, pan_current, tilt_current, lidar_distance, system_state, crc)`
  - ⚠️ `lidar_distance` birimi **santimetre** → Python'da `/100.0` ile metreye çevir
- ⚠️ **CRC Endian Notu:** `stnm_protocol.cpp` `STNM_VerifyPacket` fonksiyonunda `(packet[length-2] << 8) | packet[length-1]` ile big-endian okuyor, fakat struct little-endian mimaride `uint16_t` olarak yazılıyor. Python CRC doğrulamasını uygulamadan önce bu davranışı test et; gerekirse iki byte'ı swap et.
- Arduino'da `STNM_PTU_System.ino` içindeki ASCII test modu (`p/o/t/g/r` char komutları) production build'den **kaldırılacak**

- [ ] Arduino'dan dönen pozisyon, durum ve **LiDAR mesafe okumaları** (`uint16_t lidar_distance`, cm) `StnmTelemetryPacket_t` içerisinden parse edilerek arayüze aktarılacak.

### 7.4 Framing (Paket Çerçeveleme) ve Güvenilirlik
- [ ] Gömülü protokole (STNM Protocol) sadık kalınacak: Başlangıç biti `SOP = 0xAA`.
- [ ] CRC algoritması: CRC-16/IBM (polynomial 0xA001, init 0xFFFF) — `stnm_crc.cpp` ile aynı algoritma Python'da implement edilecek.
- [ ] Her alınan telemetry paketinde CRC doğrulaması yapılacak; hatalı paket atılacak.
- [ ] Arduino'dan Python tarafına iki yönlü güvenlik için `StnmAckPacket_t` (7 byte) parse edilecek ve komut başarı/hata durumu loglanacak.

