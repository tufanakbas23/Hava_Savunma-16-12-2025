# Hava Savunma Sistemi - Teknik Dokümantasyon

## 1. Genel Bakış

Bu sistem, YOLO tabanlı nesne algılama ve pan-tilt taret kontrolü kullanarak balonları otomatik olarak tespit eden, takip eden ve angaje eden bir hava savunma sistemidir.

### Sistem Bileşenleri

| Bileşen | Açıklama |
|---------|----------|
| **Kamera** | USB kamera (640×480, 30fps) |
| **Bilgisayar** | Python + YOLO modeli çalıştıran PC |
| **Arduino** | Pan-tilt taret motorlarını kontrol eden mikrodenetleyici |
| **Pan-Tilt Taret** | 2 eksenli step motor sistemi (Pan: ±135°, Tilt: ±30°) |
| **Lazer** | Hedef işaretleme/ateş mekanizması |

### Çalışma Prensibi

```
Kamera → YOLO Algılama → Kalman Takip → P Kontrol → Arduino → Motorlar
  ↑                                                              ↓
  └──────────── Görsel Geri Besleme (Closed Loop) ◄──────────────┘
```

---

## 2. Yazılım Mimarisi

### 2.1 Dosya Yapısı

```
yolo8_proje_goruntu/
├── arayuz.py                  # Ana GUI (PyQt6)
├── arduınoya_atılacak.cpp     # Arduino firmware
├── balon_baska.pt             # YOLO modeli
│
├── config/
│   └── system_config.py       # Tüm ayarlanabilir parametreler
│
├── detector/
│   └── yolo_detector.py       # YOLO sarmalayıcı
│
├── control/
│   ├── ibvs_controller.py     # P kontrol (Kp × error)
│   └── kalman_tracker.py      # Kalman filtre (pozisyon + hız tahmini)
│
├── hardware/
│   └── arduino_interface.py   # Arduino seri port haberleşme
│
├── stages/
│   ├── base_stage.py          # Ortak kontrol mantığı
│   ├── stage1.py              # Hareketli hedef takibi
│   ├── stage2.py              # Dost/düşman ayrımı
│   └── stage3.py              # Angajman sistemi
│
└── classify/
    └── sticker_lab_color.py   # Renk sınıflandırma (LAB)
```

### 2.2 Stage Mimarisi

Sistem 3 aşamalı çalışır:

#### Stage 1 - Hareketli Hedef Takibi
- YOLO ile balon algılama
- Kalman filtre ile pozisyon/hız tahmini
- P kontrol + Feedforward ile takip
- Lead prediction ile hedefin ilerisine nişan alma
- Nişangah balonun üzerine gelince **LOCKED** durumu

#### Stage 2 - Dost/Düşman Ayrımı
- Stage 1'in tüm özellikleri + renk tespiti
- LAB renk uzayında balon rengi sınıflandırma
- Kırmızı = düşman, Mavi = dost
- Birden fazla frame'de renk onaylama (güvenilirlik)

#### Stage 3 - Angajman Sistemi
- Sticker renk okuma (OCR)
- Platform açılarına dönme
- Ateş sekansı yönetimi
- Çoklu hedef sıralama

---

## 3. Kontrol Sistemi

### 3.1 Kontrol Akışı (Her Frame)

```
1. Kamera frame alır
2. YOLO balon algılar → (x, y, w, h)
3. Kalman filtre güncellenir → (x, y, vx, vy)
4. Lead Prediction: hedefin ilerisine nişan al
   predicted_x = target_x + vx × LEAD_TIME_S
5. Hata hesapla: error = predicted - kamera_merkezi
6. P Kontrol: dpan = Kp × error_norm
7. Feedforward: hedef hızını motor komutuna ekle
8. Rate Limiter: ani hareketleri sınırla
9. Arduino'ya gönder: "P450,T-150,F0"
10. Arduino motorları hareket ettirir
```

### 3.2 Kontrol Parametreleri (system_config.py)

| Parametre | Değer | Açıklama |
|-----------|-------|----------|
| `KP` | 1.5 | Oransal kazanç |
| `LEAD_TIME_S` | 0.20 | Öngörü süresi (saniye) |
| `DEAD_ZONE_NORM` | 0.04 | Merkez bölge (hareket etme) |
| `MAX_DELTA_DEG` | 5.0 | Frame başına maks açı değişimi |
| `FEEDFORWARD_GAIN` | 1.0 | Hedef hız takip katsayısı |
| `PROCESS_NOISE` | 0.3 | Kalman reaktifliği |
| `LOCK_TOLERANCE_PIX` | 20 | Kilit mesafesi (piksel) |

### 3.3 Kalman Filtre

4 durumlu (x, y, vx, vy) Kalman filtre:
- **Tahmin**: Hedefin bir sonraki konumunu öngörür
- **Güncelleme**: YOLO ölçümüyle hata düzeltir
- **Hız çıkışı**: Feedforward ve lead prediction için kullanılır

---

## 4. Arduino Firmware

### 4.1 Donanım

| Bileşen | Özellik |
|---------|---------|
| Pan motor | 3:1 kayış, 26.67 pulse/derece |
| Tilt motor | 1:1 kayış, 8.89 pulse/derece |
| Microstep | 1/16 (3200 pulse/tur) |
| Hız | 120 derece/saniye |
| İletişim | Serial 115200 baud |

### 4.2 Çalışma Modları

| Mod | Kontrol | Açıklama |
|-----|---------|----------|
| **Manuel** | Joystick | Analog joystick ile motorları kontrol et |
| **Otonom** | Python | Seri port komutlarıyla pozisyon kontrolü |

### 4.3 Seri Protokol

```
Python → Arduino:
  P450,T-150,F0    (Pan=45.0°, Tilt=-15.0°, Fire=off)
  M1                (Otonom moda geç)
  M0                (Manuel moda geç)
  H                 (Home pozisyonu)
  ?                 (Durum sorgula)

Arduino → Python:
  OK,P450,T-150     (Pozisyon cevabı)
  S,1,450,-150,0,1  (Durum: mod,pan,tilt,fire,enabled)
  OK,M1             (Mod değişti)
```

---

## 5. Arayüz (GUI)

PyQt6 tabanlı arayüz:

- **Video paneli**: Canlı kamera görüntüsü + overlay
- **Stage seçimi**: Tab ile Stage 1/2/3 geçişi
- **Kontrol butonları**: Başlat, Durdur, Track, Fire
- **Telemetri**: FPS, hedef bilgisi, lock durumu
- **Log paneli**: Sistem mesajları

### Overlay Bilgileri
- FPS (hareketli ortalama)
- Stage numarası ve modu
- Track durumu (ON/OFF)
- Cycle durumu (ARMED/WAIT)
- Nişangah (+)
- Hedef bounding box ve ID

---

## 6. Kurulum ve Çalıştırma

### Gereksinimler
```
Python 3.10+
PyQt6
ultralytics (YOLO)
opencv-python
numpy
pyserial
```

### Çalıştırma
```bash
# 1. Arduino kodunu yükle (Arduino IDE ile)
# 2. Arduino IDE'yi kapat
# 3. Python arayüzü başlat
python arayuz.py
```

### Parametre Ayarlama
Tüm ayarlar `config/system_config.py` dosyasında:
- **Yavaş takip** → `KP` artır
- **Salınım** → `KP` azalt veya `DEAD_ZONE_NORM` artır
- **Geride kalma** → `FEEDFORWARD_GAIN` ve `LEAD_TIME_S` artır
- **Motor hızı** → Arduino kodunda `TARGET_SPEED_DEG_PER_SEC` değiştir
