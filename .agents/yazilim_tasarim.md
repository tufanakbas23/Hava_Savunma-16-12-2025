# Yazılım Tasarımı

## 1. Yazılım Bileşenleri

Sistemin yazılım katmanı dört ana bileşen grubundan oluşmaktadır: donanım arayüz katmanı, algılama ve takip katmanı, karar ve kontrol katmanı ve kullanıcı arayüzü katmanı.

### 1.1 Donanım Arayüz Katmanı

**ZED Kamera Modülü** — ZED 2 stereo kameranın başlatılması, kare yakalanması ve stereo derinlik haritasının üretilmesinden sorumludur. ZED SDK üzerinden çalışır. Derinlik modu olarak hız-doğruluk dengesi gözeten PERFORMANCE modu tercih edilmiştir. Çıktı olarak BGR renk görüntüsü ve piksel bazında metre cinsinden derinlik haritası üretir. Temel gereksinimler: USB 3.0 bağlantısı, CUDA destekli GPU ve ZED SDK 4.x kurulumu.

**Logitech Kamera Modülü** — Logitech C920 kameranın başlatılması ve kare yakalanmasından sorumludur. OpenCV kütüphanesi üzerinden DirectShow backend ile çalışır. Tek çıktı BGR renk görüntüsüdür, derinlik bilgisi üretmez. Temel gereksinimler: OpenCV 4.x kurulumu ve DirectShow desteği.

**Arduino Haberleşme Modülü** — Pan-tilt mekanizmasını kontrol eden Arduino Mega ile UART seri port üzerinden haberleşmeyi yönetir. İki farklı komut formatı gönderir: pozisyon modu için mutlak açı paketi ve otonom mod için normalize hata paketi. Arduino'dan gelen durum bilgilerini okur ve ayrıştırır. Temel gereksinimler: PySerial 3.5 kütüphanesi ve COM4 port erişimi.

**LiDAR Haberleşme Modülü** — Benewake TF03 tek boyutlu LiDAR sensöründen sürekli mesafe verisi okur. Arka plan iş parçacığında çalışarak dokuz baytlık ikili paketleri ayrıştırır, sağlama toplamını doğrular ve son geçerli mesafe değerini metre cinsinde tutar. Temel gereksinimler: UART seri port erişimi (COM3, 115200 baud).

### 1.2 Algılama ve Takip Katmanı

**Nesne Tespit Modülü** — YOLOv8 derin öğrenme modeli ile video karelerindeki hava hedeflerini tespit eder. Model, NVIDIA TensorRT formatında derlenerek GPU üzerinde çalıştırılır. Statik batch boyutu iki olarak ayarlanmıştır; her çıkarım çağrısında ZED ve Logitech görüntüleri birlikte işlenir. Çıktı olarak her tespit için sınırlayıcı kutu koordinatları, sınıf kimliği ve güven yüzdesi üretir. Temel gereksinimler: NVIDIA GPU, CUDA Toolkit, TensorRT 8.x, Ultralytics kütüphanesi.

**Çoklu Nesne Takip Modülü** — ByteTrack algoritması ile ardışık karelerdeki tespitleri birbirine bağlar ve her hedefe kalıcı bir takip numarası atar. Kısa süreli kayıplarda hedefi kaybetmez. Ultralytics kütüphanesinin yerleşik takip fonksiyonu üzerinden çalışır. Temel gereksinimler: Ultralytics kütüphanesi.

### 1.3 Karar ve Kontrol Katmanı

**Hedef Önceliklendirme Modülü** — Sahnede birden fazla hedef bulunduğunda en kritik olanını seçer. Sınıf önceliği, yaklaşma hızı ve görüntü merkezine yakınlık kriterlerini değerlendirerek birincil hedefi belirler. Kendi geliştirmemizdir.

**Handover (Geçiş) Modülü** — ZED kamerasında tespit edilen hedefin piksel koordinatlarından pan-tilt açılarını hesaplar, Arduino'ya konumlanma komutu gönderir ve Logitech kamerasında hedefi yeniden bulur. Kendi geliştirmemizdir.

**PD Kontrol Modülü** — Logitech görüntüsündeki hedefin çerçeve merkezinden sapmasını piksel hatası olarak hesaplar, buna geometrik sapma düzeltmesi ve açısal hız tahmini ekleyerek Arduino'ya iletir. Arduino firmware'ında orantısal-türev (PD) kontrol algoritması çalışarak motorları hedefe yönlendirir. Kendi geliştirmemizdir.

**Renk Sınıflandırma Modülü** — Yalnızca üçüncü aşamada aktiftir. Hedefin görüntü bölgesini LAB renk uzayına dönüştürerek dost (mavi) veya düşman (kırmızı) olarak sınıflandırır. Dost hedefler kara listeye eklenerek bir daha seçilmez. Kendi geliştirmemizdir.

### 1.4 Kullanıcı Arayüzü Katmanı

**C# WinForms Arayüzü** — .NET 6.0 ile geliştirilmiş masaüstü uygulamasıdır. İki kameradan gelen işlenmiş görüntüleri gösterir, aşama seçimi ve ateş komutu gibi operatör girdilerini alır. Python backend ile doğrudan ağ bağlantısı kullanmaz; haberleşme tamamen Memory-Mapped File üzerinden gerçekleşir. Temel gereksinimler: .NET 6.0 Runtime, Windows 10/11.

---

## 2. Yazılım Arayüzleri

Sistemdeki yazılım bileşenleri birbirleriyle üç farklı arayüz tipi üzerinden haberleşmektedir.

### 2.1 Memory-Mapped File (Paylaşımlı Bellek) Arayüzü

Python backend ile C# kullanıcı arayüzü arasındaki tüm veri alışverişi Windows işletim sisteminin sağladığı Memory-Mapped File mekanizması üzerinden gerçekleştirilmektedir. Bu mekanizma, iki farklı sürecin aynı bellek bölgesini okumasına ve yazmasına olanak tanır. Ağ soketi veya dosya tabanlı haberleşmeye kıyasla çok daha düşük gecikme sağlar.

Python'dan arayüze doğru iki bellek alanı kullanılmaktadır. Birincisi ham kamera görüntüsünü taşıyan alan, ikincisi ise üzerinde tespit sonuçları ve durum bilgileri çizilmiş işlenmiş görüntüyü taşıyan alandır. Arayüzden Python'a doğru ise beş farklı bellek alanı bulunmaktadır: aşama seçimini ileten alan, ateş komutunu ileten alan, ateşe devam komutunu ileten alan, angajman kabul bilgisini ileten alan ve yasaklı bölge açı aralığını ileten alan.

### 2.2 UART Seri Port Arayüzü

Python backend ile Arduino arasındaki haberleşme UART seri port protokolü üzerinden gerçekleştirilmektedir. İki farklı paket formatı tanımlanmıştır. Pozisyon modu paketleri mutlak açı değerlerini iletir ve handover geçişinde kullanılır. Hata modu paketleri ise normalize piksel hata değerlerini iletir ve otonom takip döngüsünde kullanılır. Arduino'dan Python'a doğru ise durum sorgusu yanıtları ve komut onayları gönderilmektedir.

LiDAR sensörü ile haberleşme de UART üzerinden ancak ayrı bir port üzerinden gerçekleştirilmektedir. LiDAR, sürekli olarak dokuz baytlık ikili paketler göndermekte ve Python arka plan iş parçacığı bu paketleri dinleyerek mesafe bilgisini güncellemektedir.

### 2.3 Modüller Arası Dahili Arayüzler

Python backend içindeki modüller arasındaki haberleşme doğrudan fonksiyon çağrıları ve nesne referansları üzerinden gerçekleşmektedir. Tüm modüller ortak bir yapılandırma dosyasından parametrelerini okur. Her operasyonel aşama, bir temel sınıftan miras alarak ortak işlevleri paylaşır ve yalnızca kendine özgü mantığı ekler.

---

## 3. Yazılım Arayüz Şeması

```mermaid
flowchart TB
    subgraph UI["Kullanıcı Arayüzü (C# .NET 6.0)"]
        FORM["WinForms\nGörüntü + Butonlar + Log"]
    end

    subgraph MMF["Memory-Mapped File Arayüzü"]
        M1["raw_frame\n(Görüntü)"]
        M2["processed_frame\n(Görüntü)"]
        M3["SystemMode\n(Komut)"]
        M4["FireDataMemory\n(Komut)"]
    end

    subgraph PY["Python Backend"]
        subgraph DONANIM["Donanım Arayüz Katmanı"]
            ZED_M["ZED Kamera\nModülü"]
            LOG_M["Logitech Kamera\nModülü"]
            ARD_M["Arduino\nHaberleşme Modülü"]
            LID_M["LiDAR\nHaberleşme Modülü"]
        end
        subgraph ALGILAMA["Algılama ve Takip Katmanı"]
            YOLO_M["Nesne Tespit\nModülü (YOLOv8)"]
            BT_M["Çoklu Nesne\nTakip Modülü"]
        end
        subgraph KONTROL["Karar ve Kontrol Katmanı"]
            PRIO_M["Hedef\nÖnceliklendirme"]
            HO_M["Handover\nModülü"]
            PD_M["PD Kontrol\nModülü"]
            COLOR_M["Renk\nSınıflandırma"]
        end
        CONFIG["Yapılandırma\nDosyası"]
    end

    subgraph HW["Donanım"]
        ZED_HW["ZED 2"]
        LOG_HW["Logitech C920"]
        ARD_HW["Arduino Mega"]
        LID_HW["TF03 LiDAR"]
        PT_HW["Pan-Tilt + Lazer"]
    end

    %% UI ↔ MMF ↔ Python
    FORM <-- "okuma" --> M1
    FORM <-- "okuma" --> M2
    FORM -- "yazma" --> M3
    FORM -- "yazma" --> M4
    M1 <-- "yazma" --- PY
    M2 <-- "yazma" --- PY
    M3 -- "okuma" --> PY
    M4 -- "okuma" --> PY

    %% Donanım ↔ Modüller
    ZED_HW -- "USB 3.0" --> ZED_M
    LOG_HW -- "USB 2.0" --> LOG_M
    ARD_HW <-- "UART COM4" --> ARD_M
    LID_HW -- "UART COM3" --> LID_M

    %% Arduino → Fiziksel
    ARD_HW -- "Step/Dir" --> PT_HW

    %% Modüller arası
    ZED_M -- "frame + depth" --> YOLO_M
    LOG_M -- "frame" --> YOLO_M
    YOLO_M -- "tespitler" --> BT_M
    BT_M -- "track_id + hız" --> PRIO_M
    PRIO_M -- "birincil hedef" --> HO_M
    HO_M -- "açı komutu" --> ARD_M
    HO_M -- "hedef bilgisi" --> PD_M
    PD_M -- "hata paketi" --> ARD_M
    LID_M -- "mesafe (m)" --> PD_M
    LOG_M -- "ROI" --> COLOR_M
    CONFIG -. "parametreler" .-> DONANIM
    CONFIG -. "parametreler" .-> ALGILAMA
    CONFIG -. "parametreler" .-> KONTROL
```
