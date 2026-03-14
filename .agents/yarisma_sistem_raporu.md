# Yazılım Tasarımı

Sistemimizin yazılım mimarisi, yüksek hızlı veri işleme ve düşük gecikmeli donanım kontrolü prensiplerine göre tasarlanmıştır. Bu kapsamda geliştirdiğimiz yazılım bileşenleri, birbirleriyle kesintisiz ve akıcı bir etkileşim halindedir.

## 1. Yazılımların Birbirleriyle Arayüzleri

Sistemdeki veri akışı, kameradan alınan ham görüntünün mikrodenetleyiciye iletilen fiziksel bir harekete dönüşmesine kadar birbirine sıkı sıkıya bağlı bir süreçtir. 

Öncelikle sahadaki optik veriler **OpenCV** kütüphanesi aracılığıyla dijital görüntü matrislerine dönüştürülür. Elde edilen bu anlık görüntü akışı, beklemeden hedef tespiti yapmak üzere **YOLOv8** modeline aktarılır. YOLOv8 sahnede yer alan hava araçlarını tespit ettiği anda, bu nesnelerin koordinat ve güvenilirlik verileri **ByteTrack** algoritmasına iletilir. ByteTrack, anlık konum değişimlerini hesaplayarak hedefin ardışık karelerde kaybolmadan takip edilmesini sağlar ve hedefin merkeze olan piksel sapmasını sayısal olarak netleştirir.

Görüntü işleme ve analiz tarafında bu işlemler gerçekleştirilirken, sonuçların anlık takibi ve operatör müdahalesi için veriler sistem hafızası üzerinden **.NET WinForms** masaüstü arayüzüne gönderilir. Böylece kullanıcı, sistemin durumunu canlı görüntü üzerinden takip edip yönlendirebilir.

Son aşamada, tespit edilen piksel sapmaları **NumPy** kütüphanesinin matematiksel altyapısıyla hata oranlarına dönüştürülür. Hesaplanan bu konum düzeltme değerleri, seri haberleşme protokolü olan **PySerial** üzerinden fiziksel donanıma (mikrodenetleyiciye) iletilir. Mikrodenetleyici tarafında ise **AccelStepper** kütüphanesi devreye girerek bu komutları alır; pan-tilt motorlarına sarsıntısız, ivmelenmeli ve hassas bir hareket profili uygulayarak namluyu hedefle hizalar. 

## 2. Temel Gereksinimler

Sistemin hedeflenen performansla ve kararlı bir şekilde çalışabilmesi için yazılım bileşenlerinin karşılaması gereken temel gereksinimler şunlardır:

*   **Gerçek Zamanlı İşleme:** YOLOv8 ve OpenCV işlemlerinin, hedefin yüksek hızından dolayı minimum 30 FPS (kare/saniye) hızında çalışması ve görüntü analizinin kare düşmesi yaşanmadan tamamlanması.
*   **Düşük Gecikme (Low Latency):** Görüntünün kameradan alınması ile motorların harekete geçmesi arasındaki toplam sürenin, hedefin görüş alanından çıkmasını engelleyecek kadar kısa (milisaniye mertebesinde) olması.
*   **Kesintisiz Takip:** ByteTrack algoritmasının, kamera hareketinden veya anlık görüntü bozulmalarından etkilenmeyerek hedefin kimliğini ve takibini sürekli kılması.
*   **Sarsıntısız Motor Kontrolü:** AccelStepper algoritması sayesinde hedefe yönelme sırasında sistemin kendi titreşimini engellemek için motor başlangıç ve duruşlarının ani sıçramalar olmadan, yumuşak ivmelenme (trapezoidal hareket) ile gerçekleşmesi.
*   **Asenkron Haberleşme:** PySerial UART haberleşmesinin ve işlemlerin birbirini bekleyip kitlememesi adına, kamera okuma, görüntü işleme ve motor komut gönderim süreçlerinin birbirinden bağımsız (asenkron veya eş zamanlı iş parçacıklı) yürütülmesi.

## 3. Arayüz Şeması (Blok Diyagramı)

Rapora eklenecek mimari çizim için aşağıdaki veri akış şeması referans alınabilir:

*   **[Kamera Donanımı]** → (Ham Video Akışı) → **[OpenCV Modülü]**
*   **[OpenCV Modülü]** → (Görüntü Matrisi) → **[YOLOv8 ve ByteTrack Modülü]**
*   **[YOLOv8 ve ByteTrack Modülü]** → (Sınırlayıcı Kutu ve Takip Verisi) → **[NumPy Matematiksel İşlemci]**
*   **[YOLOv8 ve ByteTrack Modülü]** → (İşlenmiş Görüntü + Durum Verisi) → **[.NET WinForms Kullanıcı Arayüzü]**
*   **[.NET WinForms Kullanıcı Arayüzü]** → (Operatör Komutları) → **[Ana Python Döngüsü]**
*   **[NumPy Matematiksel İşlemci]** → (Hesaplanmış X-Y Piksel Sapması) → **[PySerial Haberleşme Arayüzü]**
*   **[PySerial Haberleşme Arayüzü]** → (UART Motor Komutları, Örn: Hedef Açı) → **[Mikrodenetleyici]**
*   **[Mikrodenetleyici]** → (İvmeli Adım Sinyalleri) → **[AccelStepper Motor Sürücüsü]**
*   **[AccelStepper Motor Sürücüsü]** → (Yumuşak Fiziksel Hareket) → **[Pan-Tilt Motorları]**
