# Hava Savunma Sistemi — Yazılım Çalışma Açıklaması

## 1. Sistemin Genel Yapısı ve Donanım İlişkisi

Bu sistem, havadan gelen hedefleri (füze, helikopter, F16 savaş uçağı ve mini/mikro İHA) otomatik olarak tespit eden, takip eden ve lazerle imha eden bir hava savunma platformudur. Sistem beş ana donanım bileşeninden oluşur ve bunların tamamı bir Python yazılımı tarafından koordine edilir.

Birinci donanım bileşeni, sabit bir tripod üzerine monte edilmiş ZED 2 stereo kameradır. Bu kamera asla hareket etmez ve yaklaşık yüz on derecelik geniş bir yatay görüş alanına sahiptir. ZED 2'nin en önemli özelliği, sahip olduğu iki lens sayesinde stereo görüntüleme yaparak her pikselin kameradan kaç metre uzakta olduğunu hesaplayabilmesidir. Bu derinlik bilgisi, hedefin ne kadar yaklaştığını anlamak ve diğer kameraya doğru yönlendirme bilgisi üretmek için kullanılır.

İkinci donanım bileşeni, bir pan-tilt mekanizması üzerine monte edilmiş Logitech C920 kameradır. Bu kamera, ZED 2'nin yaklaşık yirmi yedi santimetre altında ve beş santimetre yanında konumlanmıştır. Pan-tilt mekanizması sayesinde yatay ve dikey eksenlerde hareket edebilir ve hedefe doğru yönelebilir. Görüş alanı ZED 2'ye göre daha dardır, yaklaşık yetmiş sekiz derecedir. Logitech kameranın asıl görevi, ZED 2 tarafından belirlenen hedefi yakın çekimle takip etmek ve lazerin tam hedefe isabet etmesini sağlamaktır.

Üçüncü bileşen, yine pan-tilt mekanizması üzerinde, Logitech kameranın yanına monte edilmiş Benewake TF03 tek boyutlu LiDAR sensörüdür. Bu sensör, lazer ışını göndererek hedefin tam olarak kaç metre uzakta olduğunu santimetre hassasiyetinde ölçer. LiDAR, seri port üzerinden saniyede birçok kez dokuz baytlık ikili paketler gönderir. Her paket iki adet senkronizasyon baytı, iki baytlık mesafe verisi, iki baytlık sinyal gücü, iki baytlık sıcaklık ve bir baytlık sağlama toplamı içerir. Yazılım bu paketleri sürekli arka planda okur, sağlama toplamını doğrular ve en son geçerli mesafe değerini metre cinsinden tutar.

Dördüncü bileşen, pan-tilt mekanizmasını kontrol eden Arduino Mega mikrodenetleyicisidir. Arduino, Python yazılımından UART seri port üzerinden komutlar alır ve step motorları hareket ettirir. İki çalışma modu vardır: manuel modda joystick ile kontrol edilir, otonom modda ise Python'dan gelen piksel hatası değerlerine göre kendi içinde PD (Proportional-Derivative) kontrol algoritması çalıştırarak motorları hedefe yönlendirir.

Beşinci ve son bileşen, Arduino'nun dokuzuncu pinine bağlı lazer modülüdür. Ateş komutu geldiğinde Arduino bu pini belirli bir süre boyunca aktif eder ve lazer ışını hedeefteki balonu patlatmak üzere gönderilir.

Tüm bu donanımlar bir Python uygulaması tarafından yönetilir. Kullanıcı arayüzü ise ayrı bir C# WinForms masaüstü uygulamasıdır. Python ile C# arasındaki haberleşme, Windows'un Memory-Mapped File (paylaşımlı bellek) mekanizması üzerinden gerçekleşir. Python işlenmiş video karelerini paylaşımlı belleğe yazar, C# arayüzü buradan okuyarak ekranda gösterir. Arayüzdeki buton tıklamaları da yine paylaşımlı bellek üzerinden Python'a iletilir.

---

## 2. Yapay Zeka ve Algılama Katmanı

Sistemin çekirdeğinde YOLOv8 nesne tespit modeli yer alır. Model, PyTorch formatından NVIDIA TensorRT formatına dönüştürülmüş ve GPU üzerinde optimize edilmiş bir "engine" dosyası olarak çalışır. Kullanılan engine dosyası "balon_baska.engine" olarak adlandırılmıştır ve statik batch boyutu ikiye ayarlanmıştır. Bu, her çıkarım çağrısında iki görüntünün aynı anda GPU'ya gönderilip işlenmesi anlamına gelir. Her frame'de ZED 2'den ve Logitech'ten alınan ikişer görüntü birlikte YOLO'ya verilir ve her iki kamera için ayrı ayrı tespit sonuçları elde edilir.

YOLO modeli dört sınıfı tanımak üzere eğitilmiştir: füze (missile), helikopter (helicopter), F16 savaş uçağı (f16) ve insansız hava aracı (uav). Balon sınıfı hiçbir aşamada kullanılmaz. Her bir tespit sonucu, hedefin görüntüdeki konumunu belirten bir sınırlayıcı kutu (bounding box), tespit edilen sınıfın adını ve güven yüzdesini (confidence) içerir.

Tespit edilen nesnelerin ardışık kareler arasında birbirine bağlanması için ByteTrack çoklu nesne takip algoritması kullanılır. ByteTrack, Ultralytics kütüphanesinin yerleşik bir özelliğidir ve "model.track" fonksiyonu ile çağrılır. Bu algoritma, her hedefe kalıcı bir takip numarası (track_id) atar ve hedef kısa süreliğine kaybolsa bile numarasını korur. Bu sayede sistem hangi hedefin daha önce tespit edildiğini, hangisinin yeni geldiğini ve hangisinin kaybedildiğini bilir.

---

## 3. Matematiksel Dönüşümler

Sistemde iki kritik matematiksel dönüşüm kullanılır. Birincisi açısal hız dönüşümüdür. ZED 2 kamerasında ByteTrack ile takip edilen hedefin piksel cinsinden hareket hızı ölçülür. Ancak bu piksel hızı doğrudan Logitech kamerasına aktarılamaz çünkü iki kameranın görüş alanları farklıdır. Aynı açısal hız, geniş görüş alanına sahip ZED'de daha az piksel, dar görüş alanına sahip Logitech'te daha çok piksel olarak görünür. Bu nedenle önce ZED'deki piksel hızı fiziksel açısal hıza dönüştürülür. Yatay açısal hız, yatay piksel hızının kare genişliğine bölünüp ZED'in yatay görüş alanı açısıyla çarpılmasıyla elde edilir. Dikey eksende de aynı işlem dikey görüş alanı açısıyla yapılır. Bu açısal hız değeri derece/saniye cinsindendir ve kameradan bağımsız fiziksel bir büyüklüktür. Logitech tarafında bu açısal hız, hedefe ileriye yönelik tahmin (feedforward) olarak eklenir.

İkinci dönüşüm paralaks telafisidir. Logitech kameranın lens merkezi ile lazer modülü arasında yaklaşık iki yüz milimetrelik yatay bir fiziksel mesafe vardır. Bu, kameranın tam ortasına hizalanmış bir hedefin aslında lazer açısından biraz sola kayık olduğu anlamına gelir. Kayma miktarı hedefin uzaklığına bağlıdır: yakın hedeflerde kayma büyük, uzak hedeflerde küçülür. Telafi formülü, fiziksel ofseti hedefin derinliğine bölüp kameranın odak uzunluğuyla çarparak piksel cinsinden kayma miktarını hesaplar. Bu kayma, nişan noktasından çıkarılarak lazer için doğru hedef konumu elde edilir.

---

## 4. Stage 1 — Manuel Mod (Statik Hedefler)

Stage 1'de sistem tamamen manuel olarak çalışır. Operatör arayüzdeki "Manuel" butonuna basar ve arayüz bu bilgiyi paylaşımlı bellek üzerinden Python'a iletir.

Python tarafında Stage 1 motoru başlatılır. Hem ZED 2 hem de Logitech C920 kamerası açılır. Her frame'de iki kameradan da görüntü alınır ve batch boyutu iki olan YOLO engine'ine birlikte gönderilir. ZED 2'den gelen görüntüdeki tespitler derinlik bilgisiyle zenginleştirilir. LiDAR sensörü de sürekli aktiftir ve her frame'de son mesafe değeri okunur.

Hedefleme için kırmızı piksel tespit yöntemi kullanılır. YOLO bir hedef tespit ettiğinde, tespit edilen sınırlayıcı kutunun (bbox) içindeki bölge kesilir. Bu bölge HSV veya LAB renk uzayına dönüştürülür ve kırmızı renk aralığına uyan pikseller maskelenir. Maskelenen kırmızı piksellerin ağırlık merkezi hesaplanır ve bu nokta nişan noktası olarak belirlenir. Bu şekilde sistemin tam hedef merkezini değil, hedef üzerindeki kırmızı bölgeyi (balonu veya işaret noktasını) hedeflemesi sağlanır.

Ancak Stage 1'de sistem hiçbir otomatik nişan alma veya ateş etme eylemi gerçekleştirmez. Arduino manuel modda çalışır ve operatör joystick ile pan-tilt mekanizmasını elle kontrol eder. Ekranda bounding box, sınıf adı, güven yüzdesi, ZED derinliği, LiDAR mesafesi, kırmızı hedef noktası ve FPS bilgileri overlay olarak çizilir. Operatör hedefi hizaladığına karar verdiğinde arayüzdeki "Ateş" butonuna basar, bu bilgi paylaşımlı bellek üzerinden Python'a ulaşır ve Python Arduino'ya ateş komutunu gönderir.

Aktif YOLO sınıfları bu aşamada füze, helikopter, F16 ve İHA olmak üzere dörttür.

---

## 5. Stage 2 — Otonom Swarm Takibi

Stage 2 tamamen otonom çalışır. Operatör arayüzden "Otonom" butonuna basar. Bu aşamada üç koldan birer hedef olmak üzere toplamda üç hedef gelir. Hedefler mesafelerine bakılmaksızın imha edilir, yani mesafe tabanlı bir ateş etme koşulu yoktur.

Aktif YOLO sınıfları yalnızca füze ve İHA olmak üzere ikidir. Arduino otonom moda alınır. ZED 2, Logitech ve LiDAR aynı anda çalışır.

Stage 2'nin çalışma mantığı iç içe geçmiş iki döngüden oluşur: dış spotter döngüsü ve iç PID takip döngüsü.

**Dış döngü (Spotter — ZED tarama):** Sistem sürekli olarak ZED 2 kamerasından ve Logitech'ten frame alır, ikisini birlikte YOLO'ya gönderir. ZED üzerindeki tespitlere ByteTrack uygulanarak her hedefe kalıcı takip numarası atanır. Her takip numarası için bir derinlik geçmişi tutulur. Bu geçmiş, zaman damgası ve derinlik değeri çiftlerinden oluşan bir sıralı listedir. İki ardışık ölçüm arasındaki derinlik farkı zamana bölünerek yaklaşma hızı hesaplanır. Eğer hedef uzaklaşıyorsa, yani yaklaşma hızı negatifse, o hedef filtrelenir ve dikkate alınmaz.

Kalan hedefler arasından en kritik olan seçilir. Önceliklendirme üç kritere göre yapılır: birincisi sınıf önceliği (füze, İHA'dan daha önceliklidir), ikincisi kritik mesafeye tahmini varış süresi, üçüncüsü frame merkezine yakınlıktır. Bu üç kriter değerlendirilerek birincil hedef (primary target) belirlenir.

Ayrıca hedefin piksel hızı ZED üzerinde ölçülür ve yukarda anlatılan formülle açısal hıza dönüştürülür. Bu açısal hız, ileride Logitech tarafında feedforward olarak kullanılacaktır.

**Handover (Spotter'dan Tracker'a geçiş):** Birincil hedef belirlendiğinde, ZED görüntüsündeki hedefin piksel koordinatlarından pan ve tilt açıları hesaplanır. Yatay açı, hedefin yatay piksel koordinatından frame genişliğinin yarısı çıkarılıp frame genişliğine bölünmesiyle ve sonucun ZED'in yatay görüş alanıyla çarpılmasıyla elde edilir. Dikey açı da aynı şekilde hesaplanır.

Bu açı değerleri Arduino'ya gönderilir ve pan-tilt mekanizması hesaplanan konuma döndürülür. Ardından yaklaşık yüz milisaniye beklenir. Bu bekleme süresi, motorların hareketi tamamlamasını ve mekanik titreşimlerin sönümlenmesini sağlar. Bekleme sonrasında Logitech kameranın görüntü ara belleği temizlenir. Bu önemlidir çünkü ara bellekteki eski kareler motor hareket etmeden önceki konumdan çekilmiştir ve artık geçersizdir. Ara bellek temizlendikten sonra yeni, taze bir kare alınır.

Bu taze karede Logitech üzerinden YOLO çalıştırılır ve hedef aranır. Arama, sınıf eşleşmesi, bounding box boyutu benzerliği ve frame merkezine yakınlık kriterlerine göre yapılır. Eğer hedef otuz frame içinde bulunamazsa handover başarısız sayılır ve sistem spotter döngüsüne geri döner. Engine değiştirme işlemi gerekmez çünkü batch boyutu iki olan engine her zaman çalışır durumdadır.

**İç döngü (PID Takip — Tight Loop):** Handover başarılı olduktan sonra sistem dar bir PID takip döngüsüne girer. Bu döngü dış spotter döngüsünden bağımsızdır ve çok daha hızlı çalışır.

Her PID frame'inde şu adımlar gerçekleşir: Önce ZED ve Logitech'ten eş zamanlı frame alınır ve birlikte YOLO'ya gönderilir. ZED sonuçlarından hedefin güncel derinliği ve açısal hızı güncellenir. Logitech sonuçlarından hedefin frame içindeki konumu belirlenir. Hedefin piksel koordinatlarından frame merkezine olan uzaklık normalize edilerek piksel hatası hesaplanır. Bu hata değerine paralaks telafi uygulanır. Ardından ZED'den gelen açısal hız feedforward olarak eklenir. Son hata değeri Arduino'ya UART üzerinden gönderilir ve Arduino kendi PD algoritmasıyla motorları hareket ettirir.

Eğer Logitech görüntüsünde hedef aniden kaybolursa, sistem panik yapıp hemen spotter'a dönmez. Bunun yerine son gönderilen Arduino komutunu dondurur (motorlar son pozisyonda kalır) ve on beş frame boyunca hedefe yeniden arar. Eğer on beş frame içinde hedef tekrar bulunursa PID döngüsüne kaldığı yerden devam eder. Bulunamazsa sistem spotter dış döngüsüne geri döner.

Hedef kilitlenme (LOCKED) durumu, piksel hatasının belirli bir eşik değerinin altında art arda N frame boyunca kalmasıyla tespit edilir. Stage 2'de hedef kilitlendiğinde mesafe kontrolü yapılmaz ve sistem hemen ateş komutunu verir. LiDAR ve ZED derinlik verileri yalnızca ekrandaki overlay ve telemetri bilgilerinde gösterilmeye devam eder.

**Post-Engagement (Atış sonrası):** Ateş komutu verildikten sonra yaklaşık yarım saniyelik kısa bir bekleme yapılır. Bu bekleme, lazerin hedefte etkisini göstermesini sağlar. Ardından sistem otomatik olarak spotter döngüsüne geri döner ve bir sonraki en kritik hedefi arar. Eğer tüm hedefler yok edildiyse veya uzaklaşıyorsa, sistem boşta tarama moduna geçer ve ZED ile çevreyi taramaya devam eder.

---

## 6. Stage 3 — Otonom + Dost/Düşman Ayrımı

Stage 3, Stage 2'nin bir üst kümesidir. Spotter döngüsü, handover protokolü, PID iç döngüsü ve post-engagement mantığı Stage 2 ile birebir aynıdır. Stage 3'ün iki temel farkı vardır: aktif YOLO sınıfları dörde çıkar (füze, helikopter, F16, İHA) ve handover ile PID arasına bir renk doğrulama fazı eklenir.

**Balon rengi maskeleme:** Her hedefin üzerinde bir balon bulunur. Balonun konumu sınıfa göre değişir. F16 ve helikopterde balon yolcu bölümünde (kokpit alanında) yer alır, İHA'da gövdenin altında asılıdır, füzede ise burnunun ucundadır. Dost/düşman renk tespiti yapılırken balonun pikselleri dışlanmalıdır çünkü tüm hedeflerin balonları aynı renkte olabilir. Bunun için bounding box'ın sınıfa göre belirlenen alt bölgesi maskelenir ve renk sınıflandırıcı yalnızca maket gövdesi pikselleri üzerinde çalıştırılır.

**Dost Blacklist mekanizması:** Spotter döngüsünde hedef seçerken, daha önce "dost" olarak işaretlenmiş takip numaraları bir kara listeye eklenir. Spotter, bu kara listedeki hedefleri atlar ve sonraki düşman hedefine geçer. Bu sayede aynı dost hedef tekrar tekrar seçilmez.

**Renk doğrulama fazı:** Handover başarıyla tamamlandıktan sonra, PID döngüsüne girmeden önce bir renk doğrulama fazı devreye girer. Bu fazda Logitech'ten alınan her frame'de hedefin bounding box'ı kırpılır ve LAB renk uzayında sınıflandırılır. LAB renk uzayı, insan algısına daha yakın bir renk temsili sunar ve aydınlatma değişimlerine karşı BGR'ye göre çok daha dayanıklıdır. A kanalı kırmızı-yeşil eksenini, B kanalı sarı-mavi eksenini temsil eder.

Eğer sınıflandırıcı mavi renk tespit ederse, hedef dost olarak kabul edilir. Bu hedefin takip numarası kara listeye eklenir ve sistem hemen spotter'a geri döner. Eğer kırmızı renk tespit edilirse, hedef düşman olarak kabul edilir ancak tek bir frame yeterli değildir. Beş frame'den en az üçünde kırmızı tespit edilmelidir. Bu üçte beşlik onay sağlandığında sistem PID iç döngüsüne geçer. Eğer renk belirsizse (ne net kırmızı ne net mavi), sistem tekrar dener. Ancak bu döngü sonsuza kadar süremez. Otuz frame içinde net bir sonuç elde edilemezse, hedef atlanır ve sistem spotter'a geri döner.

**Range Gate (Menzil kontrolü):** Stage 2'den farklı olarak, Stage 3'te hedef kilitlendiğinde mesafe kontrolü uygulanır. Her sınıf için farklı minimum ve maksimum menzil değerleri tanımlanmıştır. F16 için on ile on beş metre, füze için on ile on beş metre, mini/mikro İHA için sıfır ile on beş metre, helikopter için beş ile on beş metredir. Hedef kilitlendiğinde LiDAR mesafesi bu tabloya göre değerlendirilir. Eğer hedef çok uzaksa, yani henüz menzile girmemişse, sistem PID döngüsünde kalmaya devam eder ve hedefin yaklaşmasını bekler. Eğer hedef çok yakınsa, yani menzili geçip tehlikeli derecede yaklaşmışsa, sistem o hedefi iptal eder ve spotter'a döner. Eğer hedef menzil aralığındaysa, ateş komutu verilir.

**LiDAR Fallback:** LiDAR sensörü zaman zaman hata verebilir veya geçici olarak veri gönderemeyebilir. Eğer LiDAR beş yüz milisaniyeden uzun süre geçerli bir mesafe döndürmezse, sistem LiDAR yerine en son güvenilir ZED stereo derinlik değerini kullanır. Ancak stereo derinlik tek boyutlu LiDAR kadar hassas olmadığı için bir metrelik güvenlik payı eklenir. Eğer ZED derinliği artı/eksi bir metre toleransla menzil aralığında kalıyorsa ateş komutu yine verilir. Aksi halde PID döngüsüne devam edilir.

---

## 7. Haberleşme Mimarisi

Python arka planı ile C# arayüzü arasındaki tüm haberleşme Windows Memory-Mapped File mekanizması üzerinden gerçekleşir.

C# arayüzünden Python'a doğru beş farklı paylaşımlı bellek kanalı kullanılır. Birincisi "SystemMode" adlı on baytlık bir alan olup stage seçimini iletir. İçine "1" yazıldığında otonom mod, "2" yazıldığında manuel mod, "3" yazıldığında yarı otonom mod aktifleşir. İkincisi "FireDataMemory" adlı tek baytlık bir boolean alandır ve ateş komutu anlamına gelir. Üçüncüsü "FireCommandMemory" adlı yine tek baytlık bir alan olup ateşe devam komutu iletir. Dördüncüsü "EngagementDataMemory" adlı tek baytlık alan olup angajman kabul bilgisini taşır. Beşincisi "AngleDataMemory" adlı iki bin kırk sekiz baytlık bir alan olup yasaklı bölge açı aralığını "minimum,maximum" formatında string olarak içerir.

Python'dan C# arayüzüne doğru ise iki paylaşımlı bellek kanalı kullanılır. Birincisi "raw_frame" olup ham kamera görüntüsünü, ikincisi "processed_frame" olup YOLO tespitleri ve overlay bilgileri çizilmiş işlenmiş görüntüyü taşır. C# arayüzü bu iki bellek alanını yaklaşık on beş milisaniye aralıklarla okur ve ilgili PictureBox bileşenlerinde gösterir.

Python ile Arduino arasındaki haberleşme ise UART seri port üzerinden gerçekleşir. COM4 portu üzerinden yüz on beş bin iki yüz baud hızında iletişim kurulur. Konum modu paketleri "P{pan açısı çarpı on},T{tilt açısı çarpı on},F{ateş durumu}" formatında, hata modu paketleri ise "E{yatay hata çarpı bin},Y{dikey hata çarpı bin}" formatında gönderilir.

LiDAR ile haberleşme COM3 portu üzerinden aynı baud hızında, dokuz baytlık ikili paketlerle gerçekleşir. Bir arka plan iş parçacığı sürekli bu portu dinler, gelen paketleri ayrıştırır ve en son geçerli mesafe değerini bir değişkende tutar. Ana döngü bu değişkeni her ihtiyaç duyduğunda okur.

---

## 8. Hata Yönetimi ve Kurtarma

Hata yönetimi sistemin son aşamasında, tüm stage'ler çalışır duruma geldikten sonra uygulanacaktır.

Kamera açma veya frame okuma hatalarında sistem otomatik olarak yeniden bağlanma dener. Arduino seri port bağlantısı koparsa, bir yeniden bağlanma döngüsü başlar ve aynı zamanda arayüzde uyarı gösterilir. LiDAR bağlantısı koparsa, sistem fallback moduna geçer ve ZED derinlik verisini kullanmaya devam eder. YOLO model yükleme hatası oluşursa anlamlı bir hata mesajı verilir ve sistem düzgün şekilde durur. Tek bir frame'in işlenmesi sırasında oluşan hatalar sessizce atlanır ve döngü bir sonraki frame ile devam eder.

Sistemin temel prensibi hiçbir koşulda çökmemektir. Kritik donanım hatalarında arayüzde büyük kırmızı uyarı gösterilir ve olay kaydedilir. Geçici hatalarda ise sistema devam eder, kullanıcı farkına bile varmaz.
