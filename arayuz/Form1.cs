using OpenCvSharp;
using System.Drawing.Imaging;
using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;
using System.Text;
using System.Drawing;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Window;
using System.Windows.Forms;



namespace tasarımdeneme3
{
    public partial class Form1 : Form
    {
        private VideoCapture _capture;
        private Thread? _cameraThread;
        private int frameCount = 0; // Gelen frame sayısı
        private const int FrameWidth = 640;
        private const int FrameHeight = 640;
        private Thread _sharedMemoryThread;
        private System.Diagnostics.Process pythonProcess; // Python sürecini izlemek için
                                                          // Başlangıç değerlerini saklamak için değişkenler
        private Dictionary<Control, Rectangle> originalControlBounds = new Dictionary<Control, Rectangle>();
        private System.Drawing.Size originalFormSize;
        private bool isMaximized = false; // Pencerenin durumunu kontrol etmek için
        private bool isAngleSent = false; // Açı gönderildi mi?
        // Win32 API Tanımları
        [DllImport("user32.dll")]
        public static extern int SendMessage(IntPtr hWnd, int Msg, int wParam, int lParam);

        [DllImport("user32.dll")]
        public static extern bool ReleaseCapture();

        // Sürükleme için gereken mesaj sabitleri
        public const int WM_NCLBUTTONDOWN = 0xA1;
        public const int HT_CAPTION = 0x2;


        public Form1()
        {
            InitializeComponent();
            var handle = this.Handle;
            // MouseDown olayını bağlayın
        }
        private void Form1_Load(object sender, EventArgs e)
        {


            //StartSharedMemoryReader();


            // Timer'ı oluştur ve ayarla

            clockTimer.Tick += ClockTimer_Tick; // Timer her tetiklendiğinde çalışacak metod
            clockTimer.Start(); // Timer'ı başlat
            fpsTimer.Tick += FpsTimer_Tick;
            fpsTimer.Start();

            // Formun başlangıç boyutunu kaydet
            originalFormSize = this.Size;

            // Tüm kontrol ve başlangıç boyutlarını/kordinatlarını kaydet
            foreach (Control control in this.Controls)
            {
                originalControlBounds[control] = control.Bounds;
            }

            btnYarıOto.BackgroundImageLayout = ImageLayout.Stretch;
            btnOtonom.BackgroundImageLayout = ImageLayout.Stretch;
            btnManual.BackgroundImageLayout = ImageLayout.Stretch;
            btnStart.BackgroundImageLayout = ImageLayout.Stretch;
            btnExit.BackgroundImageLayout = ImageLayout.Stretch;
            btnMinimize.BackgroundImageLayout = ImageLayout.Stretch;
            btnSize.BackgroundImageLayout = ImageLayout.Stretch;
        }


        private void vScrollBar1_Scroll(object sender, ScrollEventArgs e)
        {
            // Geçerli satır indeksini hesapla
            int lineIndex = e.NewValue;

            // RichTextBox'ta geçerli satır sayısını al
            int totalLines = textLog.Lines.Length;

            // Geçersiz bir satır indisine erişilmediğinden emin ol
            if (lineIndex >= 0 && lineIndex < totalLines)
            {
                // Satırın başlangıcındaki karakter indeksini alın
                int charIndex = textLog.GetFirstCharIndexFromLine(lineIndex);
                if (charIndex != -1)
                {
                    textLog.SelectionStart = charIndex; // İmleci ayarla
                    textLog.ScrollToCaret(); // Kaydırmayı gerçekleştir
                }
            }
        }


        private void PictureBox1_MouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                // Pencereyi taşımak için API çağrılarını kullan
                ReleaseCapture();
                SendMessage(this.Handle, WM_NCLBUTTONDOWN, HT_CAPTION, 0);
            }
        }





        private void EnterFullscreen()
        {
            // Tam ekran moduna geçiş için pencere özelliklerini ayarla
            this.FormBorderStyle = FormBorderStyle.None; // Kenarlık kaldır
            this.WindowState = FormWindowState.Normal;   // Normal durum
            this.Bounds = Screen.PrimaryScreen.Bounds;  // Tam ekran boyutuna ayarla
            this.TopMost = true;                         // Uygulama diğer pencerelerin önünde olsun

            ResizeComponents(); // Bileşenleri yeniden boyutlandır
        }

        private void RestoreWindow()
        {
            // Varsayılan pencere moduna geçiş
            this.TopMost = false;                         // Diğer pencerelerin önceliğini kaldır
            this.FormBorderStyle = FormBorderStyle.None; // Kenarlık geri getir
            this.WindowState = FormWindowState.Normal;    // Normal pencere durumuna getir
            this.Size = originalFormSize;                 // Eski boyutuna döndür

            ResizeComponents(true); // Orijinal boyutları geri yükle
        }

        private void ResizeComponents(bool restoreOriginal = false)
        {
            float xRatio = (float)this.Width / originalFormSize.Width;   // Genişlik oranı
            float yRatio = (float)this.Height / originalFormSize.Height; // Yükseklik oranı

            foreach (Control control in this.Controls)
            {
                // Kontrolün orijinal boyutlarını al
                Rectangle originalBounds = originalControlBounds[control];

                if (restoreOriginal)
                {
                    // Orijinal boyutlara geri dön
                    control.SetBounds(originalBounds.X, originalBounds.Y, originalBounds.Width, originalBounds.Height);
                }
                else
                {
                    // Yeni boyut ve konumu hesapla
                    int newX = (int)(originalBounds.X * xRatio);
                    int newY = (int)(originalBounds.Y * yRatio);
                    int newWidth = (int)(originalBounds.Width * xRatio);
                    int newHeight = (int)(originalBounds.Height * yRatio);

                    // Kontrolün boyut ve konumunu güncelle
                    control.SetBounds(newX, newY, newWidth, newHeight);
                }
            }
        }


        //CLOCK TICK EVENTLERI-----------------------
        private void ClockTimer_Tick(object? sender, EventArgs e)
        {
            // Label'a güncel saati yaz
            lblSaat.Text = DateTime.Now.ToString("HH:mm:ss"); // Saat, dakika ve saniye formatında
        }
        private void FpsTimer_Tick(object? sender, EventArgs e)
        {
            // FPS hesapla ve Label'a yaz
            lblFps.Text = $"{frameCount}";
            frameCount = 0; // Sayacı sıfırla
        }

        //MEMORY OKUMA START SHARED MEMORY READ MEMORY------------------------------
        private void StartSharedMemoryReader()
        {
            try
            {
                _sharedMemoryThread = new Thread(ReadSharedMemory);
                _sharedMemoryThread.IsBackground = true;
                _sharedMemoryThread.Start();

                AppendTextWithColor(textLog, "Shared memory thread başlatıldı.\n", Color.White);
            }
            catch (Exception ex)
            {
                textLog.Text = ($"Thread başlatma hatası: {ex.Message}");
                AppendTextWithColor(textLog, $"Thread başlatma hatası: {ex.Message}", Color.Red);
            }
        }



        private void ReadSharedMemory()
        {
            try
            {
                using (var rawMmf = MemoryMappedFile.OpenExisting("raw_frame"))
                using (var processedMmf = MemoryMappedFile.OpenExisting("processed_frame"))
                //using (var counterMmf = MemoryMappedFile.OpenExisting("lock_counter"))
                //using (var arduinoDataMmf = MemoryMappedFile.OpenExisting("ArduinoData")) // ArduinoData için shared memory
                {
                    while (true)
                    {
                        // İşlenmemiş görüntüyü oku
                        using (var rawAccessor = rawMmf.CreateViewAccessor())
                        {
                            byte[] rawBuffer = new byte[FrameWidth * FrameHeight * 3];
                            rawAccessor.ReadArray(0, rawBuffer, 0, rawBuffer.Length);

                            var rawBmp = new Bitmap(FrameWidth, FrameHeight, PixelFormat.Format24bppRgb);
                            var rawRect = new Rectangle(0, 0, rawBmp.Width, rawBmp.Height);
                            var rawData = rawBmp.LockBits(rawRect, ImageLockMode.WriteOnly, rawBmp.PixelFormat);
                            System.Runtime.InteropServices.Marshal.Copy(rawBuffer, 0, rawData.Scan0, rawBuffer.Length);
                            rawBmp.UnlockBits(rawData);

                            //rawBmp.RotateFlip(RotateFlipType.RotateNoneFlipX);

                            if (this.IsHandleCreated)
                            {
                                frameCount++;
                                Invoke(new Action(() =>
                                {
                                    pictureBox2.Image?.Dispose();
                                    pictureBox2.Image = rawBmp; // İşlenmemiş görüntü
                                }));
                            }
                        }

                        // İşlenmiş görüntüyü oku
                        using (var processedAccessor = processedMmf.CreateViewAccessor())
                        {
                            byte[] processedBuffer = new byte[FrameWidth * FrameHeight * 3];
                            processedAccessor.ReadArray(0, processedBuffer, 0, processedBuffer.Length);

                            var processedBmp = new Bitmap(FrameWidth, FrameHeight, PixelFormat.Format24bppRgb);
                            var processedRect = new Rectangle(0, 0, processedBmp.Width, processedBmp.Height);
                            var processedData = processedBmp.LockBits(processedRect, ImageLockMode.WriteOnly, processedBmp.PixelFormat);
                            System.Runtime.InteropServices.Marshal.Copy(processedBuffer, 0, processedData.Scan0, processedBuffer.Length);
                            processedBmp.UnlockBits(processedData);

                            // Görüntüyü yatay olarak ters çevir
                            // processedBmp.RotateFlip(RotateFlipType.RotateNoneFlipX);

                            if (this.IsHandleCreated)
                            {
                                Invoke(new Action(() =>
                                {
                                    pictureBox3.Image?.Dispose();
                                    pictureBox3.Image = processedBmp; // İşlenmiş görüntü
                                }));
                            }
                        }

                        //// Sayaç değerini oku
                        //using (var counterAccessor = counterMmf.CreateViewAccessor())
                        //{
                        //	int lockCount = counterAccessor.ReadInt32(0);

                        //	if (this.IsHandleCreated)
                        //	{
                        //		Invoke(new Action(() =>
                        //		{
                        //			lblLock.Text = $"Lock Count: {lockCount}"; // Sayaç değeri
                        //		}));
                        //	}
                        //}

                        //ArduinoData verisini oku
                        //using (var arduinoDataAccessor = arduinoDataMmf.CreateViewAccessor())
                        //{
                        //	byte[] buffer = new byte[10]; // Python tarafında belirlenen boyut
                        //	arduinoDataAccessor.ReadArray(0, buffer, 0, buffer.Length);
                        //	string arduinoData = Encoding.UTF8.GetString(buffer).Trim();

                        //	if (this.IsHandleCreated)
                        //	{
                        //		Invoke(new Action(() =>
                        //		{
                        //			textLog.Text += $"Arduino Data: {arduinoData}"; // Arduino'dan gelen veri

                        //		}));
                        //	}
                        //}

                        Thread.Sleep(15);
                    }
                }
            }
            catch (Exception ex)
            {
                AppendTextWithColor(textLog, $"Shared Memory Hatası: {ex.Message}", Color.Red);
            }
        }

        private void WriteToSharedMemory(string data)
        {
            try
            {
                using (var mmf = MemoryMappedFile.CreateOrOpen("SystemMode", 10)) // 10 karakterlik shared memory
                using (var accessor = mmf.CreateViewAccessor())
                {
                    byte[] buffer = Encoding.UTF8.GetBytes(data.PadRight(10)); // Veriyi shared memory boyutuna uygun hale getir
                    accessor.WriteArray(0, buffer, 0, buffer.Length); // Veriyi shared memory'ye yaz
                }


                AppendTextWithColor(textLog, $"Shared Memory'e yazıldı: {data}\n", Color.White);
            }
            catch (Exception ex)
            {

                AppendTextWithColor(textLog, $"Shared Memory yazma hatası: {ex.Message}\n", Color.Red);
            }
        }

        // Mesaj yazdırma fonksiyonu---------------
        private void AppendTextWithColor(RichTextBox richTextBox, string text, Color color)
        {
            if (richTextBox.InvokeRequired)
            {
                richTextBox.Invoke(new Action(() => AppendTextWithColor(richTextBox, text, color)));
            }
            else
            {
                int start = richTextBox.TextLength; // Mevcut metin uzunluğu
                richTextBox.AppendText(text); // Yeni metni ekle
                richTextBox.SelectionStart = start; // Yeni metnin başlangıç konumu
                richTextBox.SelectionLength = text.Length; // Yeni metnin uzunluğu
                richTextBox.SelectionColor = color; // Yeni metnin rengi
                richTextBox.SelectionStart = richTextBox.TextLength; // İmleci sona taşı
                richTextBox.SelectionColor = richTextBox.ForeColor; // Varsayılan rengi geri yükle
            }
        }
        //CLEAN UP SHARED MEMORY METOD BUTONLAR İÇİN---------------
        // Paylaşımlı belleği temizlemek için bir yöntem ekleyin
        private void CleanupSharedMemory(string name)
        {
            try
            {
                var shm = MemoryMappedFile.OpenExisting(name);
                shm.Dispose();
                textLog.AppendText($"Paylaşımlı bellek '{name}' temizlendi.\n");
            }
            catch (FileNotFoundException)
            {
                textLog.AppendText($"Paylaşımlı bellek '{name}' bulunamadı.\n");
            }
            catch (Exception ex)
            {
                textLog.AppendText($"Paylaşımlı bellek temizleme hatası: {ex.Message}\n");
            }
        }


        //YÜKLENİYOR YAZISI 


        private void ShowLoadingMessage()
        {
            pictureBox2.Invalidate(); // PictureBox1'i yeniden çiz
            pictureBox3.Invalidate(); // PictureBox2'yi yeniden çiz
            pictureBox2.Paint += PictureBox_Paint; // Paint olayına bağlan
            pictureBox3.Paint += PictureBox_Paint; // Paint olayına bağlan
        }

        private void HideLoadingMessage()
        {
            pictureBox2.Paint -= PictureBox_Paint; // Paint olayını kaldır
            pictureBox3.Paint -= PictureBox_Paint; // Paint olayını kaldır
            pictureBox2.Invalidate(); // PictureBox1'i yeniden çiz
            pictureBox3.Invalidate(); // PictureBox2'yi yeniden çiz
        }

        private void PictureBox_Paint(object sender, PaintEventArgs e)
        {
            // "Yükleniyor" yazısını çiz
            var g = e.Graphics;
            var rect = ((PictureBox)sender).ClientRectangle;
            using (var brush = new SolidBrush(Color.FromArgb(128, Color.Black))) // Şeffaf arka plan
            using (var font = new Font("Arial", 16, FontStyle.Bold))
            {
                g.FillRectangle(brush, rect); // Şeffaf arka plan
                var text = "Yükleniyor...";
                var textSize = g.MeasureString(text, font);
                var textPoint = new PointF(
                    (rect.Width - textSize.Width) / 2,
                    (rect.Height - textSize.Height) / 2
                );
                g.DrawString(text, font, Brushes.White, textPoint); // Beyaz renk yazı
            }
        }

        //PYTHON SCRTIP BASLATMA VE DELAYLER--------------------

        private async Task WaitForSharedMemory()
        {
            int maxAttempts = 10; // Maksimum 10 saniye bekle
            int attempts = 0;

            while (attempts < maxAttempts)
            {
                try
                {
                    // Ortak bellek var mı kontrol et
                    MemoryMappedFile.OpenExisting("raw_frame").Dispose();
                    MemoryMappedFile.OpenExisting("processed_frame").Dispose();
                    return; // Eğer bellek bulunursa çık
                }
                catch (FileNotFoundException)
                {
                    attempts++;
                    await Task.Delay(10000); // 1 saniye bekle
                }
            }

            throw new Exception("Ortak bellek bulunamadı. Python betiği henüz başlatılmamış olabilir.");
        }

        private void StartPythonScript()
        {
            try
            {
                // Python betiğini çalıştırmak için Process oluştur
                pythonProcess = new System.Diagnostics.Process();
                pythonProcess.StartInfo.FileName = "python";                // Python'un çalıştırılabilir dosyası
                pythonProcess.StartInfo.Arguments = @"C:\Users\demir\airdefence\HavaSavunmaAnaScript.py"; // Betiğin tam yolu
                pythonProcess.StartInfo.WorkingDirectory = @"C:\Users\demir\airdefence"; // Çalışma dizini
                pythonProcess.StartInfo.UseShellExecute = false; // Komut satırı penceresini gizle
                pythonProcess.StartInfo.CreateNoWindow = true;  // Yeni bir pencere açılmasın
                pythonProcess.StartInfo.RedirectStandardError = true; // Hataları yakala
                pythonProcess.StartInfo.RedirectStandardOutput = true; // Çıktıyı yakala

                // Python betiğini başlat
                pythonProcess.Start();

                // Hataları ve çıktı loglarını (isteğe bağlı)
                Task.Run(() =>
                {
                    string output = pythonProcess.StandardOutput.ReadToEnd();
                    string error = pythonProcess.StandardError.ReadToEnd();

                    if (!string.IsNullOrEmpty(output))
                        textLog.Invoke(new Action(() =>
                        {
                            textLog.AppendText($"Python Çıktısı: {output}\n");
                        }));

                    if (!string.IsNullOrEmpty(error))
                        textLog.Invoke(new Action(() =>
                        {
                            textLog.AppendText($"Python Hatası: {error}\n");
                        }));
                });

                AppendTextWithColor(textLog, "Python betiği arka planda başlatıldı.\n", Color.Green);
            }
            catch (Exception ex)
            {
                AppendTextWithColor(textLog, $"Python betiği çalıştırılırken hata oluştu: {ex.Message}", Color.Red);
            }
        }


        private void StopAllPythonProcesses()
        {
            try
            {
                // Tüm çalışan Python süreçlerini al
                var pythonProcesses = System.Diagnostics.Process.GetProcessesByName("python");

                if (pythonProcesses.Length > 0)
                {
                    foreach (var process in pythonProcesses)
                    {
                        try
                        {
                            if (!process.HasExited)
                            {
                                process.Kill(); // Süreci sonlandır
                                process.WaitForExit(); // Çıkmasını bekle
                                textLog.AppendText($"Python süreci {process.Id} başarıyla durduruldu.\n");
                            }
                        }
                        catch (Exception ex)
                        {
                            textLog.AppendText($"Süreç {process.Id} kapatılırken hata: {ex.Message}\n");
                        }
                        finally
                        {
                            process.Dispose(); // Kaynakları serbest bırak
                        }
                    }
                }
                else
                {
                    textLog.AppendText("Çalışan Python süreci bulunamadı.\n");
                }
            }
            catch (Exception ex)
            {
                textLog.AppendText($"Python süreçlerini kontrol ederken hata oluştu: {ex.Message}\n");
            }
        }



        //MOD BUTONLAR CLICKLERR-------------------
        // Durumları takip etmek için değişkenler

        private bool isOtonomClicked = false;
        private bool isManuelClicked = false;
        private bool isYarıOtonomClicked = false;
        private bool isStarted = false;
        //private bool isSendcordinatesClicked = false;
        private bool isAtısdevamClicked = false;
        private bool isAtısClicked = false;
        private bool isAngajmanKabuluClicked = false;
        private bool isSendcord = false;



        // btnStart Click Olayı
        private async void BtnStart_Click(object sender, EventArgs e)
        {
            if (!isStarted) // Başlat modundan Durdur moduna geçiş
            {
                lblDurum.Text = "AKTİF";
                // Arka plan resmini değiştir
                btnStart.BackgroundImage = Properties.Resources.durdur1; // Tıklanmış resim
                btnStart.BackgroundImageLayout = ImageLayout.Stretch;
                isStarted = true;

                // Tüm butonları sıfırla
                ResetAllButtons();

                // "Yükleniyor" mesajını göster
                ShowLoadingMessage();

                // Python betiğini arka planda çalıştır
                await Task.Run(() => StartPythonScript());

                // Ortak bellek hazır olana kadar bekle
                try
                {
                    await WaitForSharedMemory();
                }
                catch (Exception ex)
                {
                    // Hata durumunda "Yükleniyor" mesajını kaldır ve mesaj göster
                    HideLoadingMessage();
                    MessageBox.Show(ex.Message);
                    btnStart.BackgroundImage = Properties.Resources.baslat1; // Varsayılan resim
                    btnStart.BackgroundImageLayout = ImageLayout.Stretch;
                    isStarted = false;
                    return;
                }

                // "Yükleniyor" mesajını kaldır
                HideLoadingMessage();

                // Ortak bellekten veri okumayı başlat
                StartSharedMemoryReader();
            }
            else // Durdur modundan Başlat moduna geçiş
            {
                lblDurum.Text = "PASİF";
                // Arka plan resmini değiştir
                btnStart.BackgroundImage = Properties.Resources.baslat1; // Varsayılan resim
                btnStart.BackgroundImageLayout = ImageLayout.Stretch;
                isStarted = false;

                // Tüm butonları sıfırla
                ResetAllButtons();

                StopAllPythonProcesses();
            }
        }

        private void BtnOtonom_Click(object sender, EventArgs e)
        {
            ResetAllButtons();
            // Resim değiştirme işlemi
            if (isOtonomClicked)
            {
                btnOtonom.BackgroundImage = Properties.Resources.otonom1; // Varsayılan resim
                isOtonomClicked = false;
            }
            else
            {
                btnOtonom.BackgroundImage = Properties.Resources.otonom3; // Tıklanmış resim
                isOtonomClicked = true;
            }
            btnOtonom.BackgroundImageLayout = ImageLayout.Stretch;

            // Otonom moda geçiş işlemi
            CleanupSharedMemory("raw_frame");
            CleanupSharedMemory("processed_frame");
            WriteToSharedMemory("1"); // "Otonom" mod için 1 yaz

            // "Aramayı sürdür" butonunun metnini kaldır
            //btnYarıOto.Text = ""; // Buton üzerindeki yazıyı kaldır

            textLog.AppendText("Otonom Moda Geçildi\n");
        }

        private void BtnManuel_Click(object sender, EventArgs e)
        {
            ResetAllButtons();
            // Resim değiştirme işlemi
            if (isManuelClicked)
            {
                btnManual.BackgroundImage = Properties.Resources.manuel1; // Varsayılan resim
                isManuelClicked = false;
            }
            else
            {
                btnManual.BackgroundImage = Properties.Resources.manuel3; // Tıklanmış resim
                isManuelClicked = true;
            }
            btnManual.BackgroundImageLayout = ImageLayout.Stretch;

            // Manuel moda geçiş işlemi
            if (pythonProcess == null || pythonProcess.HasExited)
            {
                AppendTextWithColor(textLog, "Hata: Sistem çalışmıyor. Lütfen önce başlatın.\n", Color.Red);
                return;
            }

            CleanupSharedMemory("raw_frame");
            CleanupSharedMemory("processed_frame");
            WriteToSharedMemory("2"); // "Manual" mod için 2 yaz
                                         //lblMod.Text = "Manual";
                                      //lblMod.ForeColor = Color.Yellow;
            textLog.AppendText("Manuel Moda Geçildi\n");
        }

        private void BtnYarıOtonom_Click(object sender, EventArgs e)
        {
            ResetAllButtons();
            // Resim değiştirme işlemi
            if (isYarıOtonomClicked)
            {
                btnYarıOto.BackgroundImage = Properties.Resources.yarıotonom1; // Varsayılan resim
                isYarıOtonomClicked = false;
            }
            else
            {
                btnYarıOto.BackgroundImage = Properties.Resources.yarıotonom3; // Tıklanmış resim
                isYarıOtonomClicked = true;
            }
            btnYarıOto.BackgroundImageLayout = ImageLayout.Stretch;

            // Yarı Manuel moda geçiş işlemi
            if (pythonProcess == null || pythonProcess.HasExited)
            {
                AppendTextWithColor(textLog, "Hata: Sistem çalışmıyor. Lütfen önce başlatın.\n", Color.Red);
                return;
            }

            CleanupSharedMemory("raw_frame");
            CleanupSharedMemory("processed_frame");
            WriteToSharedMemory("3"); // "Yarı Manuel" mod için 3 yaz
            textLog.AppendText("Yarı Otonom Moda Geçildi\n");


        }

        //DİĞER BUTONLARı sıfırlayan metod--------------

        private void ResetAllButtons()
        {
            // Otonom Butonu
            btnOtonom.BackgroundImage = Properties.Resources.otonom1; // Varsayılan resim
            btnOtonom.BackgroundImageLayout = ImageLayout.Stretch;
            isOtonomClicked = false;

            // Manuel Butonu
            btnManual.BackgroundImage = Properties.Resources.manuel1; // Varsayılan resim
            btnManual.BackgroundImageLayout = ImageLayout.Stretch;
            isManuelClicked = false;

            // Yarı Otonom Butonu
            btnYarıOto.BackgroundImage = Properties.Resources.yarıotonom1; // Varsayılan resim
            btnYarıOto.BackgroundImageLayout = ImageLayout.Stretch;
            isYarıOtonomClicked = false;

            // Atış Devam Butonu
            btn_AtışDevam.BackgroundImage = Properties.Resources.AtisaDevam3; // Varsayılan resim
            btn_AtışDevam.BackgroundImageLayout = ImageLayout.Stretch;
            isAtısdevamClicked = false;

            //Angajman Kabul Butonu
            btnAcceptEngagement.BackgroundImage = Properties.Resources.AngajmanKabutEt3; // Varsayılan resim
            btnAcceptEngagement.BackgroundImageLayout = ImageLayout.Stretch;
            isAngajmanKabuluClicked = false;


            // Atış Butonu
            button2.BackgroundImage = Properties.Resources.Atis3; // Varsayılan resim
            button2.BackgroundImageLayout = ImageLayout.Stretch;
            isAtısClicked = false;

            // yasaklı alan açısını ileten Butonu
            BtnSend_Coodinates.BackgroundImage = Properties.Resources.add3; // Varsayılan resim
            BtnSend_Coodinates.BackgroundImageLayout = ImageLayout.Stretch;
            isSendcord = false;


        }

        //BUTON MOUSE MOVELAR-----------

        // btnStart MouseMove Olayı
        private void BtnStart_MouseMove(object sender, MouseEventArgs e)
        {
            if (!isStarted) // Başlat modunda
            {
                btnStart.BackgroundImage = Properties.Resources.baslat2; // Hover resmi
            }
            else // Durdur modunda
            {
                btnStart.BackgroundImage = Properties.Resources.durdur2; // Hover resmi
            }
            btnStart.BackgroundImageLayout = ImageLayout.Stretch;
        }

        // btnStart MouseLeave Olayı
        private void BtnStart_MouseLeave(object sender, EventArgs e)
        {
            if (!isStarted) // Başlat modunda
            {
                btnStart.BackgroundImage = Properties.Resources.baslat1; // Varsayılan resim
            }
            else // Durdur modunda
            {
                btnStart.BackgroundImage = Properties.Resources.durdur1; // Varsayılan resim
            }
            btnStart.BackgroundImageLayout = ImageLayout.Stretch;
        }

        // Yarı Otonom Butonu Olayları
        private void BtnYarıOtonom_MouseMove(object sender, MouseEventArgs e)
        {
            if (!isYarıOtonomClicked)
            {
                btnYarıOto.BackgroundImage = Properties.Resources.yarıotonom2; // Hover resmi
                btnYarıOto.BackgroundImageLayout = ImageLayout.Stretch;
            }

        }

        private void BtnYarıOtonom_MouseLeave(object sender, EventArgs e)
        {
            if (!isYarıOtonomClicked)
            {
                btnYarıOto.BackgroundImage = Properties.Resources.yarıotonom1; // Varsayılan resim
                btnYarıOto.BackgroundImageLayout = ImageLayout.Stretch;
            }

        }
        // Manuel Butonu Olayları
        private void BtnManuel_MouseMove(object sender, MouseEventArgs e)
        {
            if (!isManuelClicked)
            {
                btnManual.BackgroundImage = Properties.Resources.manuel2; // Hover resmi
                btnManual.BackgroundImageLayout = ImageLayout.Stretch;
            }
        }

        private void BtnManuel_MouseLeave(object sender, EventArgs e)
        {
            if (!isManuelClicked)
            {
                btnManual.BackgroundImage = Properties.Resources.manuel1; // Varsayılan resim
                btnManual.BackgroundImageLayout = ImageLayout.Stretch;
            }
        }
        // Otonom Butonu Olayları
        private void BtnOtonom_MouseMove(object sender, MouseEventArgs e)
        {
            if (!isOtonomClicked)
            {
                btnOtonom.BackgroundImage = Properties.Resources.otonom2; // Hover resmi
                btnOtonom.BackgroundImageLayout = ImageLayout.Stretch;
            }
        }

        private void BtnOtonom_MouseLeave(object sender, EventArgs e)
        {
            if (!isOtonomClicked)
            {
                btnOtonom.BackgroundImage = Properties.Resources.otonom1; // Varsayılan resim
                btnOtonom.BackgroundImageLayout = ImageLayout.Stretch;
            }
        }

        private void button2_MouseMove(object sender, MouseEventArgs e)
        {      // Atış butonu 
            if (!isAtısClicked)
            {
                button2.BackgroundImage = Properties.Resources.Atis1; // Hover resmi
                button2.BackgroundImageLayout = ImageLayout.Stretch;
            }
        }

        private void button2_MouseLeave(object sender, EventArgs e)
        {     // Atış Butonu 
            if (!isAtısClicked)
            {
                button2.BackgroundImage = Properties.Resources.Atis3; // Varsayılan resim
                button2.BackgroundImageLayout = ImageLayout.Stretch;
            }
        }
        private void BtnAtısDevam_MouseMove(object sender, MouseEventArgs e)
        {
            if (!isAtısdevamClicked)
            {
                btn_AtışDevam.BackgroundImage = Properties.Resources.AtisaDevam1; // Hover resmi
                btn_AtışDevam.BackgroundImageLayout = ImageLayout.Stretch;
            }
        }

        private void BtnAtısDevam_MouseLeave(object sender, EventArgs e)
        {
            if (!isAtısdevamClicked)
            {
                btn_AtışDevam.BackgroundImage = Properties.Resources.AtisaDevam3; // Varsayılan resim
                btn_AtışDevam.BackgroundImageLayout = ImageLayout.Stretch;
            }
        }
        private void BtnAcceptEngagement_MouseMove(object sender, MouseEventArgs e)
        {
            if (!isAngajmanKabuluClicked)
            {
                btnAcceptEngagement.BackgroundImage = Properties.Resources.AngajmanKabutEt1; // Hover resmi
                btnAcceptEngagement.BackgroundImageLayout = ImageLayout.Stretch;
            }
        }

        private void BtnAcceptEngagement_MouseLeave(object sender, EventArgs e)
        {
            if (!isAngajmanKabuluClicked)
            {
                btnAcceptEngagement.BackgroundImage = Properties.Resources.AngajmanKabutEt3; // Varsayılan resim
                btnAcceptEngagement.BackgroundImageLayout = ImageLayout.Stretch;
            }
        }
        private void BtnSendCord_MouseMove(object sender, MouseEventArgs e)
        {
            if (!isSendcord)
            {
                BtnSend_Coodinates.BackgroundImage = Properties.Resources.add1; // Hover resmi
                BtnSend_Coodinates.BackgroundImageLayout = ImageLayout.Stretch;
            }
        }

        private void BtnSendCord_MouseLeave(object sender, EventArgs e)
        {
            if (!isSendcord)
            {
                BtnSend_Coodinates.BackgroundImage = Properties.Resources.add3; // Varsayılan resim
                BtnSend_Coodinates.BackgroundImageLayout = ImageLayout.Stretch;
            }
        }




        //SAĞ ÜST BUTONLAR ---------------------------------
        private void btnResize_Click(object sender, EventArgs e)
        {
            if (!isMaximized) // Pencere tam ekrana geçecekse
            {
                EnterFullscreen();
                isMaximized = true;
            }
            else // Pencere küçültülecekse
            {
                RestoreWindow();
                isMaximized = false;
            }
        }


        private void buttonMinimize_Click(object sender, EventArgs e)
        {
            // Pencereyi küçült
            this.WindowState = FormWindowState.Minimized;
        }

        private void button1_Click_1(object sender, EventArgs e)
        {
            StopAllPythonProcesses();
            this.Close();
        }




        //------------------------------------------------------------------------------
        //FORM 1 KAPATMA 


        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {

            if (_cameraThread != null && _cameraThread.IsAlive)
            {
                _cameraThread.Join();
            }

            if (_sharedMemoryThread != null && _sharedMemoryThread.IsAlive)
            {
                _sharedMemoryThread.Join();
            }

            if (_capture != null)
            {
                _capture.Release();
                _capture.Dispose();
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            //fire click
            ResetAllButtons();
            // Resim değiştirme işlemi
            if (isAtısClicked)
            {
                button2.BackgroundImage = Properties.Resources.Atis3; // Varsayılan resim
                isAtısClicked = false;
            }
            else
            {
                button2.BackgroundImage = Properties.Resources.Atis2; // Tıklanmış resim
                isAtısClicked = true;
            }
            button2.BackgroundImageLayout = ImageLayout.Stretch;
            try
            {
                using (var mmf = MemoryMappedFile.CreateOrOpen("FireDataMemory", 1))
                using (var accessor = mmf.CreateViewAccessor())
                {
                    bool fireCommand = true;
                    accessor.Write(0, ref fireCommand);
                }
                AppendTextWithColor(textLog, "Ateş komutu shared memory'e yazıldı.\n", Color.Green);
            }
            catch (Exception ex)
            {
                AppendTextWithColor(textLog, $"Ateş komutu yazma hatası: {ex.Message}\n", Color.Red);
            }
        }

        private void BtnAcceptEngagement_Click(object sender, EventArgs e)
        {
            ResetAllButtons();

            if (!isAngleSent)
            {
                MessageBox.Show("Koordinat girilmedi!", "Uyarı", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return; // Angajman kabul işlemi iptal ediliyor
            }

            // Resim değiştirme işlemi
            if (isAngajmanKabuluClicked)
            {
                btnAcceptEngagement.BackgroundImage = Properties.Resources.AngajmanKabutEt3;
                isAngajmanKabuluClicked = false;
            }
            else
            {
                btnAcceptEngagement.BackgroundImage = Properties.Resources.AngajmanKabutEt2;
                isAngajmanKabuluClicked = true;
            }
            btnAcceptEngagement.BackgroundImageLayout = ImageLayout.Stretch;

            try
            {
                CleanupSharedMemory("raw_frame");
                CleanupSharedMemory("processed_frame");
                WriteToSharedMemory("4");

                using (var mmf = MemoryMappedFile.CreateOrOpen("EngagementDataMemory", 1))
                using (var accessor = mmf.CreateViewAccessor())
                {
                    bool engagementAccepted = true;
                    accessor.Write(0, ref engagementAccepted);
                }

                AppendTextWithColor(textLog, "Angajman kabul bilgisi shared memory'e yazıldı.\n", Color.Green);
            }
            catch (Exception ex)
            {
                AppendTextWithColor(textLog, $"Angajman kabul bilgisi yazma hatası: {ex.Message}\n", Color.Red);
            }
        }

        private void btn_AtışDevam_Click(object sender, EventArgs e)
        {

            ResetAllButtons();
            // Resim değiştirme işlemi
            if (isAtısdevamClicked)
            {
                btn_AtışDevam.BackgroundImage = Properties.Resources.AtisaDevam3; // Varsayılan resim
                isAtısdevamClicked = false;
            }
            else
            {
                btn_AtışDevam.BackgroundImage = Properties.Resources.AtisaDevam2; // Tıklanmış resim
                isAtısdevamClicked = true;
            }
            btn_AtışDevam.BackgroundImageLayout = ImageLayout.Stretch;


            try
            {
                // Ortak belleğe "Ateşe Devam" komutunu yazma işlemi
                using (var mmf = MemoryMappedFile.CreateOrOpen("FireCommandMemory", 1)) // Boolean için 1 byte yeterli
                using (var accessor = mmf.CreateViewAccessor())
                {
                    bool continueFireCommand = true; // Ateşe devam komutu
                    accessor.Write(0, ref continueFireCommand);
                }

                // Log mesajı ekleme
                AppendTextWithColor(textLog, "Ateşe devam komutu shared memory'e yazıldı.\n", Color.Green);
            }
            catch (Exception ex)
            {
                // Hata durumunda log mesajı ekleme
                AppendTextWithColor(textLog, $"Ateşe devam komutu yazma hatası: {ex.Message}\n", Color.Red);
            }
        }

        private async void BtnSend_Coodinates_Click(object sender, EventArgs e)
        {
            ResetAllButtons();
            isSendcord = true;
            BtnSend_Coodinates.BackgroundImage = Properties.Resources.add2;

            int minAngle = (int)nudMinAngle.Value;
            int maxAngle = (int)nudMaxAngle.Value;

            if (minAngle < maxAngle)
            {
                try
                {
                    string angleRange = $"{minAngle},{maxAngle}";

                    using (var mmf = MemoryMappedFile.CreateOrOpen("AngleDataMemory", 2048))
                    using (var accessor = mmf.CreateViewAccessor())
                    {
                        byte[] angleBytes = Encoding.UTF8.GetBytes(angleRange);
                        accessor.WriteArray(0, angleBytes, 0, angleBytes.Length);
                    }

                    AppendTextWithColor(textLog, $"Açı aralığı shared memory'e yazıldı: {angleRange}\n", Color.Green);

                    isAngleSent = true; // Açı başarıyla gönderildi
                }
                catch (Exception ex)
                {
                    AppendTextWithColor(textLog, $"Açı aralığı yazma hatası: {ex.Message}\n", Color.Red);
                    isAngleSent = false;
                }
            }
            else
            {
                AppendTextWithColor(textLog, "Minimum açı, maksimum açıdan küçük olmalıdır.\n", Color.Red);
                isAngleSent = false;
            }

            await Task.Delay(2000);
            BtnSend_Coodinates.BackgroundImage = Properties.Resources.add3;
            isSendcord = false;
        }

        private void nudMinAngle_ValueChanged(object sender, EventArgs e)
        {

        }

        private void textLog_TextChanged(object sender, EventArgs e)
        {

        }
    }
}
