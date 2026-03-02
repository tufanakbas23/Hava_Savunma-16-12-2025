# hardware/arduino_interface.py
# Hibrit Pan-Tilt Arayüzü
# Otonom (Python) + Manuel (Joystick) kontrolü destekler

import time
import threading
import serial
from serial import SerialException


class ArduinoInterface:
    """
    Arduino pan-tilt sistemi ile iletişim sınıfı.
    
    Protokol:
        Gönderim: P{pan*10},T{tilt*10},F{0|1}
        Cevap:    OK,P{pan*10},T{tilt*10},F{0|1}
        Durum:    ?  ->  STATUS,MODE={AUTO|MANUAL},P{pan},T{tilt},F{fire},ENABLED={0|1}
    """
    
    def __init__(self, port="COM4", baudrate=115200, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        
        # Bağlantıyı kur
        self._connect()
        
        # Merkez-referanslı hedef açılar (derece, 0 = merkez)
        self.pan_deg = 0.0
        self.tilt_deg = 0.0
        
        # Mevcut pozisyonlar (Arduino'dan okunan)
        self.current_pan_deg = 0.0
        self.current_tilt_deg = 0.0
        
        # Açısal hızlar (deg/s) - motion compensation için
        self.omega_pan = 0.0
        self.omega_tilt = 0.0
        self._last_pan_deg = 0.0
        self._last_tilt_deg = 0.0
        self._last_omega_time = time.time()
        
        # Şartname limitleri (merkez 0)
        self.pan_min, self.pan_max = -135.0, +135.0
        self.tilt_min, self.tilt_max = -30.0, +30.0
        
        # Kalibrasyon kazançları
        self.pan_gain = 1.0
        self.tilt_gain = 1.0
        
        # Yumuşatma parametreleri
        self.smoothing_alpha = 0.8  # Yüksek = daha hızlı tepki
        
        # Mod durumu
        self.autonomous_mode = False
        self.motors_enabled = True
        self.laser_on = False
        
        # Cevap tamponu
        self._response_lock = threading.Lock()
        self._last_response = ""
    
    def _connect(self):
        """Arduino'ya bağlan."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2.0)  # Arduino reset için bekle
            # Buffer'ı temizle
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"[INFO] Arduino {self.port} bağlandı.")
        except SerialException as e:
            print(f"[WARN] Arduino {self.port} açılmadı: {e}")
            self.ser = None
    
    def is_connected(self) -> bool:
        """Arduino bağlı mı?"""
        return self.ser is not None and self.ser.is_open
    
    def _clamp(self, val, vmin, vmax):
        """Değeri limitler arasında tut."""
        return max(vmin, min(vmax, val))
    
    def _send_raw(self, cmd: str) -> str:
        """Ham komut gönder ve cevap al."""
        if self.ser is None:
            return ""
        
        try:
            # Komutu gönder
            self.ser.write((cmd + "\n").encode("ascii"))
            self.ser.flush()
            
            # Cevap bekle
            time.sleep(0.01)  # Kısa bekleme
            
            response = ""
            while self.ser.in_waiting > 0:
                response += self.ser.read(self.ser.in_waiting).decode("ascii", errors="ignore")
            
            return response.strip()
            
        except SerialException as e:
            print(f"[WARN] Seri iletişim hatası: {e}")
            return ""
    
    # ==================== ANA KONTROL METODLARİ ====================
    
    def update_angles(self, d_pan: float, d_tilt: float):
        """
        Delta açı ile pozisyon güncelle.
        
        Args:
            d_pan: Pan açısı değişimi (derece)
            d_tilt: Tilt açısı değişimi (derece)
        """
        self.pan_deg = self._clamp(
            self.pan_deg + d_pan * self.pan_gain, 
            self.pan_min, 
            self.pan_max
        )
        self.tilt_deg = self._clamp(
            self.tilt_deg + d_tilt * self.tilt_gain, 
            self.tilt_min, 
            self.tilt_max
        )
    
    def update_angles_smooth(self, d_pan: float, d_tilt: float, alpha: float = None):
        """
        Yumuşatılmış delta açı güncellemesi (IBVS için).
        
        Args:
            d_pan: Pan açısı değişimi (derece)
            d_tilt: Tilt açısı değişimi (derece)
            alpha: Yumuşatma katsayısı (0-1, düşük = daha yumuşak)
        """
        if alpha is None:
            alpha = self.smoothing_alpha
        
        # Exponential moving average
        target_pan = self.pan_deg + d_pan * self.pan_gain
        target_tilt = self.tilt_deg + d_tilt * self.tilt_gain
        
        self.pan_deg = self._clamp(
            self.pan_deg * (1 - alpha) + target_pan * alpha,
            self.pan_min,
            self.pan_max
        )
        self.tilt_deg = self._clamp(
            self.tilt_deg * (1 - alpha) + target_tilt * alpha,
            self.tilt_min,
            self.tilt_max
        )
    
    def set_absolute_position(self, pan_deg: float, tilt_deg: float):
        """
        Mutlak pozisyon ayarla.
        
        Args:
            pan_deg: Pan açısı (derece)
            tilt_deg: Tilt açısı (derece)
        """
        self.pan_deg = self._clamp(pan_deg, self.pan_min, self.pan_max)
        self.tilt_deg = self._clamp(tilt_deg, self.tilt_min, self.tilt_max)
    
    def send(self, fire: bool = False):
        """
        Mevcut hedef açıları Arduino'ya gönder.
        
        Args:
            fire: Lazer/ateş aktif mi?
        """
        if self.ser is None:
            return
        
        self.laser_on = fire
        
        # Açısal hızı hesapla (motion compensation için)
        self._update_omega()
        
        # Derece*10 → int (0.1° çözünürlük)
        p10 = int(round(self.pan_deg * 10))
        t10 = int(round(self.tilt_deg * 10))
        
        cmd = f"P{p10},T{t10},F{1 if fire else 0}"
        
        try:
            self.ser.write((cmd + "\n").encode("ascii"))
            # NOT: Cevap okuma kaldırıldı - gecikmeyi önlemek için
                
        except SerialException as e:
            print(f"[WARN] Seri yazma hatası: {e}")
    
    def _update_omega(self):
        """Açısal hızı hesapla."""
        now = time.time()
        dt = now - self._last_omega_time
        
        if dt > 0.001:  # Minimum dt
            self.omega_pan = (self.pan_deg - self._last_pan_deg) / dt
            self.omega_tilt = (self.tilt_deg - self._last_tilt_deg) / dt
            
            self._last_pan_deg = self.pan_deg
            self._last_tilt_deg = self.tilt_deg
            self._last_omega_time = now
    
    def _parse_response(self, response: str):
        """Arduino cevabını parse et."""
        lines = response.strip().split("\n")
        for line in lines:
            line = line.strip()
            if line.startswith("OK,P") or line.startswith("STATUS,") or line.startswith("S,"):
                # Pozisyon bilgisini çıkar
                try:
                    if "P" in line:
                        p_start = line.index("P") + 1
                        p_end = line.index(",", p_start)
                        self.current_pan_deg = int(line[p_start:p_end]) / 10.0
                    
                    if "T" in line:
                        t_start = line.index("T") + 1
                        # Sonraki virgül veya satır sonu
                        t_end = line.find(",", t_start)
                        if t_end < 0:
                            t_end = len(line)
                        self.current_tilt_deg = int(line[t_start:t_end]) / 10.0
                except (ValueError, IndexError):
                    pass
    
    # ==================== MOTION COMPENSATION ====================
    
    def get_omega(self) -> tuple:
        """
        Mevcut açısal hızları döndür (motion compensation için).
        
        Returns:
            (omega_pan, omega_tilt): deg/s cinsinden açısal hızlar
        """
        return self.omega_pan, self.omega_tilt
    
    def get_current_position(self) -> tuple:
        """
        Arduino'dan son okunan mevcut pozisyonu döndür.
        
        Returns:
            (pan_deg, tilt_deg): Derece cinsinden açılar
        """
        return self.current_pan_deg, self.current_tilt_deg
    
    def get_target_position(self) -> tuple:
        """
        Hedef pozisyonu döndür.
        
        Returns:
            (pan_deg, tilt_deg): Derece cinsinden açılar
        """
        return self.pan_deg, self.tilt_deg
    
    # ==================== ÖZEL KOMUTLAR ====================
    
    def go_home(self):
        """Home pozisyonuna git (0, 0)."""
        self.pan_deg = 0.0
        self.tilt_deg = 0.0
        self._send_raw("H")
    
    def set_mode(self, autonomous: bool):
        """
        Kontrol modunu ayarla.
        
        Args:
            autonomous: True = Python kontrol, False = Joystick kontrol
        """
        cmd = "M1" if autonomous else "M0"
        self._send_raw(cmd)
        self.autonomous_mode = autonomous
    
    def get_status(self) -> dict:
        """
        Arduino durumunu sorgula.
        
        Returns:
            dict: Durum bilgileri
        """
        response = self._send_raw("?")
        
        status = {
            "mode": "UNKNOWN",
            "pan": 0.0,
            "tilt": 0.0,
            "fire": False,
            "enabled": False,
            "raw": response
        }
        
        if response.startswith("S,"):
            # Yeni format: S,mode,pan,tilt,fire,enabled
            try:
                parts = response.split(",")
                if len(parts) >= 6:
                    status["mode"] = "AUTO" if parts[1] == "1" else "MANUAL"
                    self.autonomous_mode = (parts[1] == "1")
                    status["pan"] = int(parts[2]) / 10.0
                    status["tilt"] = int(parts[3]) / 10.0
                    status["fire"] = (parts[4] == "1")
                    status["enabled"] = (parts[5].strip() == "1")
                    self.motors_enabled = status["enabled"]
            except (ValueError, IndexError):
                pass
        elif "STATUS" in response:
            # Eski format: STATUS,MODE=AUTO,P450,T-150,F0,ENABLED=1
            if "MODE=AUTO" in response:
                status["mode"] = "AUTO"
                self.autonomous_mode = True
            elif "MODE=MANUAL" in response:
                status["mode"] = "MANUAL"
                self.autonomous_mode = False
            
            if "ENABLED=1" in response:
                status["enabled"] = True
                self.motors_enabled = True
            elif "ENABLED=0" in response:
                status["enabled"] = False
                self.motors_enabled = False
        
        return status
    
    def send_stage_led(self, stage: int):
        """
        Stage bilgisini gönder (LED kontrolü için).
        
        Args:
            stage: Stage numarası (0, 1, 2, 3)
        """
        if stage not in (0, 1, 2, 3):
            return
        
        self._send_raw(str(stage))
    
    def fire(self, duration_ms: int = 100):
        """
        Lazeri belirli süre aktif et.
        
        Args:
            duration_ms: Aktif süre (milisaniye)
        """
        self.send(fire=True)
        time.sleep(duration_ms / 1000.0)
        self.send(fire=False)
    
    # ==================== KALIBRASYON ====================
    
    def set_gains(self, pan_gain: float, tilt_gain: float):
        """
        Kalibrasyon kazançlarını ayarla.
        
        Args:
            pan_gain: Pan kazancı (çarpan)
            tilt_gain: Tilt kazancı (çarpan)
        """
        self.pan_gain = pan_gain
        self.tilt_gain = tilt_gain
    
    def set_smoothing(self, alpha: float):
        """
        Yumuşatma katsayısını ayarla.
        
        Args:
            alpha: 0-1 arası (düşük = daha yumuşak)
        """
        self.smoothing_alpha = self._clamp(alpha, 0.1, 1.0)
    
    def set_limits(self, pan_min: float, pan_max: float, tilt_min: float, tilt_max: float):
        """
        Açı limitlerini ayarla.
        
        Args:
            pan_min, pan_max: Pan limitleri (derece)
            tilt_min, tilt_max: Tilt limitleri (derece)
        """
        self.pan_min = pan_min
        self.pan_max = pan_max
        self.tilt_min = tilt_min
        self.tilt_max = tilt_max
    
    # ==================== CLEANUP ====================
    
    def close(self):
        """Bağlantıyı kapat."""
        if self.ser is not None and self.ser.is_open:
            # Home'a git ve lazeri kapat
            try:
                self.go_home()
                self.send(fire=False)
                time.sleep(0.1)
            except:
                pass
            
            self.ser.close()
            print(f"[INFO] Arduino {self.port} bağlantısı kapatıldı.")
    
    def __enter__(self):
        """Context manager için."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager için."""
        self.close()
        return False


# ==================== TEST ====================

if __name__ == "__main__":
    print("ArduinoInterface Test")
    print("=" * 40)
    
    with ArduinoInterface(port="COM4") as arduino:
        if not arduino.is_connected():
            print("Arduino bağlı değil!")
            exit(1)
        
        # Durum sorgula
        status = arduino.get_status()
        print(f"Durum: {status}")
        
        # Test hareketleri
        print("\nTest: Pan sağa...")
        arduino.set_absolute_position(30, 0)
        arduino.send()
        time.sleep(1)
        
        print("Test: Tilt yukarı...")
        arduino.set_absolute_position(30, 15)
        arduino.send()
        time.sleep(1)
        
        print("Test: Home...")
        arduino.go_home()
        time.sleep(1)
        
        print("\nTest tamamlandı!")
