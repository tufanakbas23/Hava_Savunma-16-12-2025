// pan_tilt_hybrid.ino
// Hibrit Mod: Otonom (Python) + Manuel (Joystick) Kontrolü
// Joystick butonu ile mod değişimi yapılır

#include <AccelStepper.h>

// ============== PIN TANIMLARI ==============
const int PAN_STEP_PIN   = 5;
const int PAN_DIR_PIN    = 6;
const int PAN_ENABLE_PIN = 7;

const int TILT_STEP_PIN   = 2;
const int TILT_DIR_PIN    = 3;
const int TILT_ENABLE_PIN = 4;

const int JOY_X_PIN = A0;
const int JOY_Y_PIN = A1;
const int JOY_SW_PIN = 8;

const int LASER_PIN = 9;  // Lazer/ateş pini

// ============== KALİBRASYON DEĞERLERİ ==============
// Motor: 20 diş, 1/16 microstep (3200 pulse/tur)

// Pan: 20 diş motor → 60 diş platform (3:1 kayış)
const float pulsesPerRevPan = 3200.0;
const float gearRatioPan = 3.0;
const float pulsesPerDegreePan = (3200.0 * 3.0) / 360.0;  // = 26.67

// Tilt: 20 diş motor → 20 diş platform (1:1 kayış)
const float pulsesPerRevTilt = 3200.0;
const float gearRatioTilt = 1.0;
const float pulsesPerDegreeTilt = (3200.0 * 1.0) / 360.0;  // = 8.89

// ============== JOYSTICK PARAMETRELERİ ==============
const int JOY_CENTER = 512;
const int JOY_DEADZONE = 40;

// ============== HIZ PARAMETRELERİ ==============
const float TARGET_SPEED_DEG_PER_SEC = 90.0;  // 45→90 derece/saniye

// Pan için pulse/saniye
const float MAX_SPEED_PAN = TARGET_SPEED_DEG_PER_SEC * pulsesPerDegreePan;
// Tilt için pulse/saniye
const float MAX_SPEED_TILT = TARGET_SPEED_DEG_PER_SEC * pulsesPerDegreeTilt;

// İvme - yumuşak başlangıç için düşürüldü
const float ACCELERATION_PAN = 3000.0;   // 5000→3000 (kasılma önleme)
const float ACCELERATION_TILT = 1000.0;  // 2000→1000 (kasılma önleme)

// ============== POZİSYON LİMİTLERİ (derece) ==============
const float PAN_MIN_DEG = -135.0;
const float PAN_MAX_DEG = 135.0;
const float TILT_MIN_DEG = -30.0;
const float TILT_MAX_DEG = 30.0;

// ============== STEPPER NESNELERİ ==============
AccelStepper panMotor(AccelStepper::DRIVER, PAN_STEP_PIN, PAN_DIR_PIN);
AccelStepper tiltMotor(AccelStepper::DRIVER, TILT_STEP_PIN, TILT_DIR_PIN);

// ============== DURUM DEĞİŞKENLERİ ==============
bool motorsEnabled = true;
bool autonomousMode = false;  // false = Manuel (joystick), true = Otonom (Python)
float panSpeed = 0.0;
float tiltSpeed = 0.0;

// Hedef pozisyonlar (derece, 0 = merkez)
float targetPanDeg = 0.0;
float targetTiltDeg = 0.0;

// Mevcut pozisyonlar
float currentPanDeg = 0.0;
float currentTiltDeg = 0.0;

// Lazer durumu
bool laserOn = false;

// Seri komut tamponu
String serialBuffer = "";
unsigned long lastSerialTime = 0;
const unsigned long SERIAL_TIMEOUT = 100;  // 100ms timeout

// NOT: Timeout kaldırıldı - Mod değişimi sadece Python'dan M0/M1 komutu ile yapılır

void setup()
{
    Serial.begin(115200);
    
    pinMode(PAN_ENABLE_PIN, OUTPUT);
    pinMode(TILT_ENABLE_PIN, OUTPUT);
    digitalWrite(PAN_ENABLE_PIN, LOW);
    digitalWrite(TILT_ENABLE_PIN, LOW);
    
    pinMode(JOY_SW_PIN, INPUT_PULLUP);
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);
    
    // Pan motor ayarları
    panMotor.setMaxSpeed(MAX_SPEED_PAN);
    panMotor.setAcceleration(ACCELERATION_PAN);
    panMotor.setCurrentPosition(0);
    
    // Tilt motor ayarları
    tiltMotor.setMaxSpeed(MAX_SPEED_TILT);
    tiltMotor.setAcceleration(ACCELERATION_TILT);
    tiltMotor.setCurrentPosition(0);
    
    printBanner();
    delay(500);
}

void printBanner()
{
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  HİBRİT PAN-TILT KONTROL SİSTEMİ      ║");
    Serial.println("║  Otonom (Python) + Manuel (Joystick)   ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Komutlar:");
    Serial.println("  P{pan*10},T{tilt*10},F{0|1}  -> Pozisyon komutu");
    Serial.println("  M0 -> Manuel moda geç");
    Serial.println("  M1 -> Otonom moda geç");
    Serial.println("  H  -> Home pozisyonuna git");
    Serial.println("  ?  -> Durum sorgula");
    Serial.println();
    Serial.println("Joystick butonu: Motor ON/OFF");
    Serial.println("════════════════════════════════════════\n");
}

void loop()
{
    // Seri komutları kontrol et
    checkSerialCommands();
    
    // Joystick butonu kontrolü (sadece motor ON/OFF)
    checkModeButton();
    
    // Mod'a göre kontrol
    if (autonomousMode)
    {
        runAutonomousMode();
    }
    else
    {
        runManualMode();
    }
    
    // Motorları çalıştır
    panMotor.run();
    tiltMotor.run();
    
    // Durum yazdır (sadece debug için)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500)
    {
        printStatus();
        lastPrint = millis();
    }
}

// ==================== SERİ KOMUT İŞLEME ====================

void checkSerialCommands()
{
    while (Serial.available())
    {
        char c = Serial.read();
        lastSerialTime = millis();
        
        if (c == '\n' || c == '\r')
        {
            if (serialBuffer.length() > 0)
            {
                processCommand(serialBuffer);
                serialBuffer = "";
            }
        }
        else
        {
            serialBuffer += c;
            if (serialBuffer.length() > 50)
            {
                serialBuffer = "";  // Buffer overflow koruması
            }
        }
    }
    
    // Timeout - yarım kalan komut varsa temizle
    if (serialBuffer.length() > 0 && (millis() - lastSerialTime > SERIAL_TIMEOUT))
    {
        serialBuffer = "";
    }
}

void processCommand(String cmd)
{
    cmd.trim();
    if (cmd.length() == 0) return;
    
    // Durum sorgusu
    if (cmd == "?")
    {
        sendStatusResponse();
        return;
    }
    
    // Home komutu
    if (cmd == "H" || cmd == "h")
    {
        goHome();
        Serial.println("OK,HOME");
        return;
    }
    
    // Mod değiştirme
    if (cmd == "M0")
    {
        autonomousMode = false;
        Serial.println("OK,MODE=MANUAL");
        return;
    }
    if (cmd == "M1")
    {
        autonomousMode = true;
        Serial.println("OK,MODE=AUTO");
        return;
    }
    
    // Stage LED komutu (tek rakam)
    if (cmd.length() == 1 && cmd[0] >= '0' && cmd[0] <= '3')
    {
        // Stage LED işleme (isteğe bağlı LED kontrolü)
        Serial.print("OK,STAGE=");
        Serial.println(cmd);
        return;
    }
    
    // Pozisyon komutu: P{pan*10},T{tilt*10},F{0|1}
    if (cmd.startsWith("P"))
    {
        parsePositionCommand(cmd);
        return;
    }
    
    // Bilinmeyen komut
    Serial.print("ERR,UNKNOWN:");
    Serial.println(cmd);
}

void parsePositionCommand(String cmd)
{
    // Format: P{pan*10},T{tilt*10},F{0|1}
    // Örnek: P450,T-150,F0  -> Pan=45.0°, Tilt=-15.0°, Fire=off
    
    int pIndex = cmd.indexOf('P');
    int tIndex = cmd.indexOf('T');
    int fIndex = cmd.indexOf('F');
    
    if (pIndex < 0 || tIndex < 0)
    {
        Serial.println("ERR,FORMAT");
        return;
    }
    
    // Pan değerini parse et
    String panStr = cmd.substring(pIndex + 1, tIndex - 1);
    float panVal = panStr.toFloat() / 10.0;  // 0.1° çözünürlük
    
    // Tilt değerini parse et
    String tiltStr;
    if (fIndex > 0)
    {
        tiltStr = cmd.substring(tIndex + 1, fIndex - 1);
    }
    else
    {
        tiltStr = cmd.substring(tIndex + 1);
    }
    float tiltVal = tiltStr.toFloat() / 10.0;
    
    // Fire değerini parse et
    bool fireVal = false;
    if (fIndex > 0 && fIndex + 1 < cmd.length())
    {
        fireVal = (cmd.charAt(fIndex + 1) == '1');
    }
    
    // Limitleri uygula
    targetPanDeg = constrain(panVal, PAN_MIN_DEG, PAN_MAX_DEG);
    targetTiltDeg = constrain(tiltVal, TILT_MIN_DEG, TILT_MAX_DEG);
    
    // Lazer kontrolü
    laserOn = fireVal;
    digitalWrite(LASER_PIN, laserOn ? HIGH : LOW);
    
    // NOT: Mod değişimi kaldırıldı - Sadece otonom moddayken pozisyon komutu işlenir
    
    // Cevap gönder
    Serial.print("OK,P");
    Serial.print((int)(currentPanDeg * 10));
    Serial.print(",T");
    Serial.print((int)(currentTiltDeg * 10));
    Serial.print(",F");
    Serial.println(laserOn ? "1" : "0");
}

void sendStatusResponse()
{
    // Format: STATUS,MODE={AUTO|MANUAL},P{pan*10},T{tilt*10},F{0|1},ENABLED={0|1}
    Serial.print("STATUS,MODE=");
    Serial.print(autonomousMode ? "AUTO" : "MANUAL");
    Serial.print(",P");
    Serial.print((int)(currentPanDeg * 10));
    Serial.print(",T");
    Serial.print((int)(currentTiltDeg * 10));
    Serial.print(",F");
    Serial.print(laserOn ? "1" : "0");
    Serial.print(",ENABLED=");
    Serial.println(motorsEnabled ? "1" : "0");
}

void goHome()
{
    targetPanDeg = 0.0;
    targetTiltDeg = 0.0;
    laserOn = false;
    digitalWrite(LASER_PIN, LOW);
}

// ==================== MOTOR BUTONU ====================

void checkModeButton()
{
    static unsigned long lastPress = 0;
    static unsigned long pressStart = 0;
    bool buttonPressed = (digitalRead(JOY_SW_PIN) == LOW);
    
    if (buttonPressed)
    {
        if (pressStart == 0) 
        {
            pressStart = millis();
        }
        
        // 500ms basılı tutulursa motor toggle
        if (millis() - pressStart > 500 && millis() - lastPress > 1000)
        {
            motorsEnabled = !motorsEnabled;
            
            if (motorsEnabled)
            {
                digitalWrite(PAN_ENABLE_PIN, LOW);
                digitalWrite(TILT_ENABLE_PIN, LOW);
                Serial.println("[INFO] Motorlar AKTIF");
            }
            else
            {
                digitalWrite(PAN_ENABLE_PIN, HIGH);
                digitalWrite(TILT_ENABLE_PIN, HIGH);
                panMotor.setSpeed(0);
                tiltMotor.setSpeed(0);
                Serial.println("[INFO] Motorlar PASIF");
            }
            
            pressStart = 0;
            lastPress = millis();
        }
    }
    else
    {
        pressStart = 0;
    }
}

// ==================== OTONOM MOD ====================

void runAutonomousMode()
{
    if (!motorsEnabled)
    {
        panMotor.setSpeed(0);
        tiltMotor.setSpeed(0);
        return;
    }
    
    // Hedef pozisyonu pulse'a çevir
    long targetPanPulse = (long)(targetPanDeg * pulsesPerDegreePan);
    long targetTiltPulse = (long)(targetTiltDeg * pulsesPerDegreeTilt);
    
    // Hedefleri ayarla
    panMotor.moveTo(targetPanPulse);
    tiltMotor.moveTo(targetTiltPulse);
    
    // Mevcut pozisyonları güncelle
    currentPanDeg = panMotor.currentPosition() / pulsesPerDegreePan;
    currentTiltDeg = tiltMotor.currentPosition() / pulsesPerDegreeTilt;
}

// ==================== MANUEL MOD ====================

void runManualMode()
{
    int xValue = analogRead(JOY_X_PIN);
    int yValue = analogRead(JOY_Y_PIN);
    
    if (!motorsEnabled)
    {
        panMotor.setSpeed(0);
        tiltMotor.setSpeed(0);
        return;
    }
    
    // ========== PAN KONTROLÜ (X ekseni) ==========
    int xError = xValue - JOY_CENTER;
    
    if (abs(xError) < JOY_DEADZONE)
    {
        panMotor.setSpeed(0);
        panSpeed = 0;
    }
    else
    {
        int activeRange = abs(xError) - JOY_DEADZONE;
        int maxRange = JOY_CENTER - JOY_DEADZONE;
        
        float speedFactor = (float)activeRange / (float)maxRange;
        speedFactor = constrain(speedFactor, 0.0, 1.0);
        
        panSpeed = speedFactor * MAX_SPEED_PAN;
        
        if (panSpeed > 0 && panSpeed < 100)
            panSpeed = 100;
        
        // Limit kontrolü
        float nextPanDeg = currentPanDeg + ((xError > 0 ? 1 : -1) * 0.1);
        if (nextPanDeg >= PAN_MIN_DEG && nextPanDeg <= PAN_MAX_DEG)
        {
            if (xError > 0)
                panMotor.setSpeed(panSpeed);
            else
                panMotor.setSpeed(-panSpeed);
        }
        else
        {
            panMotor.setSpeed(0);
        }
    }
    
    // ========== TILT KONTROLÜ (Y ekseni) ==========
    int yError = yValue - JOY_CENTER;
    
    if (abs(yError) < JOY_DEADZONE)
    {
        tiltMotor.setSpeed(0);
        tiltSpeed = 0;
    }
    else
    {
        int activeRange = abs(yError) - JOY_DEADZONE;
        int maxRange = JOY_CENTER - JOY_DEADZONE;
        
        float speedFactor = (float)activeRange / (float)maxRange;
        speedFactor = constrain(speedFactor, 0.0, 1.0);
        
        tiltSpeed = speedFactor * MAX_SPEED_TILT;
        
        if (tiltSpeed > 0 && tiltSpeed < 30)
            tiltSpeed = 30;
        
        // Limit kontrolü
        float nextTiltDeg = currentTiltDeg + ((yError > 0 ? 1 : -1) * 0.1);
        if (nextTiltDeg >= TILT_MIN_DEG && nextTiltDeg <= TILT_MAX_DEG)
        {
            if (yError > 0)
                tiltMotor.setSpeed(tiltSpeed);
            else
                tiltMotor.setSpeed(-tiltSpeed);
        }
        else
        {
            tiltMotor.setSpeed(0);
        }
    }
    
    // Manuel modda motorları hız modunda çalıştır
    panMotor.runSpeed();
    tiltMotor.runSpeed();
    
    // Mevcut pozisyonları güncelle
    currentPanDeg = panMotor.currentPosition() / pulsesPerDegreePan;
    currentTiltDeg = tiltMotor.currentPosition() / pulsesPerDegreeTilt;
    
    // Hedefleri de güncelle (otonom moda geçişte ani sıçrama olmasın)
    targetPanDeg = currentPanDeg;
    targetTiltDeg = currentTiltDeg;
}

// ==================== DURUM YAZDIR ====================

void printStatus()
{
    float panDegPerSec = panSpeed / pulsesPerDegreePan;
    float tiltDegPerSec = tiltSpeed / pulsesPerDegreeTilt;
    
    Serial.print("Mod: ");
    Serial.print(autonomousMode ? "OTONOM" : "MANUEL");
    Serial.print(" | Motor: ");
    Serial.print(motorsEnabled ? "ON" : "OFF");
    Serial.print(" | Pan: ");
    Serial.print(currentPanDeg, 1);
    Serial.print("° -> ");
    Serial.print(targetPanDeg, 1);
    Serial.print("° | Tilt: ");
    Serial.print(currentTiltDeg, 1);
    Serial.print("° -> ");
    Serial.print(targetTiltDeg, 1);
    Serial.print("° | Laser: ");
    Serial.println(laserOn ? "ON" : "OFF");
}
