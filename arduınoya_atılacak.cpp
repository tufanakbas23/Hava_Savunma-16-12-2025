// pan_tilt_hybrid.ino
// Hibrit Mod: Otonom (Python) + Manuel (Joystick)
// Optimize edildi: String yok, gereksiz loop yok

#include <AccelStepper.h>

// ============== PIN TANIMLARI ==============
const int PAN_STEP_PIN   = 5;
const int PAN_DIR_PIN    = 6;
const int PAN_ENABLE_PIN = 7;

const int TILT_STEP_PIN   = 2;
const int TILT_DIR_PIN    = 3;
const int TILT_ENABLE_PIN = 4;

const int JOY_X_PIN  = A0;
const int JOY_Y_PIN  = A1;
const int JOY_SW_PIN = 8;
const int LASER_PIN  = 9;

// ============== KALİBRASYON ==============
// Pan: 3:1 kayış (20→60 diş), 1/16 microstep
const float pulsesPerDegreePan  = (3200.0 * 3.0) / 360.0;  // = 26.67
// Tilt: 1:1 kayış (20→20 diş)
const float pulsesPerDegreeTilt = (3200.0 * 1.0) / 360.0;  // = 8.89

// ============== JOYSTICK ==============
const int JOY_CENTER   = 512;
const int JOY_DEADZONE = 40;

// ============== HIZ ==============
const float TARGET_SPEED_DEG_PER_SEC = 120.0;
const float MAX_SPEED_PAN  = TARGET_SPEED_DEG_PER_SEC * pulsesPerDegreePan;
const float MAX_SPEED_TILT = TARGET_SPEED_DEG_PER_SEC * pulsesPerDegreeTilt;
const float ACCEL_PAN  = 3000.0;
const float ACCEL_TILT = 1000.0;

// ============== LİMİTLER (derece) ==============
const float PAN_MIN_DEG  = -135.0;
const float PAN_MAX_DEG  =  135.0;
const float TILT_MIN_DEG = -30.0;
const float TILT_MAX_DEG =  30.0;

// ============== MOTORLAR ==============
AccelStepper panMotor(AccelStepper::DRIVER, PAN_STEP_PIN, PAN_DIR_PIN);
AccelStepper tiltMotor(AccelStepper::DRIVER, TILT_STEP_PIN, TILT_DIR_PIN);

// ============== DURUM ==============
bool motorsEnabled   = true;
bool autonomousMode  = false;
bool laserOn         = false;

float targetPanDeg   = 0.0;
float targetTiltDeg  = 0.0;
float currentPanDeg  = 0.0;
float currentTiltDeg = 0.0;

// ============== SERİ BUFFER (String yok!) ==============
char cmdBuf[32];
uint8_t cmdLen = 0;

// ============== Stage 2: Error_X / Error_Y tabanlı PD kontrol ==============
// Python tarafı: E+0123,Y-0456 → Error_X/Y_norm ≈ ±0.123, ±0.456

float kp_pan  = 0.08f;
float kd_pan  = 0.02f;
float kp_tilt = 0.08f;
float kd_tilt = 0.02f;

float last_err_pan  = 0.0f;
float last_err_tilt = 0.0f;
unsigned long last_err_time_ms = 0;

// ==================== İLERİ DEKLARASYON ====================
void processCmd(const char* cmd);
void parsePosition(const char* cmd);
void parseErrorPacket(const char* cmd);
void sendStatus();
void runAutonomous();
float readAxis(int pin, float maxSpeed, float minSpeed);
void runManual();
void checkButton();

// ==================== SETUP ====================

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
    
    panMotor.setMaxSpeed(MAX_SPEED_PAN);
    panMotor.setAcceleration(ACCEL_PAN);
    panMotor.setCurrentPosition(0);
    
    tiltMotor.setMaxSpeed(MAX_SPEED_TILT);
    tiltMotor.setAcceleration(ACCEL_TILT);
    tiltMotor.setCurrentPosition(0);
    
    Serial.println(F("PAN-TILT READY"));
    delay(200);
}

// ==================== LOOP ====================

void loop()
{
    // Serial komutları her zaman kontrol et
    while (Serial.available())
    {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r')
        {
            if (cmdLen > 0)
            {
                cmdBuf[cmdLen] = '\0';
                processCmd(cmdBuf);
                cmdLen = 0;
            }
        }
        else if (cmdLen < (sizeof(cmdBuf) - 1))  // Buffer overflow koruması
        {
            cmdBuf[cmdLen++] = c;
        }
        else
        {
            cmdLen = 0;  // Taşma → sıfırla
        }
    }
    
    if (autonomousMode)
    {
        runAutonomous();
        // Pozisyon modu: run() sadece otonom modda
        panMotor.run();
        tiltMotor.run();
    }
    else
    {
        // Joystick sadece manuelde okunur
        checkButton();
        runManual();  // runSpeed() kendi içinde çağrılıyor
    }
}

// ==================== SERİ KOMUT ====================

void processCmd(const char* cmd)
{
    // Durum sorgusu
    if (cmd[0] == '?')
    {
        sendStatus();
        return;
    }
    
    // Home
    if (cmd[0] == 'H' || cmd[0] == 'h')
    {
        targetPanDeg = 0.0;
        targetTiltDeg = 0.0;
        laserOn = false;
        digitalWrite(LASER_PIN, LOW);
        Serial.println(F("OK,HOME"));
        return;
    }
    
    // Mod değiştirme
    if (cmd[0] == 'M')
    {
        if (cmd[1] == '0') { autonomousMode = false; Serial.println(F("OK,M0")); }
        else if (cmd[1] == '1') { autonomousMode = true; Serial.println(F("OK,M1")); }
        return;
    }
    
    // Stage LED (0-3) — sadece status/ack için
    if (cmd[0] >= '0' && cmd[0] <= '3' && cmd[1] == '\0')
    {
        Serial.print(F("OK,S"));
        Serial.println(cmd[0]);
        return;
    }
    
    // Pozisyon komutu: P{pan*10},T{tilt*10},F{0|1}
    if (cmd[0] == 'P')
    {
        parsePosition(cmd);
        return;
    }

    // Hata tabanlı hedef: E{EX},Y{EY}  (Stage 2 - Error_X, Error_Y normalize)
    if (cmd[0] == 'E')
    {
        parseErrorPacket(cmd);
        return;
    }
}

void parsePosition(const char* cmd)
{
    // P450,T-150,F1 formatı
    // atoi/atof ile hızlı parse
    
    const char* pPtr = cmd + 1;  // P'den sonra
    const char* tPtr = strchr(cmd, 'T');
    const char* fPtr = strchr(cmd, 'F');
    
    if (!tPtr) return;  // T bulunamadı → geçersiz
    
    float panVal  = atol(pPtr) / 10.0;
    float tiltVal = atol(tPtr + 1) / 10.0;
    bool fireVal  = (fPtr && *(fPtr + 1) == '1');
    
    targetPanDeg  = constrain(panVal, PAN_MIN_DEG, PAN_MAX_DEG);
    targetTiltDeg = constrain(tiltVal, TILT_MIN_DEG, TILT_MAX_DEG);
    
    laserOn = fireVal;
    digitalWrite(LASER_PIN, laserOn ? HIGH : LOW);
    
    // Kısa yanıt
    Serial.print(F("OK,P"));
    Serial.print((int)(currentPanDeg * 10));
    Serial.print(F(",T"));
    Serial.println((int)(currentTiltDeg * 10));
}

// Stage 2 için: Error_X / Error_Y normalize paketini parse et
// Format:  E+0123,Y-0456
// EX, EY = Error_*_norm * 1000  (işaretli tam sayı)
void parseErrorPacket(const char* cmd)
{
    const char* ex_ptr = cmd + 1;           // 'E' den sonrası (+/- rakamlar)
    const char* comma  = strchr(cmd, ',');
    if (!comma) return;

    const char* y_ptr = strchr(cmd, 'Y');
    if (!y_ptr) return;
    const char* ey_ptr = y_ptr + 1;         // 'Y' den sonrası

    // Küçük parse: işaret + rakamlar → int
    int ex = atol(ex_ptr);   // örn: +0123 → 123, -0456 → -456
    int ey = atol(ey_ptr);

    // Normalize geri ölçekle
    float err_pan_norm  = (float)ex / 1000.0f;  // Error_X
    float err_tilt_norm = (float)ey / 1000.0f;  // Error_Y

    // Zaman farkı
    unsigned long now_ms = millis();
    float dt_s = 0.0f;
    if (last_err_time_ms > 0)
    {
        unsigned long dt_ms = now_ms - last_err_time_ms;
        if (dt_ms > 0) dt_s = (float)dt_ms / 1000.0f;
    }
    last_err_time_ms = now_ms;

    // Türev terimi
    float d_err_pan  = (dt_s > 0.0f) ? (err_pan_norm  - last_err_pan)  / dt_s : 0.0f;
    float d_err_tilt = (dt_s > 0.0f) ? (err_tilt_norm - last_err_tilt) / dt_s : 0.0f;

    last_err_pan  = err_pan_norm;
    last_err_tilt = err_tilt_norm;

    // Normalize hatayı dereceye çevir (ZED HFOV/VFOV tahmini)
    const float HFOV_DEG = 78.0f;
    const float VFOV_DEG = 50.0f;

    float err_pan_deg  = err_pan_norm  * (HFOV_DEG / 2.0f);
    float err_tilt_deg = err_tilt_norm * (VFOV_DEG / 2.0f);

    // Basit PD: hedef açılara küçük düzeltme ekle
    float delta_pan_deg  = -(kp_pan  * err_pan_deg  + kd_pan  * d_err_pan);
    float delta_tilt_deg = -(kp_tilt * err_tilt_deg + kd_tilt * d_err_tilt);

    targetPanDeg  += delta_pan_deg;
    targetTiltDeg += delta_tilt_deg;

    // Limitler içinde tut
    targetPanDeg  = constrain(targetPanDeg,  PAN_MIN_DEG,  PAN_MAX_DEG);
    targetTiltDeg = constrain(targetTiltDeg, TILT_MIN_DEG, TILT_MAX_DEG);
}

void sendStatus()
{
    Serial.print(F("S,"));
    Serial.print(autonomousMode ? '1' : '0');
    Serial.print(',');
    Serial.print((int)(currentPanDeg * 10));
    Serial.print(',');
    Serial.print((int)(currentTiltDeg * 10));
    Serial.print(',');
    Serial.print(laserOn ? '1' : '0');
    Serial.print(',');
    Serial.println(motorsEnabled ? '1' : '0');
}

// ==================== OTONOM MOD ====================

void runAutonomous()
{
    if (!motorsEnabled) return;
    
    panMotor.moveTo((long)(targetPanDeg * pulsesPerDegreePan));
    tiltMotor.moveTo((long)(targetTiltDeg * pulsesPerDegreeTilt));
    
    currentPanDeg  = panMotor.currentPosition() / pulsesPerDegreePan;
    currentTiltDeg = tiltMotor.currentPosition() / pulsesPerDegreeTilt;
}

// ==================== MANUEL MOD ====================

float readAxis(int pin, float maxSpeed, float minSpeed)
{
    int val = analogRead(pin) - JOY_CENTER;
    if (abs(val) < JOY_DEADZONE) return 0.0;
    
    float factor = (float)(abs(val) - JOY_DEADZONE) / (float)(JOY_CENTER - JOY_DEADZONE);
    float speed = constrain(factor, 0.0, 1.0) * maxSpeed;
    if (speed > 0 && speed < minSpeed) speed = minSpeed;
    
    return (val > 0) ? speed : -speed;
}

void runManual()
{
    if (!motorsEnabled)
    {
        panMotor.setSpeed(0);
        tiltMotor.setSpeed(0);
        return;
    }
    
    float panSpd  = readAxis(JOY_X_PIN, MAX_SPEED_PAN, 100.0);
    float tiltSpd = readAxis(JOY_Y_PIN, MAX_SPEED_TILT, 30.0);
    
    // Limit kontrolü
    if ((panSpd > 0 && currentPanDeg >= PAN_MAX_DEG) ||
        (panSpd < 0 && currentPanDeg <= PAN_MIN_DEG))
        panSpd = 0;
    
    if ((tiltSpd > 0 && currentTiltDeg >= TILT_MAX_DEG) ||
        (tiltSpd < 0 && currentTiltDeg <= TILT_MIN_DEG))
        tiltSpd = 0;
    
    panMotor.setSpeed(panSpd);
    tiltMotor.setSpeed(tiltSpd);
    panMotor.runSpeed();
    tiltMotor.runSpeed();
    
    // Pozisyonları güncelle
    currentPanDeg  = panMotor.currentPosition() / pulsesPerDegreePan;
    currentTiltDeg = tiltMotor.currentPosition() / pulsesPerDegreeTilt;
    targetPanDeg   = currentPanDeg;
    targetTiltDeg  = currentTiltDeg;
}

// ==================== BUTON ====================

void checkButton()
{
    static unsigned long lastPress = 0;
    static unsigned long pressStart = 0;
    
    if (digitalRead(JOY_SW_PIN) == LOW)
    {
        if (pressStart == 0) pressStart = millis();
        
        if (millis() - pressStart > 500 && millis() - lastPress > 1000)
        {
            motorsEnabled = !motorsEnabled;
            digitalWrite(PAN_ENABLE_PIN, motorsEnabled ? LOW : HIGH);
            digitalWrite(TILT_ENABLE_PIN, motorsEnabled ? LOW : HIGH);
            
            if (!motorsEnabled)
            {
                panMotor.setSpeed(0);
                tiltMotor.setSpeed(0);
            }
            
            Serial.println(motorsEnabled ? F("MOT:1") : F("MOT:0"));
            pressStart = 0;
            lastPress = millis();
        }
    }
    else
    {
        pressStart = 0;
    }
}