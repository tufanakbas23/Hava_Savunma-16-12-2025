// joystick_test.ino
// Sadece joystick ile pan-tilt test kodu
// Sorun tespiti için basit versiyon
// Serial Monitor'dan joystick değerlerini ve motor hızlarını izleyebilirsin

#include <AccelStepper.h>

// ============== PİNLER (ana koddan alındı) ==============
const int PAN_STEP_PIN   = 5;
const int PAN_DIR_PIN    = 6;
const int PAN_ENABLE_PIN = 7;

const int TILT_STEP_PIN   = 2;
const int TILT_DIR_PIN    = 3;
const int TILT_ENABLE_PIN = 4;

const int JOY_X_PIN  = A0;  // Pan ekseni
const int JOY_Y_PIN  = A1;  // Tilt ekseni

// ============== KALİBRASYON ==============
const float pulsesPerDegreePan  = (3200.0 * 3.0) / 360.0;  // 26.67
const float pulsesPerDegreeTilt = (3200.0 * 1.0) / 360.0;  // 8.89

// ============== JOYSTICK ==============
const int JOY_DEADZONE = 50;  // Biraz daha geniş deadzone (gürültü filtresi)
int joyCenterX = 512;  // Başlangıçta kalibrasyon yapılacak
int joyCenterY = 512;

// ============== HIZ ==============
const float MAX_SPEED_PAN  = 80.0 * pulsesPerDegreePan;   // 80 derece/sn
const float MAX_SPEED_TILT = 80.0 * pulsesPerDegreeTilt;  // 80 derece/sn

// ============== MOTORLAR ==============
AccelStepper panMotor(AccelStepper::DRIVER, PAN_STEP_PIN, PAN_DIR_PIN);
AccelStepper tiltMotor(AccelStepper::DRIVER, TILT_STEP_PIN, TILT_DIR_PIN);

// ============== DEBUG ZAMANLAYICI ==============
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 300;  // 300ms'de bir yazdır

void setup()
{
    Serial.begin(115200);
    
    // Motor enable
    pinMode(PAN_ENABLE_PIN, OUTPUT);
    pinMode(TILT_ENABLE_PIN, OUTPUT);
    digitalWrite(PAN_ENABLE_PIN, LOW);   // LOW = aktif
    digitalWrite(TILT_ENABLE_PIN, LOW);
    
    // Motor ayarları
    panMotor.setMaxSpeed(MAX_SPEED_PAN);
    panMotor.setCurrentPosition(0);
    
    tiltMotor.setMaxSpeed(MAX_SPEED_TILT);
    tiltMotor.setCurrentPosition(0);
    
    // Joystick merkez kalibrasyonu (başlangıçta oku)
    delay(500);
    joyCenterX = analogRead(JOY_X_PIN);
    joyCenterY = analogRead(JOY_Y_PIN);
    
    Serial.println(F("=== JOYSTICK TEST BASLADI ==="));
    Serial.print(F("Joystick merkez X: ")); Serial.println(joyCenterX);
    Serial.print(F("Joystick merkez Y: ")); Serial.println(joyCenterY);
    Serial.println(F("Joystick'i her yone hareket ettir ve Serial Monitor'u izle"));
    Serial.println(F("Format: rawX | rawY | panHiz | tiltHiz"));
    Serial.println(F("================================"));
}

void loop()
{
    // Ham joystick değerlerini oku
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);
    
    // Merkeze göre fark hesapla
    int diffX = rawX - joyCenterX;
    int diffY = rawY - joyCenterY;
    
    // Pan hızı hesapla
    float panSpeed = 0.0;
    if (abs(diffX) > JOY_DEADZONE)
    {
        float factor = (float)(abs(diffX) - JOY_DEADZONE) / (float)(512 - JOY_DEADZONE);
        if (factor > 1.0) factor = 1.0;
        panSpeed = factor * MAX_SPEED_PAN;
        if (diffX < 0) panSpeed = -panSpeed;  // Yön
    }
    
    // Tilt hızı hesapla
    float tiltSpeed = 0.0;
    if (abs(diffY) > JOY_DEADZONE)
    {
        float factor = (float)(abs(diffY) - JOY_DEADZONE) / (float)(512 - JOY_DEADZONE);
        if (factor > 1.0) factor = 1.0;
        tiltSpeed = factor * MAX_SPEED_TILT;
        if (diffY < 0) tiltSpeed = -tiltSpeed;  // Yön
    }
    
    // Motorları çalıştır
    panMotor.setSpeed(panSpeed);
    tiltMotor.setSpeed(tiltSpeed);
    panMotor.runSpeed();
    tiltMotor.runSpeed();
    
    // Debug yazdırma (her 300ms)
    if (millis() - lastPrint > PRINT_INTERVAL)
    {
        lastPrint = millis();
        
        Serial.print(F("X:"));
        Serial.print(rawX);
        Serial.print(F(" Y:"));
        Serial.print(rawY);
        Serial.print(F(" | diffX:"));
        Serial.print(diffX);
        Serial.print(F(" diffY:"));
        Serial.print(diffY);
        Serial.print(F(" | panSpd:"));
        Serial.print((int)panSpeed);
        Serial.print(F(" tiltSpd:"));
        Serial.print((int)tiltSpeed);
        Serial.print(F(" | panPos:"));
        Serial.print(panMotor.currentPosition());
        Serial.print(F(" tiltPos:"));
        Serial.println(tiltMotor.currentPosition());
    }
}
