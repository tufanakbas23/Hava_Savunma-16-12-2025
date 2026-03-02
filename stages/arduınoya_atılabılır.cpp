#include <ArduinoRS485.h>
#define RS485_SERIAL Serial1  

#include <ArduinoRS485.h>
#include <AccelStepper.h>

#define ENABLE_PIN 11
#define STEP_PIN 9
#define DIR_PIN 10

#define ENABLE_PIN_Y 8
#define STEP_PIN_Y 6
#define DIR_PIN_Y 7


#define ENABLE_PIN_Z 2
#define STEP_PIN_ATIS 3
#define DIR_PIN_ATIS 4


#define RS485_ENABLE 12


AccelStepper stepperZ(1, STEP_PIN_ATIS, DIR_PIN_ATIS);


// AccelStepper nesnesi (1 = sürücü modu, STEP, DIRECTION)
AccelStepper stepperX(1, STEP_PIN, DIR_PIN);

AccelStepper stepperY(1, STEP_PIN_Y, DIR_PIN_Y);  // Stepper Y


int mode = 0;  
bool stepperZActive = false;


const float stepsPerDegreeX = 34;
//const float stepsPerDegreeY2 = 40;

const float stepsPerDegreeZ = 8;

const float stepsPerDegree =2350.0 / 360.0; // 800 adım = 180 derece
const float stepsPerDegreeY = 1800.0 / 360.0; // 800 adım = 180 derece


const int stepsPer90DegreesATIS = 361; 

void setup() {
 pinMode(RS485_ENABLE, OUTPUT);
digitalWrite(RS485_ENABLE, LOW); // Okuma moduna geçir


  RS485_SERIAL.begin(9600);
  RS485_SERIAL.println("Yüz Algılama ile Step Motor Kontrolü Başladı!");


    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW); // Motoru etkinleştir
    
    pinMode(ENABLE_PIN_Y, OUTPUT);
    digitalWrite(ENABLE_PIN_Y, LOW); // Motoru etkinleştir

    pinMode(ENABLE_PIN_Z, OUTPUT);
    digitalWrite(ENABLE_PIN_Z, LOW); // Motoru etkinleştiz

    stepperX.setMaxSpeed(25000);  // Maksimum hız (adım/sn)
    stepperX.setAcceleration(25000); // İvme (adım/sn²)
    stepperY.setMaxSpeed(25000);  // Maksimum hız (adım/sn)
    stepperY.setAcceleration(25000); // İvme (adım/sn²)

  stepperZ.setMaxSpeed(2500);
  stepperZ.setAcceleration(2500);

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);  stepperZ.setMaxSpeed(2500);
  stepperZ.setAcceleration(2500);

}

void loop() {

 
    if (RS485_SERIAL.available() > 0) {
        String input = RS485_SERIAL.readStringUntil('\n');
        input.trim();

        if (input.length() == 0 || input.toInt() == 9999) {
            stepperX.moveTo(0);
            stepperY.moveTo(0);
        } else if (input.toInt() == 1111) {
            stepperX.moveTo(0);
            stepperY.moveTo(0);
        } else {
            int firstComma = input.indexOf(',');
            int secondComma = input.indexOf(',', firstComma + 1);
            mode = input.substring(1, firstComma).toInt();

            if (mode == 5) {
                stepperX.move(90  * stepsPerDegree); 
                  while (stepperX.isRunning()) {       // Wait for the motor to finish its movement
                        stepperX.run();                  // Continuously run the motor
                                             }
                stepperX.setCurrentPosition(0);

            } else if (mode == 7) {
                stepperX.move(-90 * stepsPerDegree);
                 while (stepperX.isRunning()) {       // Wait for the motor to finish its movement
                        stepperX.run();                  // Continuously run the motor
                                             }

                stepperX.setCurrentPosition(0);
            } else if (mode == 1) {
                ManualMode(firstComma, secondComma, input);
            } else if (mode == 2) {
                autonomousMode(firstComma, secondComma, input);
            }
             else if (mode == 3) {
               bob();
            }
        }
    }
 
 stepperX.run(); // Motorun hareketini sağlar
 stepperY.run(); // Motorun hareketini sağlar


  if (stepperZActive) {
    if (stepperZ.distanceToGo() != 0) {
      stepperZ.run();
    } else {
      stepperZActive = false;
    }
  }

 
}

void autonomousMode(int firstComma, int secondComma, String input) {
    // Yeni format: [2, x_koordinat, y_koordinat, ates_durumu]
    int targetX = input.substring(firstComma + 1, secondComma).toInt();
    
    // İkinci virgülü bul
    int thirdComma = input.indexOf(',', secondComma + 1);
    int targetY = input.substring(secondComma + 1, thirdComma).toInt();
    
    // Atış durumunu kontrol et
    int atesDurumu = input.substring(thirdComma + 1, input.length() - 1).toInt();
    
    // Önce koordinatları işle
    moveToPosition(targetX, targetY);
    
    // Eğer atış durumu 1 ise ateş et
    if (atesDurumu == 1) {
        stepperZ.move(stepsPer90DegreesATIS * stepsPerDegreeZ);  
        // bob();

    }
}

void moveToPosition(int targetX, int targetY) {
    if (targetX != stepperX.currentPosition()) {
        stepperX.move(targetX * stepsPerDegree);
    }

    if (targetY != stepperY.currentPosition()) {
        stepperY.move(targetY * stepsPerDegreeY);
    }
}




// Function to move motors based on joystick input
void moveMotorsWithJoystick() {
  static bool setmidpos = true; 
  // Read joystick values (0-1023)
  int joyX = analogRead(A0);
  int joyY = analogRead(A1);

if (setmidpos){
  stepperX.setCurrentPosition(45 * 1600L / 360);  // Set current position to 90 degrees
  stepperY.setCurrentPosition(30 * 1600L / 360);  
 setmidpos =  false; 
}
  // Map joystick values to degrees (0-180)
  int targetX = map(joyX, 0, 1023, 0, 90);
  int targetY = map(joyY, 0, 1023, 0, 60);

/*
  if (abs(joyX - 512) < 100) {
    targetX = 45; // Neutral position
  }
  if (abs(joyY - 512) < 100) {
    targetY = 30; // Neutral position
  }
*/
  // Convert degrees to steps (assuming 1600 steps per 360 degrees)
  long stepsX = targetX * 1600L / 360;
  long stepsY = targetY * 1600L / 360;

  // Move the stepper motors towards the target positions
  stepperX.moveTo(stepsX);
  stepperY.moveTo(stepsY);

  // Make the stepper motors run
  stepperX.run();
  stepperY.run();
}






void bob(){
  stepperZActive = true;

	RS485_SERIAL.println("clockwise");
  stepperZ.move(stepsPer90DegreesATIS * stepsPerDegreeZ);  

}



void step(int steps, int pin) {
  for (int x = 0; x < steps; x++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(100);
    digitalWrite(pin, LOW);
    delayMicroseconds(100);
  }
}

void ManuakMode(int firstComma,int secondComma ,String input) {
    int targetX = input.substring(firstComma + 1, secondComma).toInt();  // İkinci eleman (target_angle)
    int targetY = input.substring(secondComma + 1, input.length() - 1).toInt();  // Üçüncü eleman (targetY)

   moveToPositionManually(targetX, targetY);
  }


void ManualMode(int firstComma, int secondComma, String input) {
    // Yeni format: [M, joyX, joyY, yasakMin, yasakMax, trigger]

    int thirdComma = input.indexOf(',', secondComma + 1);
    int fourthComma = input.indexOf(',', thirdComma + 1);

    int fifthComma = input.indexOf(',', fourthComma + 1);

    int joyX = input.substring(firstComma + 1, secondComma).toInt();
    int joyY = input.substring(secondComma + 1, thirdComma).toInt();
    int yasakMin = input.substring(thirdComma + 1, fourthComma).toInt();
    int yasakMax = input.substring(fourthComma + 1, fifthComma).toInt();
    char trigger = input.substring(fifthComma + 1).charAt(0);

    gerekliFonksiyonlar(joyX, joyY, yasakMin, yasakMax, trigger);
  
}





void gerekliFonksiyonlar(int joyX, int joyY,int yasakAlanMin,int yasakAlanMax,char received) {
  static long currentX = 0;
  static long currentY = 0;
  static unsigned long lastMoveTime = 0;
  const int deadZone = 50;
  const long maxStep = 40080;
  const long minStep = -40080;
  const unsigned long moveInterval = 150;

  int deviationX = joyX - 512;
  int deviationY = joyY - 512;

  unsigned long now = millis();
  if (now - lastMoveTime > moveInterval) {
    if (abs(deviationX) > deadZone) {
      long newX = currentX + ((deviationX > 0) ? 100 : -100);
      if (newX >= minStep && newX <= maxStep) {
        currentX = newX;
      }
    }

    if (abs(deviationY) > deadZone) {
      long newY = currentY + ((deviationY > 0) ? 55 : -55);
      if (newY >= minStep && newY <= maxStep) {
        currentY = newY;
      }
    }

    stepperX.moveTo(currentX);
    stepperY.moveTo(currentY);

    lastMoveTime = now;
  }

  stepperX.run();
  stepperY.run();

  float currentXDegrees = stepperX.currentPosition() / stepsPerDegreeX;



    bool yasakAlan = (currentXDegrees >= yasakAlanMin && currentXDegrees <= yasakAlanMax);

    if (received == '1' && !yasakAlan) {
      RS485_SERIAL.println("ATIŞ başlıyor");
      if (!stepperZActive) {
    stepperZ.move(300);
    stepperZActive = true;}

      else if (received == '1' && yasakAlan) {
      RS485_SERIAL.println("Atış yapılamaz, yasaklı bölgede!");



    }
  

}}





void moveToPositionManually(int targetX, int targetY) {
    if (targetX != stepperX.currentPosition()) {
     stepperX.move(targetX  * 100); // 2000 adım ilerle
    
    }

  if (targetY != stepperY.currentPosition()) {
     stepperY.move(targetY  * 100); // 2000 adım ilerle
    
  }
  

 

}