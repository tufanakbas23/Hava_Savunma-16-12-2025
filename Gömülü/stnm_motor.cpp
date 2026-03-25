#include "inc\stnm_motor.h"
#include <Arduino.h>
#include <AccelStepper.h>




/* ================= MOTOR ================= */

AccelStepper panMotor(AccelStepper::DRIVER, PAN_STEP_PIN, PAN_DIR_PIN);
AccelStepper tiltMotor(AccelStepper::DRIVER, TILT_STEP_PIN, TILT_DIR_PIN);


/* ================= KALİBRASYON ================= */

// 1/16 microstep
// pan gear ratio 3:1
static const float pulsesPerDegreePan = (3200.0 * 3.0) / 360.0;

// tilt gear ratio 1:1
static const float pulsesPerDegreeTilt = (3200.0 * 1.0) / 360.0;


/* ================= SPEED ================= */

static const float TARGET_SPEED_DEG_PER_SEC = 120.0;

static const float MAX_SPEED_PAN  = TARGET_SPEED_DEG_PER_SEC * pulsesPerDegreePan;
static const float MAX_SPEED_TILT = TARGET_SPEED_DEG_PER_SEC * pulsesPerDegreeTilt;

static const float ACCEL_PAN  = 3000.0;
static const float ACCEL_TILT = 1000.0;


/* ================= LIMITS ================= */

static const float PAN_MIN = -135.0;
static const float PAN_MAX =  135.0;

static const float TILT_MIN = -30.0;
static const float TILT_MAX =  30.0;


/* ================= STATE ================= */

static float currentPanDeg = 0;
static float currentTiltDeg = 0;

static float targetPanDeg = 0;
static float targetTiltDeg = 0;


/* ================= INIT ================= */

void Motor_Init()
{
    pinMode(PAN_ENABLE_PIN, OUTPUT);
    pinMode(TILT_ENABLE_PIN, OUTPUT);

    digitalWrite(PAN_ENABLE_PIN, LOW);
    digitalWrite(TILT_ENABLE_PIN, LOW);

    panMotor.setMaxSpeed(MAX_SPEED_PAN);
    panMotor.setAcceleration(ACCEL_PAN);
    panMotor.setCurrentPosition(0);

    tiltMotor.setMaxSpeed(MAX_SPEED_TILT);
    tiltMotor.setAcceleration(ACCEL_TILT);
    tiltMotor.setCurrentPosition(0);
}


/* ================= SET TARGET ================= */

void Motor_SetPan(int16_t angle)
{
    targetPanDeg = constrain(angle, PAN_MIN, PAN_MAX);
}

void Motor_SetTilt(int16_t angle)
{
    targetTiltDeg = constrain(angle, TILT_MIN, TILT_MAX);
}


/* ================= GET ================= */

int16_t Motor_GetPan()
{
    return (int16_t)currentPanDeg;
}

int16_t Motor_GetTilt()
{
    return (int16_t)currentTiltDeg;
}


/* ================= UPDATE ================= */

void Motor_Update()
{
    long panSteps  = targetPanDeg  * pulsesPerDegreePan;
    long tiltSteps = targetTiltDeg * pulsesPerDegreeTilt;

    panMotor.moveTo(panSteps);
    tiltMotor.moveTo(tiltSteps);

    panMotor.run();
    tiltMotor.run();

    currentPanDeg  = panMotor.currentPosition() / pulsesPerDegreePan;
    currentTiltDeg = tiltMotor.currentPosition() / pulsesPerDegreeTilt;
}