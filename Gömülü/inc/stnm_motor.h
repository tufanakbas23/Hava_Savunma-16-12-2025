#ifndef STNM_MOTOR_H
#define STNM_MOTOR_H

#include <stdint.h>

/* ================= PINLER ================= */


#define PAN_STEP_PIN   5
#define PAN_DIR_PIN    6
#define PAN_ENABLE_PIN 7

#define TILT_STEP_PIN   2
#define TILT_DIR_PIN    3
#define TILT_ENABLE_PIN 4


/* ================= Functions  ================= */

 

void Motor_Init();
void Motor_Update();

void Motor_SetPan(int16_t angle);
void Motor_SetTilt(int16_t angle);

int16_t Motor_GetPan();
int16_t Motor_GetTilt();

#endif