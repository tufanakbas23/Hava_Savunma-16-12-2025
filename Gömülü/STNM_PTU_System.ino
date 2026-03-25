#include "inc\stnm_protocol.h"
#include "inc\stnm_motor.h"
#include "inc\stnm_lidar.h"
#include <Arduino.h>

#define UART_BAUD 115200

/* ================= SYSTEM STATE ================= */

uint8_t seq = 0;
uint8_t system_state = STNM_STATE_IDLE;

/* ================= PACKETS ================= */

StnmCommandPacket_t commandPacket;
StnmTelemetryPacket_t telemetryPacket;
StnmAckPacket_t ackPacket;

/* ===================== SETUP ===================== */

void setup()
{
    Serial.begin(UART_BAUD);

    STNM_Protocol_Init();
    Motor_Init();
    Lidar_Init();

    commandPacket.command_id = STNM_CMD_NONE;
}

/* ===================== LOOP ===================== */

void loop()
{
    // ---- SERIAL MONITOR TEST MODE ----
    if (Serial.available())
    {
        char cmd = Serial.read();

        if(cmd == 'p')   // pan +
        {
            Motor_SetPan(Motor_GetPan() + 1000);
        }

        if(cmd == 'o')   // pan -
        {
            Motor_SetPan(Motor_GetPan() - 10);
        }

        if(cmd == 't')   // tilt +
        {
            Motor_SetTilt(Motor_GetTilt() + 5);
        }

        if(cmd == 'g')   // tilt -
        {
            Motor_SetTilt(Motor_GetTilt() - 5);
        }

        if(cmd == 'r')   // reset
        {
            resetSystem();
        }
    }

    // protocol
    STNM_CheckForCommand(&commandPacket);

    if (commandPacket.command_id != STNM_CMD_NONE)
        processCommand();

    Motor_Update();

    sendTelemetry();
   
}

/* ===================== PROCESS COMMAND ===================== */

void processCommand()
{
    switch (commandPacket.command_id)
    {

        case STNM_CMD_SET_MODE:

            system_state = commandPacket.mode;

            sendAck(STNM_CMD_SET_MODE,1);

        break;


        case STNM_CMD_SET_PANTILT:

            Motor_SetPan(commandPacket.pan);
            Motor_SetTilt(commandPacket.tilt);

            sendAck(STNM_CMD_SET_PANTILT,1);

        break;


        case STNM_CMD_FIRE:

            if(commandPacket.fire)
            {
                fireWeapon();
                sendAck(STNM_CMD_FIRE,1);
            }

        break;


        case STNM_CMD_RESET:

            resetSystem();
            sendAck(STNM_CMD_RESET,1);

        break;


        default:
        break;
    }

    // Prevent reprocessing
    commandPacket.command_id = STNM_CMD_NONE;
}

/* ===================== SEND ACK ===================== */

void sendAck(uint8_t cmd,uint8_t status)
{
    STNM_BuildAck(&ackPacket,seq++,cmd,status);

    Serial.write((uint8_t*)&ackPacket,sizeof(StnmAckPacket_t));
}

/* ===================== TELEMETRY ===================== */

void sendTelemetry()
{
    static unsigned long lastSend = 0;

    if(millis() - lastSend < 100)
        return;

    lastSend = millis();

    int16_t pan  = Motor_GetPan();
    int16_t tilt = Motor_GetTilt();
    uint16_t lidar = Lidar_Read();

    // Debug: print readable numbers
    Serial.print("Pan: "); Serial.print(pan);
    Serial.print(" Tilt: "); Serial.print(tilt);
    Serial.print(" Lidar: "); Serial.print(lidar);
    Serial.print(" State: "); Serial.println(system_state);

    // Optional: still send binary for protocol
    STNM_BuildTelemetry(&telemetryPacket, seq++, pan, tilt, lidar, system_state);
    Serial.write((uint8_t*)&telemetryPacket, sizeof(StnmTelemetryPacket_t));
}

/* ===================== FIRE CONTROL ===================== */

void fireWeapon()
{
    system_state = STNM_STATE_FIRING;

    // future: trigger relay / solenoid / laser
}

/* ===================== RESET SYSTEM ===================== */

void resetSystem()
{
    system_state = STNM_STATE_IDLE;

    Motor_SetPan(0);
    Motor_SetTilt(0);
}