#include "inc\stnm_protocol.h"
#include "inc\stnm_crc.h"
#include <Arduino.h>

void STNM_Protocol_Init()
{
}

void STNM_CheckForCommand(StnmCommandPacket_t *packet)
{
    if(Serial.available() >= sizeof(StnmCommandPacket_t))
    {
        Serial.readBytes((uint8_t*)packet,sizeof(StnmCommandPacket_t));

        if(!STNM_VerifyPacket((uint8_t*)packet,sizeof(StnmCommandPacket_t)))
            packet->command_id = STNM_CMD_NONE;
    }
}

void STNM_BuildTelemetry(StnmTelemetryPacket_t *packet,
                         uint8_t seq,
                         int16_t pan,
                         int16_t tilt,
                         uint16_t lidar,
                         uint8_t state)
{
    packet->sop = STNM_UART_SOP;
    packet->packet_type = STNM_PACKET_TELEMETRY;
    packet->seq = seq;

    packet->pan_current = pan;
    packet->tilt_current = tilt;
    packet->lidar_distance = lidar;
    packet->system_state = state;

    uint16_t crc = STNM_CalcCRC((uint8_t*)packet,sizeof(StnmTelemetryPacket_t)-2);
    packet->crc = crc;
}

void STNM_BuildAck(StnmAckPacket_t *ack,
                   uint8_t seq,
                   uint8_t cmd,
                   uint8_t status)
{
    ack->sop = STNM_UART_SOP;
    ack->packet_type = STNM_PACKET_ACK;
    ack->seq = seq;

    ack->command_id = cmd;
    ack->status = status;

    uint16_t crc = STNM_CalcCRC((uint8_t*)ack,sizeof(StnmAckPacket_t)-2);
    ack->crc = crc;
}

uint8_t STNM_VerifyPacket(uint8_t *packet,uint16_t length)
{
    uint16_t received_crc =
        (packet[length-2] << 8) | packet[length-1];

    uint16_t calculated_crc =
        STNM_CalcCRC(packet,length-2);

    return received_crc == calculated_crc;
}