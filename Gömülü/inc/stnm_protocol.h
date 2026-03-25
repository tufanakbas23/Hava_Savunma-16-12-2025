#ifndef STNM_PROTOCOL_H
#define STNM_PROTOCOL_H

#include "stnm_protocol_types.h"

void STNM_Protocol_Init();

void STNM_CheckForCommand(StnmCommandPacket_t *packet);

void STNM_BuildTelemetry(StnmTelemetryPacket_t *packet,
                         uint8_t seq,
                         int16_t pan,
                         int16_t tilt,
                         uint16_t lidar,
                         uint8_t state);

void STNM_BuildAck(StnmAckPacket_t *ack,
                   uint8_t seq,
                   uint8_t cmd,
                   uint8_t status);

uint8_t STNM_VerifyPacket(uint8_t *packet,uint16_t length);

#endif