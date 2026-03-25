#ifndef STNM_PROTOCOL_TYPES_H
#define STNM_PROTOCOL_TYPES_H

#include <stdint.h>

#define STNM_UART_SOP 0xAA

#pragma pack(push,1)

typedef enum
{
    STNM_PACKET_CMD = 1,
    STNM_PACKET_TELEMETRY,
    STNM_PACKET_ACK

} StnmPacketType_t;

typedef enum
{
    STNM_MODE_RESET = 0,
    STNM_MODE_MANUAL,
    STNM_MODE_AUTONOMOUS

} StnmMode_t;

typedef enum
{
    STNM_CMD_NONE = 0,
    STNM_CMD_SET_MODE,
    STNM_CMD_SET_PANTILT,
    STNM_CMD_FIRE,
    STNM_CMD_RESET

} StnmCommandID_t;

typedef enum
{
    STNM_STATE_IDLE = 0,
    STNM_STATE_TRACKING,
    STNM_STATE_FIRING,
    STNM_STATE_ERROR

} StnmSystemState_t;

typedef struct
{
    uint8_t sop;
    uint8_t packet_type;
    uint8_t seq;

    uint8_t command_id;
    uint8_t mode;

    int16_t pan;
    int16_t tilt;

    uint8_t fire;

    uint16_t crc;

} StnmCommandPacket_t;

typedef struct
{
    uint8_t sop;
    uint8_t packet_type;
    uint8_t seq;

    int16_t pan_current;
    int16_t tilt_current;

    uint16_t lidar_distance;

    uint8_t system_state;

    uint16_t crc;

} StnmTelemetryPacket_t;

typedef struct
{
    uint8_t sop;
    uint8_t packet_type;
    uint8_t seq;

    uint8_t command_id;
    uint8_t status;

    uint16_t crc;

} StnmAckPacket_t;

#pragma pack(pop)

#endif