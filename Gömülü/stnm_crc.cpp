#include "inc/stnm_crc.h"

uint16_t STNM_CalcCRC(uint8_t *data,uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for(uint16_t i=0;i<length;i++)
    {
        crc ^= data[i];

        for(uint8_t j=0;j<8;j++)
        {
            if(crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }

    return crc;
} 