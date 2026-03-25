#include "inc\stnm_lidar.h"
#include <Arduino.h>

static uint16_t lastDistance = 0;

void Lidar_Init() {
    Serial3.begin(115200);
}

uint16_t Lidar_Read() {
    const int FRAME_SIZE = 9;
    uint8_t buf[FRAME_SIZE];

    while (Serial3.available() >= FRAME_SIZE) {
        // Peek to find header
        if (Serial3.peek() == 0x59) {
            Serial3.readBytes(buf, FRAME_SIZE);

            if (buf[0] == 0x59 && buf[1]    == 0x59) {
                // Compute checksum
                uint8_t sum = 0;
                for (int i = 0; i < 8; i++) sum += buf[i];

                if (sum == buf[8]) {
                    lastDistance = buf[2] | (buf[3] << 8);
                    return lastDistance;
                }
            }
        } else {
            Serial3.read(); // discard misaligned byte
        }
    }

    return lastDistance; // keep last valid
}