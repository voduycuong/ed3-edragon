#include <Arduino.h>

#define RXD2 16
#define TXD2 17

unsigned long time_prev = 0;

typedef union {
    float number;
    uint8_t bytes[4];
} FLOATUNION_t;

void Init_Serial()
{
    Serial.begin(115200);
    while (!Serial)
    {
        Wire.begin();
        Wire.beginTransmission(0x68);
        Wire.write(0x6B);
        Wire.write(0x00);
        Wire.endTransmission(true);
        
    };
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
}