#include <Arduino.h>
unsigned long time_prev = 0;

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
}