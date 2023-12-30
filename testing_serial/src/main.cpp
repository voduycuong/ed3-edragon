#include <Arduino.h>

double kp_anglex = 5.00;

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    static String modification;
    static String option;
    static String value;

    while (Serial.available())
    {

        // Read from serial monitor
        char inChar = (char)Serial.read();
        modification += inChar;

        option = modification;
        value = modification;

        // Option extraction
        option.remove(3, modification.length() - 3);
        // Value extraction
        value.remove(0, 3);

        // P adjustment for accelerate
        if (option.equals("pax"))
            kp_anglex = value.toFloat();
    }

    Serial.println();
    Serial.print("Mod: ");
    Serial.print(modification);
    Serial.print("\t\tValue: ");
    Serial.print(value);
    Serial.print("\t\tOption: ");
    Serial.print(option);

    Serial.println();
    Serial.print("kp_anglex: ");
    Serial.print(kp_anglex);

    delay(500);

    modification = "";
    value = "";
    option = "";
}