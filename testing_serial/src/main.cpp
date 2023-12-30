#include <Arduino.h>

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

        // Enter char received
        if (inChar == '\n')
        {
            // Option extraction
            option = modification;
            option.remove(3, modification.length() - 3);
        }
    }

    Serial.println("Mod: ");
    Serial.println(modification);
    Serial.println("Value: ");
    Serial.println(value);
    Serial.println("Option: ");
    Serial.println(option);
    delay(1000);
}