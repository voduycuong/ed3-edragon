#include <Arduino.h> // Arduino library
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

#include "IMU_Config.h"    // Personal library to configure the MPU6050
#include "Serial_Config.h" // Personal library to configure the serial communication
#include "Motor_Config.h"  // Personal library to configure the motor
#include "PID_Config.h"    // Personal library to configure the PID

// ================================================================
// Variable declaration
// ================================================================
// Most of the variables are declared in the personal library
// Define the incoming data, RECEIVED into this board

typedef struct struct_msg_Receive
{
    int Receive_PotValue;
    int Receive_JoyVrx;
    int Receive_JoyVry;
    bool Receive_Button1State;
    bool Receive_Button2State;
} struct_msg_Receive;

// Declare the structure
struct_msg_Receive Receive_Data;

// Serial
unsigned long time_prev_serial = 0;

int CtrlPWM = 0;           // Control Signal. Varies between [0 - 180]
int JoyVrx = 0;            // X value of joy con position [0 - 4095]
int JoyVry = 0;            // Y value of joy con position [0 - 4095]
bool Button1State = false; // 0 - unpressed. 1 - pressed
bool Button2State = false; // 0 - unpressed. 1 - pressed

// ================================================================
// Function Declaration
// ================================================================
// These function are kept in the main.cpp because it is easier to modify
void SerialDataPrint(); // Data from the microcontroller to the PC
void SerialDataWrite(); // Data from the PC to the microcontroller
void Init_Serial();     // Function to init the serial monitor
void Init_ESC();
void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
float floatMap(float, float, float, float, float);
void espnow_initialize();

// ================================================================
// Setup function
// ================================================================
void setup()
{
    espnow_initialize();
    Init_Serial(); // Initialize Serial Communication
    Init_MPU();    // Initialize MPU
    Init_PID();    // Initialize PID
    Init_ESC();    // Initializa ESC
}
// ================================================================
// Loop function
// ================================================================
void loop()
{
    CtrlPWM = Receive_Data.Receive_PotValue;
    JoyVrx = Receive_Data.Receive_JoyVrx;
    JoyVry = Receive_Data.Receive_JoyVry;
    Button1State = Receive_Data.Receive_Button1State;
    Button2State = Receive_Data.Receive_Button2State;

    Get_MPUangle(); // Get the angle from the IMU sensor
    // Compute_PID();     // Compute the PID output (motor_cmd)
    // Run_Motor();       // Send the PID output (motor_cmd) to the motor
    SerialDataPrint(); // Print the data on the serial monitor for debugging
    SerialDataWrite(); // User data to tune the PID parameters

    if (micros() - time_prev_serial >= 20000)
    {
        time_prev_serial = micros();
        SerialDataWrite();
    }
}

// ================================================================
// Function Definition
// ================================================================
void SerialDataPrint()
{
    if (micros() - time_prev >= 50000)
    {
        time_prev = micros();
        Serial.print(millis());
        Serial.print("\t");
        Serial.print(anglex);
        Serial.print("\t");
        Serial.print(angley);
        Serial.print("\t");
        Serial.print(anglez);
        Serial.print("\t");
        Serial.print(kp);
        Serial.print("\t");
        Serial.print(ki);
        Serial.print("\t");
        Serial.print(kd);
        Serial.print("\t");
        Serial.print(anglex_setpoint);
        Serial.print(angley_setpoint);
        Serial.print(anglez_setpoint);

        Serial.println();
    }
}

// ================================================================
// Function to tune the PID parameters. For example:
// To change the P value to 10, type p10
// To change the I value to -5, type i-5
// To change the D value to 2.4, type d2.4
// To change the setpoint to 3, type s3

void SerialDataWrite()
{
    static String received_chars;
    while (Serial.available())
    {
        char inChar = (char)Serial.read();
        received_chars += inChar;
        if (inChar == '\n')
        {
            switch (received_chars[0])
            {
            case 'p':
                received_chars.remove(0, 1);
                kp = received_chars.toFloat();
                break;
            case 'i':
                received_chars.remove(0, 1);
                ki = received_chars.toFloat();
                break;
            case 'd':
                received_chars.remove(0, 1);
                kd = received_chars.toFloat();
                break;
            case 's':
                received_chars.remove(0, 1);
                anglex_setpoint = received_chars.toFloat();
            default:
                break;
            }
            received_chars = "";
        }
    }
    Serial.print(micros() / 1000);
    Serial.print("\tP: ");
    Serial.print(CtrlPWM);
    Serial.print("\tJX: ");
    Serial.print(JoyVrx);
    Serial.print("\tJY: ");
    Serial.print(JoyVry);
    Serial.print("\tB1: ");
    Serial.print(Button1State);
    Serial.print("\tB2: ");
    Serial.print(Button2State);
    Serial.println();
}

// ******************************************
void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    // debugging serial
    Serial.print(micros() / 1000);
    Serial.println("\tData received!");
    // You must copy the incoming data to the local variables
    memcpy(&Receive_Data, incomingData, sizeof(Receive_Data));
}

// ******************************************
float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ******************************************
void espnow_initialize()
{
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataReceive);
}