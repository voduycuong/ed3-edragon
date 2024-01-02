#include <Arduino.h> // Arduino library
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <string.h>

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
void Init_ESPNOW();

// ================================================================
// Setup function
// ================================================================
void setup()
{
    Init_ESPNOW();
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
    Get_accelgyro();
    Compute_PID();  // Compute the PID output (motor_cmd)

    if (CtrlPWM > 20) // Set threshold for thrust
        Run_Motor();  // Send the PID output (motor_cmd) to the motor

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
// VARIABLES TO SEND
FLOATUNION_t send_anglex;
FLOATUNION_t send_angley;
FLOATUNION_t send_anglez;
FLOATUNION_t send_gyrox;
FLOATUNION_t send_gyroy;
FLOATUNION_t send_gyroz;
FLOATUNION_t send_pid_output_x;
FLOATUNION_t send_pid_output_y;
FLOATUNION_t send_pid_output_z;
FLOATUNION_t send_anglex_setpoint;
FLOATUNION_t send_angley_setpoint;
FLOATUNION_t send_anglez_setpoint;
FLOATUNION_t send_gyrox_setpoint;
FLOATUNION_t send_gyroy_setpoint;
FLOATUNION_t send_gyroz_setpoint;

void SerialDataPrint()
{
    send_anglex.number = anglex;
    send_angley.number = angley;
    send_anglez.number = anglez;
    send_gyrox.number = gyrox;
    send_gyroy.number = gyroy;
    send_gyroz.number = gyroz;
    send_pid_output_x.number = pid_output_x;
    send_pid_output_y.number = pid_output_y;
    send_pid_output_z.number = pid_output_z;
    send_anglex_setpoint.number = anglex_setpoint;
    send_angley_setpoint.number = angley_setpoint;
    send_anglez_setpoint.number = anglez_setpoint;
    send_gyrox_setpoint.number = gyrox_setpoint;
    send_gyroy_setpoint.number = gyroy_setpoint;
    send_gyroz_setpoint.number = gyroz_setpoint;

    if (micros() - time_prev >= 10000)
    {
        time_prev = micros();
        // Serial.print(millis());
        // Serial.print("\t");
        Serial.write('A');
        for(int i = 0; i < 4; i++) {
            Serial.write(send_anglex.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_angley.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_anglez.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_gyrox.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_gyroy.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_gyroz.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_pid_output_x.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_pid_output_y.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_pid_output_z.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_anglex_setpoint.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_angley_setpoint.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_anglez_setpoint.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_gyrox_setpoint.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_gyroy_setpoint.bytes[i]);
        }
        for(int i = 0; i < 4; i++) {
            Serial.write(send_gyroz_setpoint.bytes[i]);
        }
        // Serial.print("\t");
        // Serial.print(angley, 3);
        // Serial.print("\t");
        // Serial.print(anglez, 3);
        // Serial.print("\t");
        // Serial.print(kp);
        // Serial.print("\t");
        // Serial.print(ki);
        // Serial.print("\t");
        // Serial.print(kd);
        // Serial.print("\t");
        // Serial.print(pid_output_x, 3);
        // Serial.print("\t");
        // Serial.print(pid_output_y, 3);
        // Serial.print("\t");
        // Serial.print(pid_output_z, 3);
        // Serial.print("\t");

        Serial.print('\n');
    }
}

// ================================================================
// Function to tune the PID parameters. For example:
// To change the kp_anglex value to 10, type pax10
// To change the ki_anglex value to -5, type iax-5
// To change the kd_anglex value to 2.4, type dax2.4

// To change the kp_angley value to 10, type pay10
// To change the ki_angley value to -5, type iay-5
// To change the kd_angley value to 2.4, type day2.4

// To change the kp_anglez value to 10, type paz10
// To change the ki_anglez value to -5, type iaz-5
// To change the kd_anglez value to 2.4, type daz2.4

// To change the kp_gyrox value to 10, type pgx10
// To change the ki_gyrox value to -5, type igx-5
// To change the kd_gyrox value to 2.4, type dgx2.4

// To change the kp_gyroy value to 10, type pgy10
// To change the ki_gyroy value to -5, type igy-5
// To change the kd_gyroy value to 2.4, type dgy2.4

// To change the kp_gyroz value to 10, type pgz10
// To change the ki_gyroz value to -5, type igz-5
// To change the kd_gyroz value to 2.4, type dgz2.4

// To change the anglex_setpoint to 3, type axs3
// To change the angley_setpoint to 3, type ays3
// To change the anglez_setpoint to 3, type azs3

// To change the gyrox_setpoint to 3, type gxs3
// To change the gyroy_setpoint to 3, type gys3
// To change the gyroz_setpoint to 3, type gzs3

void SerialDataWrite()
{
    static String modification;
    static String value;
    static String option;

    while (Serial.available())
    {
        // Read from serial monitor
        char inChar = (char)Serial.read();
        modification += inChar;

        // Option extraction
        option = modification;
        option.remove(3, modification.length() - 3);

        // Value extraction
        value = modification;
        value.remove(0, 3);

        // Enter char received
        if (inChar == '\n')
        {
            // P adjustment for accelerate
            if (option.equals("pax"))
                kp_anglex = value.toFloat();
            if (option.equals("pay"))
                kp_angley = value.toFloat();
            if (option.equals("paz"))
                kp_anglez = value.toFloat();

            // I adjustment for accelerate
            if (option.equals("iax"))
                ki_anglex = value.toFloat();
            if (option.equals("iay"))
                ki_angley = value.toFloat();
            if (option.equals("iaz"))
                ki_anglez = value.toFloat();

            // D adjustment for accelerate
            if (option.equals("dax"))
                kd_anglex = value.toFloat();
            if (option.equals("day"))
                kd_angley = value.toFloat();
            if (option.equals("daz"))
                kd_anglez = value.toFloat();

            // P adjustment for gyro
            if (option.equals("pgx"))
                kp_gyrox = value.toFloat();
            if (option.equals("pgy"))
                kp_gyroy = value.toFloat();
            if (option.equals("pgz"))
                kp_gyroz = value.toFloat();

            // I adjustment for gyro
            if (option.equals("igx"))
                ki_gyrox = value.toFloat();
            if (option.equals("igy"))
                ki_gyroy = value.toFloat();
            if (option.equals("igz"))
                ki_gyroz = value.toFloat();

            // D adjustment for gyro
            if (option.equals("dgx"))
                kd_gyrox = value.toFloat();
            if (option.equals("dgy"))
                kd_gyroy = value.toFloat();
            if (option.equals("dgz"))
                kd_gyroz = value.toFloat();

            // Setpoint adjustment for accelerate
            if (option.equals("axs"))
                kd_gyrox = value.toFloat();
            if (option.equals("ays"))
                kd_gyroy = value.toFloat();
            if (option.equals("azs"))
                kd_gyroz = value.toFloat();

            // Setpoint adjustment for gyro
            if (option.equals("gxs"))
                kd_gyrox = value.toFloat();
            if (option.equals("gys"))
                kd_gyroy = value.toFloat();
            if (option.equals("gzs"))
                kd_gyroz = value.toFloat();

            // Clear input modification
            modification = "";
        }
    }
    // Serial.print(micros() / 1000);
    // Serial.print("\tP: ");
    // Serial.print(CtrlPWM);
    // Serial.print("\tJX: ");
    // Serial.print(JoyVrx);
    // Serial.print("\tJY: ");
    // Serial.print(JoyVry);
    // Serial.print("\tB1: ");
    // Serial.print(Button1State);
    // Serial.print("\tB2: ");
    // Serial.print(Button2State);
    // Serial.println();
}

// ******************************************
void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    // debugging serial
    Serial.print(micros() / 1000);
    // Serial.println("\tData received!");
    // You must copy the incoming data to the local variables
    memcpy(&Receive_Data, incomingData, sizeof(Receive_Data));
}

// ******************************************
float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ******************************************
void Init_ESPNOW()
{
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataReceive);
}