// Arduino library
#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <string.h>
#include <TinyGPSPlus.h>

// Personal library
#include "IMU_Config.h"    // Configure the MPU6050
#include "Serial_Config.h" // Configure the serial communication
#include "Motor_Config.h"  // Configure the motor
#include "PID_Config.h"    // Configure the PID
#include "GPS_Config.h"    // Configure GPS

// ================================================================
// Variable declaration
// ================================================================
#define RXD2 16
#define TXD2 17
// Most of the variables are declared in the personal library
// Define the incoming data, RECEIVED into this board

// Insert the MAC address of the other board
uint8_t controllerAddress[] = {0x48, 0xE7, 0x29, 0x9F, 0xDE, 0x7C};

typedef struct struct_msg_Receive
{
    int Receive_PotValue;
    int Receive_JoyVrx;
    int Receive_JoyVry;
    bool Receive_Button1State;
    bool Receive_Button2State;
} struct_msg_Receive;

typedef struct struct_msg_Sent
{
    double Sent_Longitude;
    double Sent_Latitude;
    double Sent_Altitude;
    double Sent_AngleX;
    double Sent_AngleY;
    double Sent_AngleZ;
    double Sent_GyroX;
    double Sent_GyroY;
    double Sent_GyroZ;
    double Sent_PidOutputX;
    double Sent_PidOutputY;
    double Sent_PidOutputZ;

    double Sent_kp_anglex;
    double Sent_ki_anglex;
    double Sent_kd_anglex;

    double Sent_kp_angley;
    double Sent_ki_angley;
    double Sent_kd_angley;

    double Sent_kp_anglez;
    double Sent_ki_anglez;
    double Sent_kd_anglez;

    double Sent_kp_gyrox;
    double Sent_ki_gyrox;
    double Sent_kd_gyrox;

    double Sent_kp_gyroy;
    double Sent_ki_gyroy;
    double Sent_kd_gyroy;

    double Sent_kp_gyroz;
    double Sent_ki_gyroz;
    double Sent_kd_gyroz;

} struct_msg_Sent;

// VARIABLES TO SEND
FLOATUNION_t send_anglex;
FLOATUNION_t send_angley;
FLOATUNION_t send_anglez;
FLOATUNION_t send_pid_output_x;
FLOATUNION_t send_pid_output_y;
FLOATUNION_t send_pid_output_z;
FLOATUNION_t send_anglex_setpoint;
FLOATUNION_t send_angley_setpoint;
FLOATUNION_t send_anglez_setpoint;
FLOATUNION_t send_gyrox_setpoint;
FLOATUNION_t send_gyroy_setpoint;
FLOATUNION_t send_gyroz_setpoint;

// Declare the structure
struct_msg_Receive Receive_Data;
struct_msg_Sent Sent_Data;

esp_now_peer_info_t peerInfo;

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
void Init_Serial(); // Function to init the serial monitor
void Init_ESC();
void Init_ESPNOW();
void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
float floatMap(float, float, float, float, float);
void SerialDataPrint(); // Data from the microcontroller to the PC
void SerialDataWrite(); // Data from the PC to the microcontroller
void Get_GPSData();
void DisplayInfo();

// ================================================================
// Setup function
// ================================================================
void setup()
{
    // Initialization
    Init_Serial();
    Init_MPU();
    Init_PID();
    Init_ESC();
    Init_ESPNOW();
}
// ================================================================
// Loop function
// ================================================================
void loop()
{
    // Get data from controller
    CtrlPWM = Receive_Data.Receive_PotValue;
    JoyVrx = Receive_Data.Receive_JoyVrx;
    JoyVry = Receive_Data.Receive_JoyVry;
    Button1State = Receive_Data.Receive_Button1State;
    Button2State = Receive_Data.Receive_Button2State;

    Get_MPUangle();  // Get the angle from the IMU sensor
    Get_GPSData();
    Get_accelgyro(); // Get rate from IMU sensor
    Compute_PID();   // Compute the PID output
    Run_Motor();     // Send the PID output to the motor

    // // Debugging
    // Serial.println(CtrlPWM);
    // Serial.println(motor_cmd_x);
    // Serial.println(motor_cmd_y);
    // Serial.println(motor_cmd_z);

    // Prepare data for sending back to Controller
    // Sent_Data.Sent_Longitude = Longitude;
    // Sent_Data.Sent_Latitude = Latitude;
    // Sent_Data.Sent_Altitude = Altitude;
    Sent_Data.Sent_AngleX = anglex;
    Sent_Data.Sent_AngleY = angley;
    Sent_Data.Sent_AngleZ = anglez;
    Sent_Data.Sent_GyroX = gyrox;
    Sent_Data.Sent_GyroY = gyroy;
    Sent_Data.Sent_GyroZ = gyroz;

    // if (micros() - time_prev_serial >= 20000)
    // {
    //     time_prev_serial = micros();
    //     SerialDataWrite();
    // }

    // Data sent over espnow
    esp_now_send(controllerAddress, (uint8_t *)&Sent_Data, sizeof(Sent_Data));
}

// ================================================================
// Function Definition
// ================================================================
void SerialDataPrint()
{
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
    static String modification = "";
    static String value = "";
    static String option = "";
    while (Serial.available())
    {
        // Read from serial monitor
        char inChar = (char)Serial.read();
        modification += inChar;
        option = modification;
        value = modification;

        // Option extraction
        option.remove(3, modification.length() - 3);
        // Value extractionpay1
        value.remove(0, 3);

        // P adjustment for angle
        if (option.equals("pax"))
            kp_anglex = value.toFloat();
        if (option.equals("pay"))
            kp_angley = value.toFloat();
        if (option.equals("paz"))
            kp_anglez = value.toFloat();

        // I adjustment for angle
        if (option.equals("iax"))
            ki_anglex = value.toFloat();
        if (option.equals("iay"))
            ki_angley = value.toFloat();
        if (option.equals("iaz"))
            ki_anglez = value.toFloat();

        // D adjustment for angle
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

        // Setpoint adjustment for angle
        if (option.equals("axs"))
            anglex_setpoint = value.toFloat();
        if (option.equals("ays"))
            angley_setpoint = value.toFloat();
        if (option.equals("azs"))
            anglez_setpoint = value.toFloat();

        // Setpoint adjustment for gyro
        if (option.equals("gxs"))
            gyrox_setpoint = value.toFloat();
        if (option.equals("gys"))
            gyroy_setpoint = value.toFloat();
        if (option.equals("gzs"))
            gyroz_setpoint = value.toFloat();
    }
    // Clear input modification
    modification = "";
    value = "";
    option = "";
}

// ******************************************
void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    // debugging serial
    // Serial.print(micros() / 1000);
    // Serial.println("\tData received!");
    // You must copy the incoming data to the local variables
    memcpy(&Receive_Data, incomingData, sizeof(Receive_Data));
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // There is nothing to do when sending data, this is just for debugging
    // Serial.print(micros() / 1000);
    // Serial.println("\tData sent!");
    // Serial.print("\r\nLast Packet Send Status:\t");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    memcpy(peerInfo.peer_addr, controllerAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }
}