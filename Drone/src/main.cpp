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
// GPS UART
#define RXD2 16
#define TXD2 17

double anglex_setpoint;
double angley_setpoint;
double anglez_setpoint;

// MAC address of Controller
uint8_t controllerAddress[] = {0x48, 0xE7, 0x29, 0x9F, 0xDE, 0x7C};

// Variable for espnow communication
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
void Init_Serial(); // Function to init the serial monitor
void Init_ESC();
void Init_ESPNOW();
void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
float floatMap(float, float, float, float, float);
void Get_GPSData();
void DisplayInfo();

// Define the incoming data
typedef struct struct_msg_Receive
{
    //
    int Receive_PotValue;
    int Receive_JoyVrx;
    int Receive_JoyVry;
    bool Receive_Button1State;
    bool Receive_Button2State;

    double Receive_kp_anglex;
    double Receive_ki_anglex;
    double Receive_kd_anglex;
    double Receive_kp_angley;
    double Receive_ki_angley;
    double Receive_kd_angley;
    double Receive_kp_anglez;
    double Receive_ki_anglez;
    double Receive_kd_anglez;

    double Receive_kp_gyrox;
    double Receive_ki_gyrox;
    double Receive_kd_gyrox;
    double Receive_kp_gyroy;
    double Receive_ki_gyroy;
    double Receive_kd_gyroy;
    double Receive_kp_gyroz;
    double Receive_ki_gyroz;
    double Receive_kd_gyroz;

    double Receive_anglex_setpoint;
    double Receive_angley_setpoint;
    double Receive_anglez_setpoint;
} struct_msg_Receive;

// Define the outgoing data
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

} struct_msg_Sent;

// Declare the structure
struct_msg_Receive Receive_Data;
struct_msg_Sent Sent_Data;

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
    // Receving data --------------------------------------------------------
    CtrlPWM = Receive_Data.Receive_PotValue;
    JoyVrx = Receive_Data.Receive_JoyVrx;
    JoyVry = Receive_Data.Receive_JoyVry;
    Button1State = Receive_Data.Receive_Button1State;
    Button2State = Receive_Data.Receive_Button2State;

    kp_anglex = Receive_Data.Receive_kp_anglex;
    ki_anglex = Receive_Data.Receive_ki_anglex;
    kd_anglex = Receive_Data.Receive_kd_anglex;
    kp_angley = Receive_Data.Receive_kp_angley;
    ki_angley = Receive_Data.Receive_ki_angley;
    kd_angley = Receive_Data.Receive_kd_angley;
    kp_anglez = Receive_Data.Receive_kp_anglez;
    ki_anglez = Receive_Data.Receive_ki_anglez;
    kd_anglez = Receive_Data.Receive_kd_anglez;

    kp_gyrox = Receive_Data.Receive_kp_gyrox;
    ki_gyrox = Receive_Data.Receive_ki_gyrox;
    kd_gyrox = Receive_Data.Receive_kd_gyrox;
    kp_gyroy = Receive_Data.Receive_kp_gyroy;
    ki_gyroy = Receive_Data.Receive_ki_gyroy;
    kd_gyroy = Receive_Data.Receive_kd_gyroy;
    kp_gyroz = Receive_Data.Receive_kp_gyroz;
    ki_gyroz = Receive_Data.Receive_ki_gyroz;
    kd_gyroz = Receive_Data.Receive_kd_gyroz;

    anglex_setpoint = Receive_Data.Receive_anglex_setpoint;
    angley_setpoint = Receive_Data.Receive_angley_setpoint;
    anglez_setpoint = Receive_Data.Receive_anglez_setpoint;
    // End of receving data --------------------------------------------------------

    Get_MPUangle();  // Get the angle from the IMU sensor
    Get_accelgyro(); // Get rate from IMU sensor
    Get_GPSData();   // Get data from GPS
    Compute_PID();   // Compute the PID output
    Run_Motor();     // Send the PID output to the motor

    // Sending data ---------------------------------------------------------------
    Sent_Data.Sent_Longitude = 0;
    Sent_Data.Sent_Latitude = 0;
    Sent_Data.Sent_Altitude = 0;

    Sent_Data.Sent_AngleX = anglex;
    Sent_Data.Sent_AngleY = angley;
    Sent_Data.Sent_AngleZ = anglez;
    Sent_Data.Sent_GyroX = gyrox;
    Sent_Data.Sent_GyroY = gyroy;
    Sent_Data.Sent_GyroZ = gyroz;
    // End of sending data ---------------------------------------------------------------

    // Data sent over espnow
    esp_now_send(controllerAddress, (uint8_t *)&Sent_Data, sizeof(Sent_Data));

    // // Debugging
    // Serial.print(anglex_setpoint);
    // Serial.print("\t");
    // Serial.print(angley_setpoint);
    // Serial.print("\t");
    // Serial.print(anglez_setpoint);
    // Serial.println();
}

// ================================================================
// Function Definition
// ******************************************
void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    // Serial.print(micros() / 1000);
    // Serial.println("\tData received!");
    // You must copy the incoming data to the local variables
    memcpy(&Receive_Data, incomingData, sizeof(Receive_Data));
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
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