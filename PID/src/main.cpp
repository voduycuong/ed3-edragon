#include <Arduino.h>       // Arduino library
#include "MyMPU.h"         // Personal library to configure the MPU6050
#include "MySerial.h"      // Personal library to configure the serial communication
#include "MyMotorConfig.h" // Personal library to configure the motor
// #include "MyPID.h"         // Personnal library to configure the PID
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
// ================================================================
// Variable declaration
// ================================================================
// Most of the variables are declared in the personal library
// Define the incoming data, RECEIVED into this board
#define MOTOR_LEFT1_PIN 32      // Pin 25 attached to ESC signal pin
#define MOTOR_LEFT2_PIN 33      // Pin 33 attached to ESC signal pin
#define MOTOR_RIGHT1_PIN 18      // Pin 18 attached to ESC signal pin
#define MOTOR_RIGHT2_PIN 19      // Pin 19 attached to ESC signal pin
#define MAX_SIGNAL 2000   // Parameter required for ESC definition
#define MIN_SIGNAL 1000   // Parameter required for the ESC definition

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

// Define the incoming data
typedef struct struct_msg_Sent
{
  int Sent_GPS_val; // replace with TinyGPSPlus object later
  float Sent_AngleX;
  float Sent_AngleY;
  float Sent_AngleZ;
  float Sent_GyroX;
  float Sent_GyroY;
  float Sent_GyroZ;
  float Sent_AccX;
  float Sent_AccY;
  float Sent_AccZ;
} struct_msg_Sent;

// Declare the structure
struct_msg_Receive Receive_Data;
struct_msg_Sent Sent_Data;

// Serial
unsigned long time_prev_serial = 0;

Servo ESC_Left1;                 // Define the ESC
Servo ESC_Left2;                 // Define the ESC
Servo ESC_Right1;                 // Define the ESC
Servo ESC_Right2;                 // Define the ESC
int CtrlPWM = 0;           // Control Signal. Varies between [0 - 180]
int JoyVrx = 0;            // X value of joy con position [0 - 4095]
int JoyVry = 0;            // Y value of joy con position [0 - 4095]
bool Button1State = false; // 0 - unpressed. 1 - pressed
bool Button2State = false; // 0 - unpressed. 1 - pressed

int GpsVal = 1;
int AngleX = 2;
int AngleY = 2;
int AngleZ = 2;
int GyroX = 3;
int GyroY = 3;
int GyroZ = 3;
int AccX = 4;
int AccY = 4;
int AccZ = 4;
// ================================================================
// Function Declaration
// ================================================================
// These function are kept in the main.cpp because it is easier to modify
void SerialDataPrint(); // Data from the microcontroller to the PC
void SerialDataWrite(); // Data from the PC to the microcontroller

void Init_Serial();      // Function to init the serial monitor
void WaitForKeyStroke(); // Function to interact with the serial monitor

void Init_ESC();

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
float floatMap(float, float, float, float, float);
void espnow_initialize();
// ================================================================
// Setup function
// ================================================================
void setup()
{
  Init_Serial();   // Initialize the serial communication
  // Init_MPU();      // Initialize the MPU
  // Init_MotorPin(); // Initialize the motor pin
  // Init_PID();      // Initialize the PID
  Init_ESC();   // Initializa ESC
  espnow_initialize();
}
// ================================================================
// Loop function
// ================================================================
void loop()
{
  // Get_MPUangle();    // Get the angle (anglex) from the IMU sensor
  // Compute_PID();     // Compute the PID output (motor_cmd)
  // Run_Motor();       // Send the PID output (motor_cmd) to the motor
  // SerialDataPrint(); // Print the data on the serial monitor for debugging
  // SerialDataWrite(); // User data to tune the PID parameters

  CtrlPWM = Receive_Data.Receive_PotValue;
  JoyVrx = Receive_Data.Receive_JoyVrx;
  JoyVry = Receive_Data.Receive_JoyVry;
  Button1State = Receive_Data.Receive_Button1State;
  Button2State = Receive_Data.Receive_Button2State;

  ESC_Left1.write(CtrlPWM); // Send the command to the ESC
  ESC_Left2.write(CtrlPWM); // Send the command to the ESC
  ESC_Right1.write(CtrlPWM); // Send the command to the ESC
  ESC_Right2.write(CtrlPWM); // Send the command to the ESC

  // Set data value to be sent
  Sent_Data.Sent_GPS_val = 1;
  Sent_Data.Sent_AngleX = 11;
  Sent_Data.Sent_AngleY = 11;
  Sent_Data.Sent_AngleZ = 11;
  Sent_Data.Sent_GyroX = 111;
  Sent_Data.Sent_GyroY = 111;
  Sent_Data.Sent_GyroZ = 111;
  Sent_Data.Sent_AccX = 1111;
  Sent_Data.Sent_AccY = 1111;
  Sent_Data.Sent_AccZ = 1111;
  // Data sent over espnow
  esp_now_send(controllerAddress, (uint8_t *)&Sent_Data, sizeof(Sent_Data));

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
  // if (micros() - time_prev >= 50000)
  // {
  //   time_prev = micros();
  //   Serial.print(millis());
  //   Serial.print("\t");
  //   Serial.print(anglex);
  //   Serial.print("\t");
  //   Serial.print(motor_cmd);
  //   Serial.print("\t");
  //   Serial.print(kp);
  //   Serial.print("\t");
  //   Serial.print(ki);
  //   Serial.print("\t");
  //   Serial.print(kd);
  //   Serial.print("\t");
  //   Serial.print(anglex_setpoint);

  //   Serial.println();
  // }
}

// ================================================================
// Function to tune the PID parameters. For example:
// To change the P value to 10, type p10
// To change the I value to -5, type i-5
// To change the D value to 2.4, type d2.4
// To change the setpoint to 3, type s3

void SerialDataWrite()
{
  // static String received_chars;
  // while (Serial.available())
  // {
  //   char inChar = (char)Serial.read();
  //   received_chars += inChar;
  //   if (inChar == '\n')
  //   {
  //     switch (received_chars[0])
  //     {
  //     case 'p':
  //       received_chars.remove(0, 1);
  //       kp = received_chars.toFloat();
  //       break;
  //     case 'i':
  //       received_chars.remove(0, 1);
  //       ki = received_chars.toFloat();
  //       break;
  //     case 'd':
  //       received_chars.remove(0, 1);
  //       kd = received_chars.toFloat();
  //       break;
  //     case 's':
  //       received_chars.remove(0, 1);
  //       anglex_setpoint = received_chars.toFloat();
  //     default:
  //       break;
  //     }
  //     received_chars = "";
  //   }
  // }
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
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // There is nothing to do when sending data, this is just for debugging
  Serial.print(micros() / 1000);
  Serial.println("\tData sent!");
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  // debugging serial
  // Serial.print(micros() / 1000);
  // Serial.println("\tData received!");
  // You must copy the incoming data to the local variables
  memcpy(&Receive_Data, incomingData, sizeof(Receive_Data));
}

void Init_ESC()
{
  ESC_Left1.attach(MOTOR_LEFT1_PIN, MIN_SIGNAL, MAX_SIGNAL);
  ESC_Left2.attach(MOTOR_LEFT2_PIN, MIN_SIGNAL, MAX_SIGNAL);
  ESC_Right1.attach(MOTOR_RIGHT1_PIN, MIN_SIGNAL, MAX_SIGNAL);
  ESC_Right2.attach(MOTOR_RIGHT2_PIN, MIN_SIGNAL, MAX_SIGNAL);
  // ESC_Left1.writeMicroseconds(MIN_SIGNAL);
  // ESC_Left2.writeMicroseconds(MIN_SIGNAL);
  // ESC_Right1.writeMicroseconds(MIN_SIGNAL);
  // ESC_Right2.writeMicroseconds(MIN_SIGNAL);
}

// ******************************************
float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ******************************************
void espnow_initialize()
{
  // Set device as a Wifi station
  WiFi.mode(WIFI_STA);
  // Initialize esp now
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataReceive);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, controllerAddress, sizeof(Sent_Data));
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if(esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(F("Failed to add peer"));
    return;
  }
}
