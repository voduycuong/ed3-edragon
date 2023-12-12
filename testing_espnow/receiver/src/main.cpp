#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
// ================================================================
// Variable declaration
// ================================================================
#define MAX_SIGNAL 2000 // Parameter required for the ESC definition
#define MIN_SIGNAL 1000 // Parameter required for the ESC definition
// #define MOTOR_PIN 13    // Pin 13 attached to the ESC signal pin

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

Servo ESC;                 // Define the ESC
int CtrlPWM = 0;           // Control Signal. Varies between [0 - 180]
int JoyVrx = 0;            // X value of joy con position [0 - 4095]
int JoyVry = 0;            // Y value of joy con position [0 - 4095]
bool Button1State = false; // 0 - unpressed. 1 - pressed
bool Button2State = false; // 0 - unpressed. 1 - pressed

unsigned long time_prev = 0; // Variable used for serial monitoring
// ================================================================
// Function declaration
// ================================================================
void SerialDataPrint();  // Function to print data on the serial monitor
void Init_Serial();      // Function to init the serial monitor
void WaitForKeyStroke(); // Function to interact with the serial monitor

void SerialDataWrite();
void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
float floatMap(float, float, float, float, float);
void espnow_initialize();

// ================================================================
// Setup
// ================================================================
void setup()
{
    Init_Serial(); // Initialize the serial communication
    // ESC.attach(MOTOR_PIN, MIN_SIGNAL, MAX_SIGNAL); // Initialize the ESC

    espnow_initialize();
    Serial.begin(115200);
}

// ================================================================
// Loop
// ================================================================
void loop()
{
    CtrlPWM = Receive_Data.Receive_PotValue;
    JoyVrx = Receive_Data.Receive_JoyVrx;
    JoyVry = Receive_Data.Receive_JoyVry;
    Button1State = Receive_Data.Receive_Button1State;
    Button2State = Receive_Data.Receive_Button2State;

    // ESC.write(CtrlPWM); // Send the command to the ESC

    if (micros() - time_prev_serial >= 20000)
    {
        time_prev_serial = micros();
        SerialDataWrite();
    }

    // SerialDataPrint(); // Print data on the serial monitor for debugging
}
// ================================================================
// Function Definition
// ================================================================
void Init_Serial()
{
    Serial.begin(115200);
    while (!Serial)
        ;
}
// ================================================================
void SerialDataPrint()
{
    if (micros() - time_prev >= 20000)
    {
        time_prev = micros();
        Serial.print(millis());
        Serial.print("\t");
        Serial.println(CtrlPWM);
    }
}
// ================================================================
void WaitForKeyStroke()
{
    while (!Serial.available())
        ;
    while (Serial.available())
        Serial.read();
}

void SerialDataWrite()
{
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
    // Serial.print(micros() / 1000);
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
