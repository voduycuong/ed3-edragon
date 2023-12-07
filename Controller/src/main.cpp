#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
// #include <PluggableUSB.h>
//  ================================================================
//  Variable declaration
//  ================================================================
#define POT_PIN 34 // Pin 36 attached to the potentiometer
#define JOYSTICK_PIN_SW 35
#define JOYSTICK_PIN_VRX 32
#define JOYSTICK_PIN_VRY 33
#define PUSHBUTTON_1 39
#define PUSHBUTTON_2 36

int joy_vrx;
int joy_vry;

// Insert the MAC address of the other board
uint8_t broadcastAddress[] = {0x48, 0xE7, 0x29, 0xA0, 0x11, 0x98};

// Define the outgoing data, SENT out from this board
typedef struct struct_msg_Sent
{
    int Sent_PotAngle;
    bool Button1State;
    bool Button2State;
} struct_msg_Sent;

// Declare the structure
struct_msg_Sent Sent_Data;

// Variable for espnow communication
esp_now_peer_info_t peerInfo;

// Serial
unsigned long time_prev_serial = 0;

// Object initialization
// Joystick_ joystick(JOYSTICK_PIN_VRX, JOYSTICK_PIN_VRY, JOYSTICK_PIN_SW);

Servo ESC;                   // Define the ESC
int CtrlPWM;                 // Control Signal. Varies between [0 - 180]
unsigned long time_prev = 0; // Variable used for serial monitoring
// ================================================================
// Function declaration
// ================================================================
void SerialDataPrint();  // Function to print data on the serial monitor
void Init_Serial();      // Function to init the serial monitor
void WaitForKeyStroke(); // Function to interact with the serial monitor
void SerialDataWrite();

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void espnow_initialize();

// ================================================================
// Setup
// ================================================================
void setup()
{
    Init_Serial(); // Initialize the serial communication
    Serial.begin(115200);
    espnow_initialize();
    pinMode(JOYSTICK_PIN_VRX, INPUT);
    pinMode(JOYSTICK_PIN_VRY, INPUT);
    pinMode(POT_PIN, INPUT);
    pinMode(PUSHBUTTON_1, INPUT_PULLDOWN);
    pinMode(PUSHBUTTON_2, INPUT_PULLDOWN);
}

// ================================================================
// Loop
// ================================================================
void loop()
{
    CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 0, 180); // Read the pot, map the reading from [0, 4095] to [0, 180]

    // Read Joystick Values
    joy_vrx = analogRead(JOYSTICK_PIN_VRX);
    joy_vry = analogRead(JOYSTICK_PIN_VRY);

    // Read Button States
    bool button1State = digitalRead(PUSHBUTTON_1) == HIGH; // Button is active-high
    bool button2State = digitalRead(PUSHBUTTON_2) == HIGH; // Button is active-high

    // Update Sent Data with Joystick and Button Values
    Sent_Data.Sent_PotAngle = joy_vrx; // Send the joystick X value
    Sent_Data.Button1State = button1State;
    Sent_Data.Button2State = button2State;

    // Data sent over espnow
    esp_now_send(broadcastAddress, (uint8_t *)&Sent_Data, sizeof(Sent_Data));

    if (micros() - time_prev_serial >= 20000)
    {
        time_prev_serial = micros();
        SerialDataPrint(); // Print data on the serial monitor for debugging
    }

    // Print Joystick Values
    if (millis() - time_prev >= 20000)
    {
        time_prev = millis();
        Serial.print("Joystick X: ");
        Serial.print(joy_vrx);
        Serial.print("\tJoystick Y: ");
        Serial.println(joy_vry);
        Serial.print("\tButton 1: ");
        Serial.print(button1State ? "Pressed" : "Released");
        Serial.print("\tButton 2: ");
        Serial.println(button2State ? "Pressed" : "Released");
    }
}

// ================================================================
// Function definition
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
        // Serial.println(CtrlPWM);
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

// ******************************************
void SerialDataWrite()
{
    Serial.print(micros() / 1000);
    Serial.print("\t");
    // Serial.print(Sent_Data.Sent_PotAngle);
    Serial.println();
}

// ******************************************
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // There is nothing to do when sending data, this is just for debugging
    // Serial.print(micros() / 1000);
    // Serial.println("\tData sent!");
    // Serial.print("\r\nLast Packet Send Status:\t");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }
}
