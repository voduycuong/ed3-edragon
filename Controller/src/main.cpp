#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
// #include <PluggableUSB.h>
//  ================================================================
//  Variable declaration
//  ================================================================
#define POT_PIN 34 // Pin 36 attached to the potentiometer
#define JOYSTICK_SW_PIN 35
#define JOYSTICK_VRX_PIN 32
#define JOYSTICK_VRY_PIN 33
#define BUTTON_1_PIN 39
#define BUTTON_2_PIN 36

// Insert the MAC address of the other board
uint8_t broadcastAddress[] = {0x48, 0xE7, 0x29, 0xA0, 0x11, 0x98};

// Define the outgoing data, SENT out from this board
typedef struct struct_msg_Sent
{
    int Sent_PotAngle;
    bool Sent_Button1State;
    bool Sent_Button2State;
    int Sent_JoyVrx;
    int Sent_JoyVry;
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
int JoyVrx;
int JoyVry;
bool Button1State;
bool Button2State;

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

    pinMode(JOYSTICK_VRX_PIN, INPUT);
    pinMode(JOYSTICK_VRY_PIN, INPUT);
    pinMode(POT_PIN, INPUT);
    pinMode(BUTTON_1_PIN, INPUT_PULLDOWN);
    pinMode(BUTTON_2_PIN, INPUT_PULLDOWN);
}

// ================================================================
// Loop
// ================================================================
void loop()
{
    CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 0, 180); // Read the pot, map the reading from [0, 4095] to [0, 180]

    // Read Joystick Values
    JoyVrx = analogRead(JOYSTICK_VRX_PIN);
    JoyVry = analogRead(JOYSTICK_VRY_PIN);

    // Read Button States
    Button1State = digitalRead(BUTTON_1_PIN) == HIGH; // Button is active-high
    Button2State = digitalRead(BUTTON_2_PIN) == HIGH; // Button is active-high

    // Update Sent Data with Joystick and Button Values
    Sent_Data.Sent_PotAngle = CtrlPWM; // Send the joystick X value
    Sent_Data.Sent_Button1State = Button1State;
    Sent_Data.Sent_Button2State = Button2State;
    Sent_Data.Sent_JoyVrx = JoyVrx;
    Sent_Data.Sent_JoyVry = JoyVry;

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
