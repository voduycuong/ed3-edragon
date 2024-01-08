// Arduino library
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// Personal library
#include "Serial_Config.h" // Configure the serial communication
//  ================================================================
//  Variable declaration
//  ================================================================
#define POT_PIN 34          // Pin 34 attached to the potentiometer
#define JOYSTICK_VRX_PIN 32 // Pin 32 attached to the VRX
#define JOYSTICK_VRY_PIN 33 // Pin 33 atxtached to the VRY
#define BUTTON_1_PIN 39     // Pin 39 attached to the Button 1 // Top
#define BUTTON_2_PIN 36     // Pin 36 attached to the Button 2 // Bottom

// Insert the MAC address of the other board
uint8_t droneAddress[] = {0x48, 0xE7, 0x29, 0xA0, 0x11, 0x98};

// Define the outgoing data, SENT out from this board
typedef struct struct_msg_Sent
{
    int Sent_PotAngle;
    int Sent_JoyVrx;
    int Sent_JoyVry;
    bool Sent_Button1State;
    bool Sent_Button2State;
} struct_msg_Sent;

// Define the incoming data
typedef struct struct_msg_Receive
{
    double Receive_Longitude;
    double Receive_Latitude;
    double Receive_Altitude;
    double Receive_AngleX;
    double Receive_AngleY;
    double Receive_AngleZ;
    double Receive_GyroX;
    double Receive_GyroY;
    double Receive_GyroZ;
    double Receive_PidOutputX;
    double Receive_PidOutputY;
    double Receive_PidOutputZ;

    // double Receive_kp_anglex;
    // double Receive_ki_anglex;
    // double Receive_kd_anglex;

    // double Receive_kp_angley;
    // double Receive_ki_angley;
    // double Receive_kd_angley;

    // double Receive_kp_anglez;
    // double Receive_ki_anglez;
    // double Receive_kd_anglez;

    // double Receive_kp_gyrox;
    // double Receive_ki_gyrox;
    // double Receive_kd_gyrox;

    // double Receive_kp_gyroy;
    // double Receive_ki_gyroy;
    // double Receive_kd_gyroy;

    // double Receive_kp_gyroz;
    // double Receive_ki_gyroz;
    // double Receive_kd_gyroz;

} struct_msg_Receive;

// Declare the structure
struct_msg_Sent Sent_Data;
struct_msg_Receive Receive_Data;

// Variable for espnow communication
esp_now_peer_info_t peerInfo;

// Serial
unsigned long time_prev_serial = 0;

// Components declaration
int CtrlPWM = 0; // Control Signal. Varies between [0 - 180]
int JoyVrx = 0;
int JoyVry = 0;
bool Button1State = false;
bool Button2State = false;
int YawVar = 0;

// Received values
double Longitude = 0;
double Latitude = 0;
double Altitude = 0;
double AngleX = 0;
double AngleY = 0;
double AngleZ = 0;
double GyroX = 0;
double GyroY = 0;
double GyroZ = 0;

// Init gain of angle
double kp_anglex = 0.0;
double ki_anglex = 0.0;
double kd_anglex = 0.0;
double kp_angley = 0.0;
double ki_angley = 0.0;
double kd_angley = 0.0;
double kp_anglez = 0.0;
double ki_anglez = 0.0;
double kd_anglez = 0.0;

// Init gain of rate
double kp_gyrox = 0.0;
double ki_gyrox = 0.0;
double kd_gyrox = 0.0;
double kp_gyroy = 0.0;
double ki_gyroy = 0.0;
double kd_gyroy = 0.0;
double kp_gyroz = 0.0;
double ki_gyroz = 0.0;
double kd_gyroz = 0.0;

// Variables for calibrate joystick
int xMin = 0;
int xMid = 1873;
int xMax = 4095;
int yMin = 0;
int yMid = 1820;
int yMax = 4095;
int xMapped = 0;
int yMapped = 0;

// ================================================================
// Function declaration
// ================================================================
void SerialDataPrint();  // Function to print data on the serial monitor
void Init_Serial();      // Function to init the serial monitor
void WaitForKeyStroke(); // Function to interact with the serial monitor
void SerialDataWrite();

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
void espnow_initialize();

// ================================================================
// Setup
// ================================================================
void setup()
{
    espnow_initialize();
    Init_Serial(); // Initialize the serial communication
    Serial.begin(115200);
    pinMode(BUTTON_1_PIN, HIGH);
    pinMode(BUTTON_2_PIN, HIGH);
}

// ================================================================
// Loop
// ================================================================
void loop()
{
    Longitude = Receive_Data.Receive_Longitude;
    Latitude = Receive_Data.Receive_Latitude;
    Altitude = Receive_Data.Receive_Altitude;
    AngleX = Receive_Data.Receive_AngleX;
    AngleY = Receive_Data.Receive_AngleY;
    AngleZ = Receive_Data.Receive_AngleZ;
    GyroX = Receive_Data.Receive_GyroX;
    GyroY = Receive_Data.Receive_GyroY;
    GyroZ = Receive_Data.Receive_GyroZ;

    // kp_anglex = Receive_Data.Receive_kp_anglex;
    // ki_anglex = Receive_Data.Receive_ki_anglex;
    // kd_anglex = Receive_Data.Receive_kd_anglex;
    // kp_angley = Receive_Data.Receive_kp_angley;
    // ki_angley = Receive_Data.Receive_ki_angley;
    // kd_angley = Receive_Data.Receive_kd_angley;
    // kp_anglez = Receive_Data.Receive_kp_anglez;
    // ki_anglez = Receive_Data.Receive_ki_anglez;
    // kd_anglez = Receive_Data.Receive_kd_anglez;

    // kp_gyrox = Receive_Data.Receive_kp_gyrox;
    // ki_gyrox = Receive_Data.Receive_ki_gyrox;
    // kd_gyrox = Receive_Data.Receive_kd_gyrox;
    // kp_gyroy = Receive_Data.Receive_kp_gyroy;
    // ki_gyroy = Receive_Data.Receive_ki_gyroy;
    // kd_gyroy = Receive_Data.Receive_kd_gyroy;
    // kp_gyroz = Receive_Data.Receive_kp_gyroz;
    // ki_gyroz = Receive_Data.Receive_ki_gyroz;
    // kd_gyroz = Receive_Data.Receive_kd_gyroz;

    CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 0, 180); // Read the pot, map the reading from [0, 4095] to [0, 180]

    // Read Joystick Values
    JoyVrx = analogRead(JOYSTICK_VRX_PIN);
    JoyVry = analogRead(JOYSTICK_VRY_PIN);

    // Read Button States
    Button1State = digitalRead(BUTTON_1_PIN); // Button is active-high
    Button2State = digitalRead(BUTTON_2_PIN); // Button is active-high

    if (Button1State)
    {
        YawVar--;
    }
    if (Button2State)
    {
        YawVar++;
    }
    // Calibrate joystick
    if (JoyVrx < xMid)
        xMapped = map(JoyVrx, xMin, xMid, 0, 2047);
    else
        xMapped = map(JoyVrx, xMid, xMax, 2047, 4095);

    if (JoyVry < yMid)
        yMapped = map(JoyVry, yMin, yMid, 0, 2047);
    else
        yMapped = map(JoyVry, yMid, yMax, 2047, 4095);

    // Update Sent Data with Joystick and Button Values
    Sent_Data.Sent_PotAngle = CtrlPWM; // Send the joystick X value
    Sent_Data.Sent_JoyVrx = xMapped;
    Sent_Data.Sent_JoyVry = yMapped;
    Sent_Data.Sent_Button1State = Button1State;
    Sent_Data.Sent_Button2State = Button2State;

    // Data sent over espnow
    esp_now_send(droneAddress, (uint8_t *)&Sent_Data, sizeof(Sent_Data));

    if (micros() - time_prev_serial >= 20000)
    {
        time_prev_serial = micros();
        // SerialDataPrint(); // Print data on the serial monitor for debugging

        // Serial.print("\n\tROLL\t\t\t\tPITCH\t\t\t\tYAW\n");

        // Serial.print(kp_anglex, 3);
        // Serial.print("\t");
        // Serial.print(ki_anglex, 3);
        // Serial.print("\t");
        // Serial.print(kd_anglex, 3);
        // Serial.print("\t\t");
        // Serial.print(kp_angley, 3);
        // Serial.print("\t");
        // Serial.print(ki_angley, 3);
        // Serial.print("\t");
        // Serial.print(kd_angley, 3);
        // Serial.print("\t\t");
        // Serial.print(kp_anglez, 3);
        // Serial.print("\t");
        // Serial.print(ki_anglez, 3);
        // Serial.print("\t");
        // Serial.print(kd_anglez, 3);

        // Serial.print("\n");

        // Serial.print(kp_gyrox, 3);
        // Serial.print("\t");
        // Serial.print(ki_gyrox, 3);
        // Serial.print("\t");
        // Serial.print(kd_gyrox, 3);
        // Serial.print("\t\t");
        // Serial.print(kp_gyroy, 3);
        // Serial.print("\t");
        // Serial.print(ki_gyroy, 3);
        // Serial.print("\t");
        // Serial.print(kd_gyroy, 3);
        // Serial.print("\t\t");
        // Serial.print(kp_gyroz, 3);
        // Serial.print("\t");
        // Serial.print(ki_gyroz, 3);
        // Serial.print("\t");
        // Serial.print(kd_gyroz, 3);

        // Serial.println();
    }

    // Print Joystick Values
    // if (millis() - time_prev >= 20000)
    // {
    //     time_prev = millis();

    //     // Receiving
    //     time_prev = micros();
    //     Serial.print(millis());
    //     Serial.print("\tGPS: ");
    //     Serial.print(GpsVal);
    //     Serial.print("\tAngleX: ");
    //     Serial.print(AngleX);
    //     Serial.print("\tAngleY: ");
    //     Serial.print(AngleY);
    //     Serial.print("\tAngleZ: ");
    //     Serial.print(AngleZ);
    //     Serial.print("\tPID X: ");
    //     Serial.print(PidOutputX);
    //     Serial.print("\tPID Y: ");
    //     Serial.print(PidOutputY);
    //     Serial.print("\tPID Z: ");
    //     Serial.print(PidOutputZ);
    //     Serial.print("\t");
    //     Serial.print(CtrlPWM);
    //     Serial.println();
    // -----------------------------------------

    // Serial.print("\tJX: ");
    // Serial.print(JoyVrx);
    // Serial.print("\tJY: ");
    // Serial.print(JoyVry);
    // Serial.print("\tB1: ");
    // Serial.print(Button1State);
    // Serial.print("\tB2: ");
    // Serial.print(Button2State);
    // Serial.println();
    // }
}

// ================================================================
// Function definition
// ================================================================
FLOATUNION_t simulink_longitude;
FLOATUNION_t simulink_latitude;
FLOATUNION_t simulink_altitude;
FLOATUNION_t simulink_anglex;
FLOATUNION_t simulink_angley;
FLOATUNION_t simulink_anglez;
FLOATUNION_t simulink_gyrox;
FLOATUNION_t simulink_gyroy;
FLOATUNION_t simulink_gyroz;
FLOATUNION_t simulink_anglex_setpoint;
FLOATUNION_t simulink_angley_setpoint;
FLOATUNION_t simulink_anglez_setpoint;

// FLOATUNION_t simulink_kp_anglex;
// FLOATUNION_t simulink_ki_anglex;
// FLOATUNION_t simulink_kd_anglex;

// FLOATUNION_t simulink_kp_angley;
// FLOATUNION_t simulink_ki_angley;
// FLOATUNION_t simulink_kd_angley;

// FLOATUNION_t simulink_kp_anglez;
// FLOATUNION_t simulink_ki_anglez;
// FLOATUNION_t simulink_kd_anglez;

// FLOATUNION_t simulink_kp_gyrox;
// FLOATUNION_t simulink_ki_gyrox;
// FLOATUNION_t simulink_kd_gyrox;

// FLOATUNION_t simulink_kp_gyroy;
// FLOATUNION_t simulink_ki_gyroy;
// FLOATUNION_t simulink_kd_gyroy;

// FLOATUNION_t simulink_kp_gyroz;
// FLOATUNION_t simulink_ki_gyroz;
// FLOATUNION_t simulink_kd_gyroz;
void SerialDataPrint()
{
    simulink_longitude.number = Longitude;
    simulink_latitude.number = Latitude;
    simulink_altitude.number = Altitude;
    simulink_anglex.number = AngleX;
    simulink_angley.number = AngleY;
    simulink_anglez.number = AngleZ;
    simulink_gyrox.number = GyroX;
    simulink_gyroy.number = GyroY;
    simulink_gyroz.number = GyroZ;
    simulink_anglex_setpoint.number = JoyVrx;
    simulink_angley_setpoint.number = JoyVry;
    simulink_anglez_setpoint.number = YawVar;

    // simulink_kp_anglex.number = kp_anglex;
    // simulink_ki_anglex.number = ki_anglex;
    // simulink_kd_anglex.number = kd_anglex;

    // simulink_kp_angley.number = kp_angley;
    // simulink_ki_angley.number = ki_angley;
    // simulink_kd_angley.number = kd_angley;

    // simulink_kp_anglez.number = kp_anglez;
    // simulink_ki_anglez.number = ki_anglez;
    // simulink_kd_anglez.number = kd_anglez;

    // simulink_kp_gyrox.number = kp_gyrox;
    // simulink_ki_gyrox.number = ki_gyrox;
    // simulink_kd_gyrox.number = kd_gyrox;

    // simulink_kp_gyroy.number = kp_gyroy;
    // simulink_ki_gyroy.number = ki_gyroy;
    // simulink_kd_gyroy.number = kd_gyroy;

    // simulink_kp_gyroz.number = kp_gyroz;
    // simulink_ki_gyroz.number = ki_gyroz;
    // simulink_kd_gyroz.number = kd_gyroz;

    if (micros() - time_prev >= 10000)
    {
        time_prev = micros();
        Serial.write('A');
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_longitude.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_latitude.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_altitude.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_anglex.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_angley.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_anglez.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_gyrox.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_gyroy.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_gyroz.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_anglex_setpoint.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_angley_setpoint.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_anglez_setpoint.bytes[i]);

        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_kp_anglex.bytes[i]);
        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_ki_anglex.bytes[i]);
        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_kd_anglex.bytes[i]);

        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_kp_angley.bytes[i]);
        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_ki_angley.bytes[i]);
        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_kd_angley.bytes[i]);

        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_kp_anglez.bytes[i]);
        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_ki_anglez.bytes[i]);
        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_kd_anglez.bytes[i]);

        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_kp_gyroz.bytes[i]);
        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_ki_gyroz.bytes[i]);
        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_kd_gyroz.bytes[i]);

        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_kp_gyroz.bytes[i]);
        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_ki_gyroz.bytes[i]);
        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_kd_gyroz.bytes[i]);

        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_kp_gyroz.bytes[i]);
        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_ki_gyroz.bytes[i]);
        // for (int i = 0; i < 4; i++)
        //     Serial.write(simulink_kd_gyroz.bytes[i]);

        Serial.print('\n');
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

void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    // debugging serial
    // Serial.print(micros() / 1000);
    // Serial.println("\tData received!");
    // You must copy the incoming data to the local variables
    memcpy(&Receive_Data, incomingData, sizeof(Receive_Data));
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
    esp_now_register_recv_cb(OnDataReceive);

    // Register peer
    memcpy(peerInfo.peer_addr, droneAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }
}