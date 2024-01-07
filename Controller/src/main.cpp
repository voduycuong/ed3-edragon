// Arduino library
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// Personal library
#include "Serial_Config.h" // Configure the serial communication

// ================================================================
// Variable declaration
// ================================================================
#define POT_PIN 34          // Pin 34 attached to the potentiometer
#define JOYSTICK_VRX_PIN 32 // Pin 32 attached to the VRX
#define JOYSTICK_VRY_PIN 33 // Pin 33 atxtached to the VRY
#define BUTTON_1_PIN 39     // Pin 39 attached to the Button 1 // Top
#define BUTTON_2_PIN 36     // Pin 36 attached to the Button 2 // Bottom

// MAC address of Drone
uint8_t droneAddress[] = {0x48, 0xE7, 0x29, 0xA0, 0x11, 0x98};

// Variable for espnow communication
esp_now_peer_info_t peerInfo;

// Components declaration
int CtrlPWM = 0; // Control Signal. Varies between [0 - 180]
int JoyVrx = 0;
int JoyVry = 0;
bool Button1State = false;
bool Button2State = false;
int YawVar = 0;

// GPS
double Longitude = 0;
double Latitude = 0;
double Altitude = 0;

// IMU
double AngleX = 0;
double AngleY = 0;
double AngleZ = 0;
double GyroX = 0;
double GyroY = 0;
double GyroZ = 0;

// Init gain of angle
double kp_anglex = 0.5;
double ki_anglex = 0.03;
double kd_anglex = 0.001;

double kp_angley = 0.5;
double ki_angley = 0.03;
double kd_angley = 0.001;

double kp_anglez = 1.0;
double ki_anglez = 0.0;
double kd_anglez = 0.001;

// Init gain of rate
double kp_gyrox = 2;
double ki_gyrox = 0.02;
double kd_gyrox = 0.002;

double kp_gyroy = 2;
double ki_gyroy = 0.02;
double kd_gyroy = 0.002;

double kp_gyroz = 1.0;
double ki_gyroz = 0.0;
double kd_gyroz = 0.001;

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
// Function Declaration
// ================================================================
void SerialDataPrint(); // Data from Controller to Drone
void SerialDataWrite(); // Data from Drone to Controller
void Init_ESPNOW();     // Function to init esp-now connection
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len);

// Define the outgoing data
typedef struct struct_msg_Sent
{
    int Sent_PotAngle;
    int Sent_JoyVrx;
    int Sent_JoyVry;
    bool Sent_Button1State;
    bool Sent_Button2State;

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

} struct_msg_Receive;

// Declare the structure
struct_msg_Sent Sent_Data;
struct_msg_Receive Receive_Data;

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

FLOATUNION_t simulink_kp_anglex;
FLOATUNION_t simulink_ki_anglex;
FLOATUNION_t simulink_kd_anglex;
FLOATUNION_t simulink_kp_angley;
FLOATUNION_t simulink_ki_angley;
FLOATUNION_t simulink_kd_angley;
FLOATUNION_t simulink_kp_anglez;
FLOATUNION_t simulink_ki_anglez;
FLOATUNION_t simulink_kd_anglez;

FLOATUNION_t simulink_kp_gyrox;
FLOATUNION_t simulink_ki_gyrox;
FLOATUNION_t simulink_kd_gyrox;
FLOATUNION_t simulink_kp_gyroy;
FLOATUNION_t simulink_ki_gyroy;
FLOATUNION_t simulink_kd_gyroy;
FLOATUNION_t simulink_kp_gyroz;
FLOATUNION_t simulink_ki_gyroz;
FLOATUNION_t simulink_kd_gyroz;

// ================================================================
// Setup
// ================================================================
void setup()
{
    Init_ESPNOW();
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
    // Sending data ---------------------------------------------------------------
    // Throttle (Potentiometer)
    Sent_Data.Sent_PotAngle = CtrlPWM;

    // Roll-Pitch (Joystick)
    Sent_Data.Sent_JoyVrx = xMapped;
    Sent_Data.Sent_JoyVry = yMapped;

    // Yaw (2 Buttons)
    Sent_Data.Sent_Button1State = Button1State;
    Sent_Data.Sent_Button2State = Button2State;

    // PID for angle (for sending)
    Sent_Data.Sent_kp_anglex = kp_anglex;
    Sent_Data.Sent_ki_anglex = ki_anglex;
    Sent_Data.Sent_kd_anglex = kd_anglex;
    Sent_Data.Sent_kp_angley = kp_angley;
    Sent_Data.Sent_ki_angley = ki_angley;
    Sent_Data.Sent_kd_angley = kd_angley;
    Sent_Data.Sent_kp_anglez = kp_anglez;
    Sent_Data.Sent_ki_anglez = ki_anglez;
    Sent_Data.Sent_kd_anglez = kd_anglez;

    // PID for gyro (for sending)
    Sent_Data.Sent_kp_gyrox = kp_gyrox;
    Sent_Data.Sent_ki_gyrox = ki_gyrox;
    Sent_Data.Sent_kd_gyrox = kd_gyrox;
    Sent_Data.Sent_kp_gyroy = kp_gyroy;
    Sent_Data.Sent_ki_gyroy = ki_gyroy;
    Sent_Data.Sent_kd_gyroy = kd_gyroy;
    Sent_Data.Sent_kp_gyroz = kp_gyroz;
    Sent_Data.Sent_ki_gyroz = ki_gyroz;
    Sent_Data.Sent_kd_gyroz = kd_gyroz;
    // End of sending data ---------------------------------------------------------------

    // Read the pot, map the reading from [0, 4095] to [0, 180]
    CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 0, 180);

    // Read Joystick Values
    JoyVrx = analogRead(JOYSTICK_VRX_PIN);
    JoyVry = analogRead(JOYSTICK_VRY_PIN);

    // Read Button States
    Button1State = digitalRead(BUTTON_1_PIN); // Button is active-high
    Button2State = digitalRead(BUTTON_2_PIN); // Button is active-high

    // Set Yaw value through 2 buttons
    if (Button1State)
        YawVar--;
    if (Button2State)
        YawVar++;

    // Calibrate joystick
    if (JoyVrx < xMid)
        xMapped = map(JoyVrx, xMin, xMid, 0, 2047);
    else
        xMapped = map(JoyVrx, xMid, xMax, 2047, 4095);

    if (JoyVry < yMid)
        yMapped = map(JoyVry, yMin, yMid, 0, 2047);
    else
        yMapped = map(JoyVry, yMid, yMax, 2047, 4095);

    // Data sent over espnow
    esp_now_send(droneAddress, (uint8_t *)&Sent_Data, sizeof(Sent_Data));

    // Receiving data ---------------------------------------------------------------
    // GPS
    Longitude = Receive_Data.Receive_Longitude;
    Latitude = Receive_Data.Receive_Latitude;
    Altitude = Receive_Data.Receive_Altitude;

    // IMU (Angle-Rate)
    AngleX = Receive_Data.Receive_AngleX;
    AngleY = Receive_Data.Receive_AngleY;
    AngleZ = Receive_Data.Receive_AngleZ;
    GyroX = Receive_Data.Receive_GyroX;
    GyroY = Receive_Data.Receive_GyroY;
    GyroZ = Receive_Data.Receive_GyroZ;
    // End of receiving data ---------------------------------------------------------------

    if (micros() - time_prev_serial >= 20000)
    {
        time_prev_serial = micros();
        // SerialDataPrint(); // Print data on the serial monitor for debugging

        // Serial.print("\n\tROLL\t\t\t\tPITCH\t\t\t\tYAW\n");

        Serial.print("P_angle_X: ");
        Serial.println(kp_anglex);
        Serial.print("I_angle_X: ");
        Serial.println(ki_anglex);
        Serial.print("D_angle_X: ");
        Serial.println(kd_anglex);

        Serial.print(">AngleX:");
        Serial.println(AngleX);
        Serial.print(">AngleY:");
        Serial.println(AngleY);

        Serial.print(">Setpoint:");
        Serial.println(0);

        SerialDataWrite();
    }
}

// ================================================================
// Function definition
// ================================================================

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

    simulink_kp_anglex.number = kp_anglex;
    simulink_ki_anglex.number = ki_anglex;
    simulink_kd_anglex.number = kd_anglex;

    simulink_kp_angley.number = kp_angley;
    simulink_ki_angley.number = ki_angley;
    simulink_kd_angley.number = kd_angley;

    simulink_kp_anglez.number = kp_anglez;
    simulink_ki_anglez.number = ki_anglez;
    simulink_kd_anglez.number = kd_anglez;

    simulink_kp_gyrox.number = kp_gyrox;
    simulink_ki_gyrox.number = ki_gyrox;
    simulink_kd_gyrox.number = kd_gyrox;

    simulink_kp_gyroy.number = kp_gyroy;
    simulink_ki_gyroy.number = ki_gyroy;
    simulink_kd_gyroy.number = kd_gyroy;

    simulink_kp_gyroz.number = kp_gyroz;
    simulink_ki_gyroz.number = ki_gyroz;
    simulink_kd_gyroz.number = kd_gyroz;

    if (micros() - time_prev_serial >= 10000)
    {
        time_prev_serial = micros();
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

        for (int i = 0; i < 4; i++)
            Serial.write(simulink_kp_anglex.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_ki_anglex.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_kd_anglex.bytes[i]);

        for (int i = 0; i < 4; i++)
            Serial.write(simulink_kp_angley.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_ki_angley.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_kd_angley.bytes[i]);

        for (int i = 0; i < 4; i++)
            Serial.write(simulink_kp_anglez.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_ki_anglez.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_kd_anglez.bytes[i]);

        for (int i = 0; i < 4; i++)
            Serial.write(simulink_kp_gyroz.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_ki_gyroz.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_kd_gyroz.bytes[i]);

        for (int i = 0; i < 4; i++)
            Serial.write(simulink_kp_gyroz.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_ki_gyroz.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_kd_gyroz.bytes[i]);

        for (int i = 0; i < 4; i++)
            Serial.write(simulink_kp_gyroz.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_ki_gyroz.bytes[i]);
        for (int i = 0; i < 4; i++)
            Serial.write(simulink_kd_gyroz.bytes[i]);

        Serial.print('\n');
    }
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
void Init_ESPNOW()
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
    }
    // Clear input modification
    modification = "";
    value = "";
    option = "";
}
