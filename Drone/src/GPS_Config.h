#include <Arduino.h>
#include <TinyGPSPlus.h>

// ================================================================
// Variable declaration
// ================================================================
#define RXD2 16
#define TXD2 17

TinyGPSPlus gps;

// GPS data variables
double Latitude = 0;
double Longitude = 0;
float Altitude = 0;
int Year = 0;
byte Month = 0;
byte Day = 0;
byte Hour = 0;
byte Minute = 0;
byte Second = 0;

// ================================================================
// Function declaration
// ================================================================
void Init_Serial();
void Get_GPSData();
void DisplayInfo();

// ================================================================
// Setup
// ================================================================
void setup() {
    Init_Serial();
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

// ================================================================
// Loop
// ================================================================
void loop() {
    Get_GPSData();
}

// ================================================================
// Function definition
// ================================================================
void Init_Serial() {
    Serial.begin(115200);
    while (!Serial) ;
}

void Get_GPSData() {
    while (Serial2.available() > 0) {
        if (gps.encode(Serial2.read())) {
            DisplayInfo();
        }
    }
}

void DisplayInfo() {
    if (gps.location.isValid()) {
        Latitude = gps.location.lat();
        Longitude = gps.location.lng();
    }

    if (gps.altitude.isValid()) {
        Altitude = gps.altitude.meters();
    }

    if (gps.date.isValid()) {
        Day = gps.date.day();
        Month = gps.date.month();
        Year = gps.date.year();
    }

    if (gps.time.isValid()) {
        Hour = gps.time.hour();
        Minute = gps.time.minute();
        Second = gps.time.second();
    }

    Serial.print("Location: ");
    Serial.print(Latitude, 6);
    Serial.print(", ");
    Serial.print(Longitude, 6);
    Serial.print("\tAltitude: ");
    Serial.print(Altitude);
    Serial.println("m");
    Serial.print("\tDate: ");
    Serial.print(Day);
    Serial.print("/");
    Serial.print(Month);
    Serial.print("/");
    Serial.print(Year);
    Serial.print("\tTime: ");
    Serial.print(Hour);
    Serial.print(":");
    Serial.print(Minute);
    Serial.print(":");
    Serial.println(Second);
}