#include <Arduino.h>
#include <ESP32Servo.h>

#define MOTOR_1_PIN 32 // Pin 32 attached to ESC signal pin
#define MOTOR_2_PIN 33 // Pin 33 attached to ESC signal pin
#define MOTOR_3_PIN 18 // Pin 18 attached to ESC signal pin
#define MOTOR_4_PIN 19 // Pin 19 attached to ESC signal pin

#define MAX_SIGNAL 2000 // Parameter required for ESC definition
#define MIN_SIGNAL 1000 // Parameter required for the ESC definition

extern int CtrlPWM;

Servo ESC_1; // Define ESC
Servo ESC_2; // Define ESC
Servo ESC_3; // Define ESC
Servo ESC_4; // Define ESC

void Init_ESC()
{
    ESC_1.attach(MOTOR_1_PIN, MIN_SIGNAL, MAX_SIGNAL);
    ESC_2.attach(MOTOR_2_PIN, MIN_SIGNAL, MAX_SIGNAL);
    ESC_3.attach(MOTOR_3_PIN, MIN_SIGNAL, MAX_SIGNAL);
    ESC_4.attach(MOTOR_4_PIN, MIN_SIGNAL, MAX_SIGNAL);
}

void Run_Motor()
{
    // Send command to ESCs
    ESC_1.write(CtrlPWM + pid_output_x + pid_output_y - pid_output_z);
    ESC_2.write(CtrlPWM - pid_output_x + pid_output_y + pid_output_z);
    ESC_3.write(CtrlPWM - pid_output_x - pid_output_y - pid_output_z);
    ESC_4.write(CtrlPWM + pid_output_x - pid_output_y + pid_output_z);
}