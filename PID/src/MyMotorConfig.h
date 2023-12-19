#include <Arduino.h>
#include <ESP32Servo.h>

#define MOTOR_LEFT1_PIN 32      // Pin 32 attached to ESC signal pin
#define MOTOR_LEFT2_PIN 33      // Pin 33 attached to ESC signal pin
#define MOTOR_RIGHT1_PIN 18      // Pin 18 attached to ESC signal pin
#define MOTOR_RIGHT2_PIN 19      // Pin 19 attached to ESC signal pin\

Servo ESC_Left1;     // Define ESC
Servo ESC_Left2;     // Define ESC
Servo ESC_Right1;    // Define ESC
Servo ESC_Right2;    // Define ESC

void Init_ESC()
{
  ESC_Left1.attach(MOTOR_LEFT1_PIN, MIN_SIGNAL, MAX_SIGNAL);
  ESC_Left2.attach(MOTOR_LEFT2_PIN, MIN_SIGNAL, MAX_SIGNAL);
  ESC_Right1.attach(MOTOR_RIGHT1_PIN, MIN_SIGNAL, MAX_SIGNAL);
  ESC_Right2.attach(MOTOR_RIGHT2_PIN, MIN_SIGNAL, MAX_SIGNAL);
}

void Run_Motor_Left1()
{
  ESC_Left1.write(CtrlPWM); // Send the command to the ESC
}

void Run_Motor_Left2()
{
  ESC_Left2.write(CtrlPWM); // Send the command to the ESC
}

void Run_Motor_Right1()
{
  ESC_Right1.write(CtrlPWM); // Send the command to the ESC
}

void Run_Motor_Right2()
{
  ESC_Right2.write(CtrlPWM); // Send the command to the ESC
}
