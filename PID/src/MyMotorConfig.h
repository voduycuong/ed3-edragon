#include <Arduino.h>

#define MOTOR_LEFT1_PIN 32      // Pin 32 attached to ESC signal pin
#define MOTOR_LEFT2_PIN 33      // Pin 33 attached to ESC signal pin
#define MOTOR_RIGHT1_PIN 18      // Pin 18 attached to ESC signal pin
#define MOTOR_RIGHT2_PIN 19      // Pin 19 attached to ESC signal pin

// #define PWM_FREQ 30000
// #define PWM_RES 8

// void Init_MotorPin()
// {
//   const int MOTOR_LEFT1_PIN = 32;      // Pin 32 attached to ESC signal pin
//   const int MOTOR_LEFT2_PIN = 33;      // Pin 33 attached to ESC signal pin
//   const int MOTOR_RIGHT1_PIN = 18;     // Pin 18 attached to ESC signal pin
//   const int MOTOR_RIGHT2_PIN = 19      // Pin 19 attached to ESC signal pin
// }

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
