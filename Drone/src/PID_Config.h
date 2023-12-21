#include <Arduino.h>
#include <PID_v1.h>

extern double anglex;
extern double angley;
extern double anglez;
// ================================================================
// Variable declaration
// ================================================================
// The PID object is configured as follows:
// input = sensor, variable to be controller;
// output = pid output, command sent to the motors;
// setpoint = reference setpoint, the desired angle (usually 0deg to maintain an upward position)
// PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

double pid_output_roll = 0;
double pid_output_pitch = 0;
double pid_output_yaw = 0;

// Init gain
double kp = 5.0;
double ki = 0.0;
double kd = 0.0;

double anglex_setpoint = 0;
double angley_setpoint = 0;
double anglez_setpoint = 0;
// Correct gain
// double kp = 12.0, ki = 100.0, kd = 0.15, anglex_setpoint = 1;

PID roll_PID(&anglex, &pid_output_roll, &anglex_setpoint, kp, ki, kd, DIRECT);
PID pitch_PID(&angley, &pid_output_pitch, &angley_setpoint, kp, ki, kd, DIRECT);
PID yaw_PID(&anglez, &pid_output_yaw, &anglez_setpoint, kp, ki, kd, DIRECT);

// ================================================================
// Function Declaration
// ================================================================
void Init_PID();
void Compute_PID();

// ================================================================
// Function Definition
// ================================================================
void Init_PID()
{
    roll_PID.SetMode(AUTOMATIC);
    roll_PID.SetOutputLimits(-127, 127);
    roll_PID.SetSampleTime(10);

    pitch_PID.SetMode(AUTOMATIC);
    pitch_PID.SetOutputLimits(-127, 127);
    pitch_PID.SetSampleTime(10);

    yaw_PID.SetMode(AUTOMATIC);
    yaw_PID.SetOutputLimits(-127, 127);
    yaw_PID.SetSampleTime(10);
}
// ================================================================
void Compute_PID()
{
    roll_PID.SetTunings(kp, ki, kd);
    pitch_PID.SetTunings(kp, ki, kd);
    yaw_PID.SetTunings(kp, ki, kd);

    roll_PID.Compute();
    pitch_PID.Compute();
    yaw_PID.Compute();

    if (abs(anglex) > 50 || abs(angley) > 50 || abs(anglez) > 50)
    {
        pid_output_roll = 0;  // motor stop when fall
        pid_output_pitch = 0; // motor stop when fall
        pid_output_yaw = 0;   // motor stop when fall
    }
}
// ================================================================