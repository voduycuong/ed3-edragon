#include <Arduino.h>
#include <PID_v1.h>

extern double anglex;
extern double angley;
extern double anglez;

extern double gyrox;
extern double gyroy;
extern double gyroz;

// ================================================================
// Variable declaration
// ================================================================
// The PID object is configured as follows:
// input = sensor, variable to be controller;
// output = pid output, command sent to the motors;
// setpoint = reference setpoint, the desired angle (usually 0deg to maintain an upward position)
// PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

double pid_output_x = 0;
double pid_output_y = 0;
double pid_output_z = 0;

// Angle PID gain
double kp_anglex;
double ki_anglex;
double kd_anglex;

double kp_angley;
double ki_angley;
double kd_angley;

double kp_anglez;
double ki_anglez;
double kd_anglez;

// Rate PID gain
double kp_gyrox;
double ki_gyrox;
double kd_gyrox;

double kp_gyroy;
double ki_gyroy;
double kd_gyroy;

double kp_gyroz;
double ki_gyroz;
double kd_gyroz;

// From the remote controller
extern double anglex_setpoint;
extern double angley_setpoint;
extern double anglez_setpoint;

double gyrox_setpoint;
double gyroy_setpoint;
double gyroz_setpoint;

// Inner loop (fast)
PID gyroxPID(&gyrox, &pid_output_x, &gyrox_setpoint, kp_gyrox, ki_gyrox, kd_gyroz, DIRECT);
PID gyroyPID(&gyroy, &pid_output_y, &gyroy_setpoint, kp_gyroy, ki_gyroy, kd_gyroy, DIRECT);
PID gyrozPID(&gyroz, &pid_output_z, &gyroz_setpoint, kp_gyroz, ki_gyroz, kd_gyroz, DIRECT);

// Outer loop (slow)
PID anglexPID(&anglex, &gyrox_setpoint, &anglex_setpoint, kp_anglex, ki_anglex, kd_anglex, DIRECT);
PID angleyPID(&angley, &gyroy_setpoint, &angley_setpoint, kp_angley, ki_angley, kd_angley, DIRECT);
PID anglezPID(&anglez, &gyroz_setpoint, &anglez_setpoint, kp_anglez, ki_anglez, kd_anglez, DIRECT);

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
    anglexPID.SetMode(AUTOMATIC);
    anglexPID.SetOutputLimits(-90, 90);
    anglexPID.SetSampleTime(10);

    angleyPID.SetMode(AUTOMATIC);
    angleyPID.SetOutputLimits(-90, 90);
    angleyPID.SetSampleTime(10);

    anglezPID.SetMode(AUTOMATIC);
    anglezPID.SetOutputLimits(-90, 90);
    anglezPID.SetSampleTime(10);

    gyroxPID.SetMode(AUTOMATIC);
    gyroxPID.SetOutputLimits(-90, 90);
    gyroxPID.SetSampleTime(10);

    gyroyPID.SetMode(AUTOMATIC);
    gyroyPID.SetOutputLimits(-90, 90);
    gyroyPID.SetSampleTime(10);

    gyrozPID.SetMode(AUTOMATIC);
    gyrozPID.SetOutputLimits(-90, 90);
    gyrozPID.SetSampleTime(10);
}
// ================================================================
void Compute_PID()
{
    anglexPID.SetTunings(kp_anglex, ki_anglex, kd_anglex);
    angleyPID.SetTunings(kp_angley, ki_angley, kd_angley);
    anglezPID.SetTunings(kp_anglez, ki_anglez, kd_anglez);

    gyroxPID.SetTunings(kp_gyrox, ki_gyrox, kd_gyrox);
    gyroyPID.SetTunings(kp_gyroy, ki_gyroy, kd_gyroy);
    gyrozPID.SetTunings(kp_gyroz, ki_gyroz, kd_gyroz);

    anglexPID.Compute();
    angleyPID.Compute();
    anglezPID.Compute();

    gyroxPID.Compute();
    gyroyPID.Compute();
    gyrozPID.Compute();
}
// ================================================================