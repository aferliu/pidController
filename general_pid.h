#ifndef GENERAL_PID_H
#define GENERAL_PID_H

#define PID_LIBRARY_VERSION 0.0.3

/*
* The controller mode is divided into automatic and manual mode
* GPID_MANUAL - Disable controller calculation output
* GPID_AUTOMATIC - Calculate the output through the PID controller
*/ 
typedef enum ControlMode
{
    GPID_MANUAL = 0,
    GPID_AUTOMATIC = 1
} ControlMode;

/*
* Is the controller output and input in direct/reverse proportion?
*/
typedef enum ControlDirection
{
    GPID_DIRECT = 0,
    GPID_REVERSE = 1
} ControlDirection;

/*
* Switch different forms of controllers to play different roles
* GPID_ON_MEASURE - It will reduce the occurrence of overshoot
* GPID_ON_ERROR - Traditional PID controller
*/
typedef enum ControlType
{
    GPID_ON_MEASURE = 0,
    GPID_ON_ERROR = 1
} ControlType;


typedef struct
{
    double dispKp; // holding in to tuning parameters in user-entered.
    double dispKi;
    double dispKd;

    double kp; // (P)roportional Tuning Parameter
    double ki; // (I)ntegral Tuning Parameter
    double kd; // (D)erivative Tuning Parameter

    ControlDirection controllerDirection; /* DIRECT:  +output leads to +input
                                REVERSE: +output leads to -input*/

    ControlType pOn; // holding in to tuning parameters in user-entered.

    double myInput;          // Sensor measurement value.
    double myOutput;         // Output value calculated by the controller.
    double mySetpoint;       // Expected value of controller setting

    unsigned long lastTime;   // The time when the controller wa entered.
    double outputSum;         // Total historical points.
    double lastInput;         // the last sensor measurement value
    unsigned long SampleTime; // How often the controller is executed. (us)
    double outMin;
    double outMax;
    ControlMode inAuto; /* AUTOMATIC: Using controller to calculate output.
                   MANUAL: Calculate output without controller */

    ControlType pOnE; /* P_ON_M: specifies that Proportional on Measurement be used
                  P_ON_E: (Proportional on Error) is the default behavior
                  note: P_ON_E will smoothly when the setpoint is changed. */
} G_PID;

void GPIDInit(G_PID *pid,
              double kp, double ki, double kd, ControlType POn, 
              ControlDirection ControllerDirection, double Min, double Max, int SampleTime);

void GPIDSetMode(G_PID *pid, ControlMode Mode);
int GPIDCompute(G_PID *pid, unsigned long nowUs);
void GPIDSetOutputLimits(G_PID *pid, double min, double max);
void GPIDSetTunings(G_PID *pid, double kp, double ki, double kd);
void GPIDSetTuningsEx(G_PID *pid, double kp, double ki, double kd, ControlType pOn);
void GPIDSetControllerDirection(G_PID *pid, ControlDirection Direction);
void GPIDSetSampleTime(G_PID *pid, int NewSampleTime);

double GPIDGetKp(G_PID *pid);
double GPIDGetKi(G_PID *pid);
double GPIDGetKd(G_PID *pid);
int GPIDGetMode(G_PID *pid);
int GPIDGetDirection(G_PID *pid);

#endif