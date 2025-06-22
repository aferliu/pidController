#include "general_pid.h"

static void Initialize(G_PID *pid);

void GPIDInit(G_PID *pid,
              double kp, double ki, double kd, ControlType POn, 
              ControlDirection ControllerDirection, double Min, double Max, 
              int SampleTime)
{
    pid->myInput = 0;
    pid->myOutput = 0;
    pid->mySetpoint = 0;

    pid->pOn = POn;
    pid->pOnE = POn;

    pid->controllerDirection = ControllerDirection;
    GPIDSetTunings(pid, kp, ki, kd);

    pid->outMin = Min;
    pid->outMax = Max;

    pid->SampleTime = SampleTime;

    GPIDSetMode(pid, GPID_AUTOMATIC);
}

void GPIDSetMode(G_PID *pid, ControlMode Mode)
{
    ControlMode newmode = (Mode == GPID_AUTOMATIC);
    if (newmode && !pid->inAuto)
    {
        Initialize(pid);
    }
    pid->inAuto = Mode;
}

int GPIDCompute(G_PID *pid, unsigned long nowUs)
{
    if (!pid->inAuto) return -1;
    unsigned long now = nowUs;
    unsigned long timeChange = now - pid->lastTime;
    if (timeChange > pid->SampleTime)
    {
        double input = pid->myInput;
        double error = pid->mySetpoint - input;
        double dinput = (input - pid->lastInput);
        double output = 0;

        pid->outputSum += pid->ki * error;

        if (!pid->pOnE) pid->outputSum -= pid->kp * dinput;
        if (pid->outputSum > pid->outMax) pid->outputSum = pid->outMax;
        else if (pid->outputSum < pid->outMin) pid->outputSum = pid->outMin;

        if (pid->pOnE) output = pid->kp * error;
        else output = 0;

        output += pid->outputSum - pid->kd * dinput;

        if (output > pid->outMax) output = pid->outMax;
        else if (output < pid->outMin) output = pid->outMin;
        pid->myOutput = output;

        pid->lastInput = input;
        pid->lastTime = now;
        return 0;
    }
    return -1;
}

void GPIDSetOutputLimits(G_PID *pid, double Min, double Max)
{
    if (Min > Max) return;
    pid->outMin = Min;
    pid->outMax = Max;

    if (pid->inAuto)
    {
        if (pid->myOutput > pid->outMax) pid->myOutput = pid->outMax;
        else if (pid->myOutput < pid->outMin) pid->myOutput = pid->outMin;

        if (pid->outputSum > pid->outMax) pid->outputSum = pid->outMax;
        else if (pid->outputSum < pid->outMin) pid->outputSum = pid->outMin;
    }
}

void GPIDSetTunings(G_PID *pid, double kp, double ki, double kd)
{
    GPIDSetTuningsEx(pid, kp, kd, ki, pid->pOn);
}

void GPIDSetTuningsEx(G_PID *pid, double kp, double ki, double kd, ControlType pOn)
{
    if (kp < 0 || ki < 0 || kd < 0) return;

    pid->pOn = pOn;
    pid->pOnE = (pid->pOn == GPID_ON_ERROR);

    pid->dispKp = kp; pid->dispKi = ki; pid->dispKd = kd;

    double SampleTimeInSec = ((double)pid->SampleTime)/1000.0f;
    pid->kp = kp;
    pid->ki = ki * SampleTimeInSec;
    pid->kd = kd * SampleTimeInSec;

    if (pid->controllerDirection == GPID_REVERSE)
    {
        pid->kp = (0 - pid->kp);
        pid->ki = (0 - pid->ki);
        pid->kd = (0 - pid->kd);
    }
}

void GPIDSetControllerDirection(G_PID *pid, ControlDirection Direction)
{
    if (pid->inAuto && (Direction != pid->controllerDirection))
    {
        pid->kp = (0 - pid->kp);
        pid->ki = (0 - pid->ki);
        pid->kd = (0 - pid->kd);
    }
    pid->controllerDirection = Direction;
}

void GPIDSetSampleTime(G_PID *pid, int NewSampleTime)
{
    if (NewSampleTime > 0)
    {
        double ratio = (double)NewSampleTime / (double)pid->SampleTime;
        pid->ki *= ratio;
        pid->kd /= ratio;
        pid->SampleTime = (unsigned long)NewSampleTime;
    }
}

static void Initialize(G_PID *pid)
{
    pid->outputSum = pid->myOutput;
    pid->lastInput = pid->myInput;

    if (pid->outputSum > pid->outMax)
        pid->outputSum = pid->outMax;
    else if (pid->outputSum < pid->outMin)
        pid->outputSum = pid->outMin;
}

double GPIDGetKp(G_PID *pid)
{
    return pid->dispKp;
}
double GPIDGetKi(G_PID *pid)
{
    return pid->dispKi;
}
double GPIDGetKd(G_PID *pid)
{
    return pid->dispKd;
}
int GPIDGetMode(G_PID *pid)
{
    return pid->inAuto;
}
int GPIDGetDirection(G_PID *pid)
{
    return pid->controllerDirection;
}