/*
  Arduino PID Library - Version 1.2.1
  by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
  Ported to the Raspberry Pi Pico by Samyar Sadat Akhavi.

  Original library: https://github.com/br3ttb/Arduino-PID-Library
  Forked version for Pico: https://github.com/samyarsadat/Pico-PID-Library

  This Library is licensed under the MIT License.
*/

#include "pico/stdlib.h"
#include <stdbool.h>

// Constants used in some of the functions below
#define AUTOMATIC  1
#define MANUAL     0
#define DIRECT     0
#define REVERSE    1
#define P_ON_M     0
#define P_ON_E     1

typedef struct {
    // Pointers to Input, Output, Setpoint
    float *myInput;
    float *myOutput;
    float *mySetpoint;
    
    // PID tuning parameters
    float kp;
    float ki;
    float kd;
    
    // Display tuning parameters
    float dispKp;
    float dispKi;
    float dispKd;
    
    // Controller direction
    int controllerDirection;
    
    // POn enum
    int pOn;
    bool pOnE;
    
    // Output limits
    float outMin;
    float outMax;
    
    // Sample time
    unsigned long SampleTime;
    
    // Derived PID variables
    float outputSum;
    float lastInput;
    
    // Mode
    bool inAuto;
    
    // Timing
    unsigned long lastTime;
} PID;

// Function prototypes
void PID_Init(PID *pid, float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, int POn, int ControllerDirection);
void PID_Init_default(PID *pid, float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, int ControllerDirection);
bool PID_Compute(PID *pid);
void PID_SetTunings(PID *pid, float Kp, float Ki, float Kd, int POn);
void PID_SetTunings_basic(PID *pid, float Kp, float Ki, float Kd);
void PID_SetSampleTime(PID *pid, int NewSampleTime);
void PID_SetOutputLimits(PID *pid, float Min, float Max);
void PID_SetMode(PID *pid, int Mode);
void PID_Initialize(PID *pid);
void PID_SetControllerDirection(PID *pid, int Direction);
float PID_GetKp(PID *pid);
float PID_GetKi(PID *pid);
float PID_GetKd(PID *pid);
int PID_GetMode(PID *pid);
int PID_GetDirection(PID *pid);

/** Constructor (...) ******************************************************
 *  The parameters specified here are those for for which we can't set up  
 *  reliable defaults, so we need to have the user set them.               
 ***************************************************************************/
void PID_Init(PID *pid, float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, int POn, int ControllerDirection)
{
    pid->myOutput = Output;
    pid->myInput = Input;
    pid->mySetpoint = Setpoint;
    pid->inAuto = false;

    PID_SetOutputLimits(pid, 0, 65535);  // default output limit corresponds to
                                         // the pico's pwm limits

    pid->SampleTime = 100;                // default Controller Sample Time is 0.1 seconds

    PID_SetControllerDirection(pid, ControllerDirection);
    PID_SetTunings(pid, Kp, Ki, Kd, POn);

    pid->lastTime = to_ms_since_boot(get_absolute_time()) - pid->SampleTime;
}


/** Constructor (...) ********************************************************
 *  To allow backwards compatability for v1.1, or for people that just want  
 *  to use Proportional on Error without explicitly saying so                
 *****************************************************************************/
void PID_Init_default(PID *pid, float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, int ControllerDirection)
{
    PID_Init(pid, Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection);
}


/** Compute() ************************************************************************
 *  This, as they say, is where the magic happens.  this function should be called         
 *  every time the main program loop executes.  the function will decide for itself
 *  whether a new pid Output needs to be computed.  returns true when the output is            
 *  computed, false when nothing has been done.                                                      
 *************************************************************************************/
bool PID_Compute(PID *pid)
{
    if (!pid->inAuto)
        return false;
    
    unsigned long now = to_ms_since_boot(get_absolute_time());
    unsigned long timeChange = (now - pid->lastTime);

    if (timeChange >= pid->SampleTime)
    {
        /* Compute all the working error variables */
        float input = *(pid->myInput);
        float error = *(pid->mySetpoint) - input;
        float dInput = (input - pid->lastInput);
        pid->outputSum += (pid->ki * error);

        /* Add Proportional on Measurement, if P_ON_M is specified */
        if (!pid->pOnE)
            pid->outputSum -= pid->kp * dInput;

        if (pid->outputSum > pid->outMax)
            pid->outputSum = pid->outMax;

        else if (pid->outputSum < pid->outMin)
            pid->outputSum = pid->outMin;

        /* Add Proportional on Error, if P_ON_E is specified */
        float output;
        
        if (pid->pOnE)
            output = pid->kp * error;
        
        else
            output = 0;

        /* Compute Rest of PID Output */
        output += pid->outputSum - pid->kd * dInput;

        if (output > pid->outMax)
            output = pid->outMax;

        else if (output < pid->outMin)
            output = pid->outMin;

        *(pid->myOutput) = output;

        /* Remember some variables for next time */
        pid->lastInput = input;
        pid->lastTime = now;
        return true;
    }

    else
        return false;
}


/** SetTunings(...) ************************************************************
 *  This function allows the controller's dynamic performance to be adjusted.  
 *  it's called automatically from the constructor, but tunings can also
 *  be adjusted on the fly during normal operation
 *******************************************************************************/
void PID_SetTunings(PID *pid, float Kp, float Ki, float Kd, int POn)
{
    if (Kp < 0 || Ki < 0 || Kd < 0)
        return;

    pid->pOn = POn;
    pid->pOnE = POn == P_ON_E;

    pid->dispKp = Kp;
    pid->dispKi = Ki;
    pid->dispKd = Kd;

    float SampleTimeInSec = ((float)pid->SampleTime) / 1000;
    pid->kp = Kp;
    pid->ki = Ki * SampleTimeInSec;
    pid->kd = Kd / SampleTimeInSec;

    if (pid->controllerDirection == REVERSE)
    {
        pid->kp = (0 - pid->kp);
        pid->ki = (0 - pid->ki);
        pid->kd = (0 - pid->kd);
    }
}


/** SetTunings(...) **********************************
 *  Set Tunings using the last-remembered POn setting  
 *****************************************************/
void PID_SetTunings_basic(PID *pid, float Kp, float Ki, float Kd)
{
    PID_SetTunings(pid, Kp, Ki, Kd, pid->pOn);
}


/** SetSampleTime(...) *******************************************************
 *  sets the period, in Milliseconds, at which the calculation is performed
 *****************************************************************************/
void PID_SetSampleTime(PID *pid, int NewSampleTime)
{
    if (NewSampleTime > 0)
    {
        float ratio = (float)NewSampleTime / (float)pid->SampleTime;

        pid->ki *= ratio;
        pid->kd /= ratio;

        pid->SampleTime = (unsigned long)NewSampleTime;
    }
}


/** SetOutputLimits(...) **********************************************************
 *  This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range  
 *  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **********************************************************************************/
void PID_SetOutputLimits(PID *pid, float Min, float Max)
{
    if (Min >= Max)
        return;

    pid->outMin = Min;
    pid->outMax = Max;

    if (pid->inAuto)
    {
        if (*(pid->myOutput) > pid->outMax)
            *(pid->myOutput) = pid->outMax;

        else if (*(pid->myOutput) < pid->outMin)
            *(pid->myOutput) = pid->outMin;

        if (pid->outputSum > pid->outMax)
            pid->outputSum = pid->outMax;

        else if (pid->outputSum < pid->outMin)
            pid->outputSum = pid->outMin;
    }
}


/** SetMode(...) ****************************************************************
 *  Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 *  when the transition from manual to auto occurs, the controller is
 *  automatically initialized
 ********************************************************************************/
void PID_SetMode(PID *pid, int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    
    if (newAuto && !pid->inAuto)
    { 
        /* we just went from manual to auto */
        PID_Initialize(pid);
    }

    pid->inAuto = newAuto;
}


/** Initialize() ***********************************************************
 *  does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ***************************************************************************/
void PID_Initialize(PID *pid)
{
    pid->outputSum = *(pid->myOutput);
    pid->lastInput = *(pid->myInput);

    if (pid->outputSum > pid->outMax)
        pid->outputSum = pid->outMax;

    else if (pid->outputSum < pid->outMin)
        pid->outputSum = pid->outMin;
}


/** SetControllerDirection(...) ***************************************************
 *  The PID will either be connected to a DIRECT acting process (+Output leads
 *  to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 *  know which one, because otherwise we may increase the output when we should
 *  be decreasing.  This is called from the constructor.
 **********************************************************************************/
void PID_SetControllerDirection(PID *pid, int Direction)
{
    if (pid->inAuto && Direction != pid->controllerDirection)
    {
        pid->kp = (0 - pid->kp);
        pid->ki = (0 - pid->ki);
        pid->kd = (0 - pid->kd);
    }

    pid->controllerDirection = Direction;
}


/** Status Functions ***********************************************************
 *  Just because you set the Kp=-1 doesn't mean it actually happened.  these
 *  functions query the internal state of the PID.  they're here for display
 *  purposes.  these are the functions the PID Front-end uses for example
 ******************************************************************************/
float PID_GetKp(PID *pid)       { return pid->dispKp; }
float PID_GetKi(PID *pid)       { return pid->dispKi; }
float PID_GetKd(PID *pid)       { return pid->dispKd; }
int PID_GetMode(PID *pid)       { return pid->inAuto ? AUTOMATIC : MANUAL; }
int PID_GetDirection(PID *pid)  { return pid->controllerDirection; }

