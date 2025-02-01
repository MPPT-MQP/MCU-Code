/*
   Arduino PID Library - Version 1.2.1
   by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
   Ported to the Raspberry Pi Pico by Samyar Sadat Akhavi.

   Original library: https://github.com/br3ttb/Arduino-PID-Library
   Forked version for Pico: https://github.com/samyarsadat/Pico-PID-Library

   This Library is licensed under the MIT License.
*/

// Constants used in some of the functions below
#define AUTOMATIC  1
#define MANUAL	   0
#define DIRECT     0
#define REVERSE    1
#define P_ON_M     0
#define P_ON_E     1

//Initialize Variables
float dispKp;				 // * we'll hold on to the tuning parameters in user-entered 
float dispKi;				 //   format for display purposes
float dispKd;

float kp;                    // * (P)roportional Tuning Parameter
float ki;                    // * (I)ntegral Tuning Parameter
float kd;                    // * (D)erivative Tuning Parameter

int controllerDirection;
int pOn;

float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
float *myOutput;             //   This creates a hard link between the variables and the 
float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                //   what these values are.  with pointers we'll just know.
        
unsigned long lastTime;
float outputSum, lastInput;

unsigned long SampleTime;
float outMin, outMax;
bool inAuto, pOnE;

// *********** commonly used functions ***********
void pid_controller(float*, float*, float*, float, float, float, int, int);    // * constructor.  links the PID to the Input, Output, and 
                                                                //   Setpoint.  Initial tuning parameters are also set here.
                                                                //   (overload for specifying proportional mode)
void set_mode(int Mode);                                        // * sets PID to either Manual (0) or Auto (non-0)

bool compute();                                                // * performs the PID calculation.  it should be
                                                                //   called every time the main program loop cycles. ON/OFF and
                                                                //   calculation frequency can be set using SetMode
                                                                //   SetSampleTime respectively

void set_output_limits(float, float);                            // * clamps the output to a specific range. 0-65535 by default, but
                                                                //   it's likely the user will want to change this depending on
                                                                //   the application


// *********** available but not commonly used functions **********
void set_tunings(float, float, float, int);   // * overload for specifying proportional mode         	  

void set_controller_direction(int);	         // * Sets the Direction, or "Action" of the controller. DIRECT
                                                //   means the output will increase when error is positive. REVERSE
                                                //   means the opposite.  it's very unlikely that this will be needed
                                                //   once it is set in the constructor.

void set_sample_time(int);                     // * sets the frequency, in Milliseconds, with which 
                                                //   the PID calculation is performed.  default is 100
                                        
                                                
// *********** Display functions ***********
float get_kp();				 //  These functions query the pid for interal values.
float get_ki();				 //  they were created mainly for the pid front-end,
float get_kd();				 //  where it's important to know what is actually 
int get_mode();			     //  inside the PID.
int get_direction();					 

void pid_initialize();
        
