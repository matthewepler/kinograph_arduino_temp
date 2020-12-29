/********************************************************
 * Kinograph - Default variables
 * 
 * v2.0
 * Matthew Epler, 2020
 * 
 * RIGHTS GO HERE
 ********************************************************/

#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.

// control panel
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

// tension hub
#include <PID_v1.h>

 // - machine controls -
const int control_goSwitchPin = 9;
const int control_dirSwitchPin = 10;
const int control_potPin = A0;
boolean machineOn; 
boolean startup = false;
int control_potVal;
Adafruit_7segment num_display = Adafruit_7segment();

// - motor hub left
const int hubLeft_minSwitchPin = 46; // ls_left
const int hubLeft_maxSwitchPin = 48; // ls_right
const int hubLeft_potPin = A7;
const int hubLeft_targetPotValue = 300;
const int hubLeft_potMinVal = 244;
const int hubLeft_potMaxVal = 486;
boolean hubLeft_minSwitchVal;
boolean hubLeft_maxSwitchVal;
int hubLeft_potVal;

// — motor drivers —
const int motorLeft_dir = 50;
const int motorLeft_pwm = 2;
const int motorRight_dir = 15;
const int motorRight_pwm = 3;

// PID variables 
boolean PIDsRunning = false;
double PIDLeft_target, PIDLeft_input, PIDLeft_output;
PID PIDLeft(&PIDLeft_input, &PIDLeft_output, &PIDLeft_target,0.5, 0.1, 0.05, DIRECT);
