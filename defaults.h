

/********************************************************
 * Kinograph - Default variables
 * 
 * v2.0
 * Matthew Epler, 2020
 * 
 * RIGHTS GO HERE
 * 
 * Libraries used:
 * PID: https://playground.arduino.cc/Code/PIDLibrary/ 
 * LED Backpack: TODO
 * DUE Flash Storage: https://github.com/sebnil/DueFlashStorage
 ********************************************************/

#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.

// control panel
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

// tension hub
#include <PID_v1.h>

// - machine controls -
const int control_scanningSwitchPin = 9;
const int control_dirSwitchPin = 10;
const int control_potPin = A0;
boolean startup = false;
boolean scanning = false;
boolean holding = false;
int control_potVal;
Adafruit_7segment num_display = Adafruit_7segment();

// - motor hub left
const int hubLeft_minSwitchPin = 40; // ls_left
const int hubLeft_maxSwitchPin = 42; // ls_right
const int hubLeft_potPin = A7;
const int hubLeft_potMinVal = 379;
const int hubLeft_potMaxVal = 742;
int hubLeft_targetPotValue = 15; // 0 - 100
boolean hubLeft_minSwitchVal;
boolean hubLeft_maxSwitchVal;
int hubLeft_potVal;

// - motor hub right
const int hubRight_minSwitchPin = 3; // ls_left
const int hubRight_maxSwitchPin = 36; // ls_right
const int hubRight_potPin = A6;
const int hubRight_potMinVal = 669;
const int hubRight_potMaxVal = 308;
int hubRight_targetPotValue = 15; // 0-100
boolean hubRight_minSwitchVal;
boolean hubRight_maxSwitchVal;
int hubRight_potVal;

// — motor drivers —
const int motorLeft_dir = 50;
const int motorLeft_pwm = 11;
const int motorRight_dir = 16;
const int motorRight_pwm = 8;

// PID variables 
boolean PIDsRunning = false;
double Kp = 0.5;
double Ki = 0.1;
double Kd = 0.05;
double PIDLeft_target, PIDLeft_input, PIDLeft_output;
double PIDRight_target, PIDRight_input, PIDRight_output;
PID PIDLeft(&PIDLeft_input, &PIDLeft_output, &PIDLeft_target, Kp, Ki, Kd, DIRECT);
PID PIDRight(&PIDRight_input, &PIDRight_output, &PIDRight_target, Kp, Ki, Kd, DIRECT);
