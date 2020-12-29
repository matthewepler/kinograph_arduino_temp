/********************************************************
   Kinograph - Control code for the Kinograph hardware

   v2.0
   Matthew Epler, 2020

   RIGHTS GO HERE
 ********************************************************/

#include "defaults.h"


void setup()
{
  Serial.begin(9600);
 
  // Initialize limit switches
  pinMode(hubLeft_minSwitchPin, INPUT_PULLUP);
  pinMode(hubLeft_maxSwitchPin, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(hubLeft_minSwitchPin), handleEngagedInterrupt, LOW);
//  attachInterrupt(digitalPinToInterrupt(hubLeft_maxSwitchPin), killAllNow, LOW);
  pinMode(hubRight_minSwitchPin, INPUT_PULLUP);
  pinMode(hubRight_maxSwitchPin, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(hubRight_minSwitchPin), handleEngagedInterrupt, LOW);
//  attachInterrupt(digitalPinToInterrupt(hubRight_maxSwitchPin), killAllNow, LOW);

  // Initialize supply (left) motor
  pinMode(hubLeft_potPin, INPUT);
  pinMode(motorLeft_pwm, OUTPUT);
  pinMode(motorLeft_dir, OUTPUT);
  digitalWrite(motorLeft_dir, LOW);

  // Initialize takeup (right) motor
  pinMode(hubRight_potPin, INPUT);
  pinMode(motorRight_pwm, OUTPUT);
  pinMode(motorRight_dir, OUTPUT);
  digitalWrite(motorRight_dir, HIGH); // HIGH == counter-clockwise

  // initialize PID variables
  PIDLeft_target = hubLeft_targetPotValue;
  PIDRight_target = hubRight_targetPotValue;

  // default PIDs to "off"
  PIDLeft.SetMode(MANUAL);
  PIDRight.SetMode(MANUAL);

  // Dashboard Hardware & Control
  pinMode(control_goSwitchPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(control_goSwitchPin), killAllNow, RISING);
  num_display.begin(0x70);
}

void loop() {
  // --- debugging printers ---
//  printLimitSwitchValues();
  printHubValues();
//  printPIDValues();
//  printControlPanelValues();

  updateMachineStatus();
  updateTension();
  updateSpeed();
}

void updateTension() {
  
  
  hubLeft_potVal = analogRead(hubLeft_potPin);
  hubRight_potVal = analogRead(hubRight_potPin);

  PIDLeft_input = hubLeft_potVal;
  PIDRight_input = hubRight_potVal;

  PIDLeft.Compute();
  PIDRight.Compute();

  if (!PIDsRunning || !startup) return; // refactor
  analogWrite(motorLeft_pwm, PIDLeft_output);
  analogWrite(motorRight_pwm, PIDRight_output);
}

void updateSpeed() {  
  // this is all temp until we get fps 
  
  control_potVal = analogRead(control_potPin);
  int _speed = int(map(control_potVal, 0, 1023, 0, 150));

  num_display.print(_speed);
  num_display.writeDisplay();

  if (!PIDsRunning || !startup) return; // refactor
  analogWrite(motorRight_pwm, _speed);
}

void updateMachineStatus() {
  
  readHubSwitches();
  readGoSwitch();

  if (!machineOn) {
    killAllNow();
  }
  
  if (machineOn && !startup) {
    killAllNow();
    startUp();
  }
}

void readHubSwitches() {
  
  hubLeft_minSwitchVal = digitalRead(hubLeft_minSwitchPin);
  hubLeft_maxSwitchVal = digitalRead(hubLeft_maxSwitchPin);
  hubRight_minSwitchVal = digitalRead(hubRight_minSwitchPin);
  hubRight_maxSwitchVal = digitalRead(hubRight_maxSwitchPin);
}


void readGoSwitch() {
  machineOn = digitalRead(control_goSwitchPin);
}

void handleEngagedInterrupt() {
  if (startup) {
    startup = false;
    killAllNow();
  }
}

void startUp() {
  Serial.println("starting up...");
  if (!hubLeft_maxSwitchPin || !hubRight_maxSwitchPin) return;

  float l_startupSpeed = 0;
  float r_startupSpeed = 0;
  
  while (hubLeft_minSwitchVal == LOW || hubRight_minSwitchVal == LOW) {
        
    if (!machineOn) break;
    readHubSwitches();
    readGoSwitch();

    if (hubLeft_minSwitchVal == LOW) {
      analogWrite(motorLeft_pwm, l_startupSpeed);
      l_startupSpeed += 0.01;
    }
    if (hubRight_minSwitchVal == LOW) {
      analogWrite(motorRight_pwm, r_startupSpeed);
      r_startupSpeed += 0.01;
    }
  }
  
  startup = true;
  initPIDs();
}

void killAllNow() {
  startup = false;
  PIDsRunning = false;
  killMotors();
}

void initPIDs() {
  PIDLeft.SetMode(AUTOMATIC);
  PIDRight.SetMode(AUTOMATIC);
  PIDsRunning = true;
}

void killMotors() {
  analogWrite(motorLeft_pwm, 0);
  analogWrite(motorRight_pwm, 0);
}


// ---------------- HELPER + DEBUGGING FUNCTIONS -----------------

void printLimitSwitchValues() {
  Serial.print(hubLeft_minSwitchVal);  // 0 at rest
  Serial.print(" ");
  Serial.print(hubLeft_maxSwitchVal);  // 1 at rest
  Serial.print(" ");
  Serial.print(hubRight_minSwitchVal); // 0 at rest
  Serial.print(" ");
  Serial.print(hubRight_maxSwitchVal); // 1 at rest
  Serial.println();
}

void printHubValues() {
  Serial.print(analogRead(hubLeft_potPin));
  Serial.print(" : " );
  Serial.print(analogRead(hubRight_potPin));
  Serial.println();
}

void printPIDValues() {
  Serial.print(PIDLeft_input);
  Serial.print(" ");
  Serial.print(PIDLeft_target);
  Serial.print(" ");
  Serial.print(PIDLeft_output);

  Serial.print(" ");

  Serial.print(PIDRight_input);
  Serial.print(" ");
  Serial.print(PIDRight_target);
  Serial.print(" ");
  Serial.print(PIDRight_output);

  Serial.println();
}

void printControlPanelValues() {
  Serial.print("pot: ");
  Serial.print(analogRead(control_potPin));
  Serial.print("\t goSwitch:");
  Serial.print(digitalRead(control_goSwitchPin));
  Serial.println();
}
