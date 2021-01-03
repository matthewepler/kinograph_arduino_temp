/********************************************************
   Kinograph - Control code for the Kinograph hardware

   v2.0
   Matthew Epler, 2020

   RIGHTS GO HERE

   TODO
   ----
   - calibration mode (see DueFlashStorage lib examples)
     - check EEPROM
     - blink if empty ("CLBT" "LEFT")
     - on move, display values
     - 3x on each switch
     - blink "CLBT RIGHT"
     - repeat
     - "ALL" "SET!"
     - set targets at equal %
     
    - If limit switch is hit and engage switch is up,
      don't restart startup automatically. Blink error and 
      require toggle to clear state and re-start startup routine. 
 ********************************************************/

#include "defaults.h"

void setup() {
  Serial.begin(9600);
 
  // Init limit switches
  pinMode(hubLeft_minSwitchPin, INPUT_PULLUP);
  pinMode(hubLeft_maxSwitchPin, INPUT_PULLUP);
  pinMode(hubRight_minSwitchPin, INPUT_PULLUP);
  pinMode(hubRight_maxSwitchPin, INPUT_PULLUP);

  // Init supply (left) motor
  pinMode(hubLeft_potPin, INPUT);
  pinMode(motorLeft_pwm, OUTPUT);
  pinMode(motorLeft_dir, OUTPUT);
  digitalWrite(motorLeft_dir, LOW);

  // Init takeup (right) motor
  pinMode(hubRight_potPin, INPUT);
  pinMode(motorRight_pwm, OUTPUT);
  pinMode(motorRight_dir, OUTPUT);
  digitalWrite(motorRight_dir, HIGH); // HIGH == counter-clockwise
 
  // Init Control Panel
  pinMode(control_scanningSwitchPin, INPUT_PULLUP);
  attachInterrupt(control_scanningSwitchPin, readControls, CHANGE);
  num_display.blinkRate(0);
  num_display.begin(0x70);

  killAllNow();
}

void loop() {
//  printLimitSwitchValues();
//  printHubValues();
//  printPIDValues();
//  printControlPanelValues();

  // if the scanning switch is ON...
  if (scanning) { 
    if (!startup) startUp();
    updateTension();
  } else {
    killAllNow();
  }
}

void updateTension() {
  readHubs();
  
  PIDLeft_input = (int) map(hubLeft_potVal, hubLeft_potMinVal, hubLeft_potMaxVal, 0, 100);
  PIDRight_input = (int) map(hubRight_potVal, hubRight_potMaxVal, hubRight_potMinVal, 100, 0);

  // target is set in default settings, and reset by initPIDs() (which is called startUp and killAllNow).
  PIDLeft.Compute();
  PIDRight.Compute();                               

//  Serial.print("PIDRight_output");
//  Serial.println(PIDRight_output);
//  Serial.print("PIDLeft_output");
//  Serial.println(PIDLeft_output);
//  Serial.println();
  
  analogWrite(motorLeft_pwm, constrain(PIDLeft_output, 0, 100));
  analogWrite(motorRight_pwm, constrain(PIDRight_output, 0, 100));
}

void readHubs() {
  hubLeft_minSwitchVal = digitalRead(hubLeft_minSwitchPin);
  hubLeft_maxSwitchVal = digitalRead(hubLeft_maxSwitchPin);
  hubRight_minSwitchVal = digitalRead(hubRight_minSwitchPin);
  hubRight_maxSwitchVal = digitalRead(hubRight_maxSwitchPin);

  int noTension = hubRight_minSwitchVal == LOW || hubLeft_minSwitchVal == LOW;
  int tensionMax = hubRight_maxSwitchVal == LOW || hubLeft_maxSwitchVal == LOW;
  
  if (startup && noTension) {
    alert("WOAH. We lost tension!", true);
    return;
  } 
 
  if (startup && tensionMax) {
    alert("WOAH. Max tension reached!", true);
    return;
  }
  
  hubLeft_potVal = analogRead(hubLeft_potPin);
  hubRight_potVal = analogRead(hubRight_potPin);
}

void readControls() {
  scanning = digitalRead(control_scanningSwitchPin) == HIGH ? true : false;
  
  // the switch has been toggled, release the hold so we can try startup again. 
  if (scanning && holding) {
    holding = false;
  }
}

void startUp() {
  alert("Starting up...", false);
  
  num_display.print(00.00);
  num_display.blinkRate(1);
  num_display.writeDisplay();

  initPIDs();

  boolean leftReady = false;
  boolean rightReady = false;

  while (!leftReady || !rightReady) {
 
    if (!scanning) {
      alert("Startup interrupted. Someone flipped the switch.", true);
      num_display.blinkRate(0);
      return;
    }

    readHubs();
    
    if (!leftReady && PIDLeft_target < hubLeft_targetPotValue) {
      PIDLeft_target += 0.1;
    } else {
      leftReady = true;
    }
    
    if (!rightReady && PIDRight_target < hubRight_targetPotValue) {
      PIDRight_target += 0.1;
    } else {
      rightReady = true;
    }

    updateTension();
  }
  
  if (hubLeft_minSwitchVal == HIGH && hubRight_minSwitchVal == HIGH) {
    startup = true;
    num_display.blinkRate(0);
  } else {
    alert("Startup complete but the hub reports a limit switch is still closed.", false);
    // let's hold off on trying to startup right away. Force user to toggle switch.
    holding = true;
  }
}

void killAllNow() {
  scanning = false;
  startup = false;
  initPIDs();
  killMotors();
  holding = true;
}

void initPIDs() {
  PIDLeft.SetMode(AUTOMATIC);
  PIDRight.SetMode(AUTOMATIC);
  PIDLeft_output = 0;
  PIDRight_output = 0;
  PIDLeft_input = 0;
  PIDRight_input = 0;
  PIDLeft_target = 0;
  PIDRight_target = 0;
}

void killMotors() {
  analogWrite(motorLeft_pwm, 0);
  analogWrite(motorRight_pwm, 0);
}

void alert(String message, boolean kill) {
  Serial.print(message);
  if (kill) killAllNow();
}


// ---------------- HELPER + DEBUGGING FUNCTIONS -----------------

void printLimitSwitchValues() {
  Serial.println();
  Serial.println("LIMIT SWITCH VALUES");
  Serial.println("====================");
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
  Serial.println();
  Serial.println("HUB VALUES");
  Serial.println("====================");
  Serial.print(analogRead(hubLeft_potPin));
  Serial.print(" : " );
  Serial.print(analogRead(hubRight_potPin));
  Serial.println();
}

void printPIDValues() {
  Serial.println();
  Serial.print("PID VALUES (L:R)(input, target, output) => ");
  Serial.print(PIDLeft_input);
  Serial.print(" ");
  Serial.print(PIDLeft_target);
  Serial.print(" ");
  Serial.print(PIDLeft_output);
  Serial.print(" : ");
  Serial.print(PIDRight_input);
  Serial.print(" ");
  Serial.print(PIDRight_target);
  Serial.print(" ");
  Serial.print(PIDRight_output);
  Serial.println();
}

void printControlPanelValues() {
  Serial.println();
  Serial.println("CONTROL PANEL VALUES");
  Serial.println("====================");
  Serial.print("pot: ");
  Serial.print(analogRead(control_potPin));
  Serial.print("\t goSwitch:");
  Serial.print(digitalRead(control_scanningSwitchPin));
  Serial.println();
}
