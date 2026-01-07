/*======================= PID CLOSED LOOP CONTROL & SERVO BEHAVIOURS ====================*/

void lockServos() {
  servoLock = 1;
  debugLog(F("Servos Locked"));
}

void unlockServos() {
  servoLock = 0;
  debugLog(F("Servos Unlocked"));
}

//Move mirrored X servos - ONLY MOVE IF ANGLE CHANGE > 1 DEG
void moveXservos(int xAngle)  {
  static int lastX = 90;  // start neutral; persists between calls
  const int servoDeadband = 1; // degrees threshold to avoid jitter

  if (abs(lastX - xAngle) > servoDeadband) {
    xServo1.write(xAngle);
    xServo2.write(180 - xAngle);  // mirrored
    lastX = xAngle;
  }
}

//Move mirrored Y servos
void moveYservos(int yAngle)  {
  static int lastY = 90;  // start neutral; persists between calls
  const int servoDeadband = 1; // degrees threshold to avoid jitter

  if (abs(lastY - yAngle) > servoDeadband) {
    yServo1.write(yAngle);
    yServo2.write(180 - yAngle);  // mirrored
    lastY = yAngle;
  }
}

//start pid controllers and output limits
void startPID() {
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-maxPIDOutput, maxPIDOutput);
  rollPID.SetOutputLimits(-maxPIDOutput, maxPIDOutput);
}


//compute new pid values
void updatePID() {
  pitchInput = heading.pitch;
  rollInput = heading.roll;

  pitchPID.Compute();
  rollPID.Compute();

  xAngle = map(pitchOutput, -maxPIDOutput, maxPIDOutput, minPitchAngle, maxPitchAngle);
  yAngle = map(rollOutput, -maxPIDOutput, maxPIDOutput, minRollAngle, maxRollAngle);

  if (servoLock) {
    zeroServos();
  } else {
    adjustServos();
  }
}

//adjust servos to pid values
void adjustServos() {
  moveXservos(xAngle);
  moveYservos(yAngle); 
}

/* MAY NEED TO FLIP THIS DEPENDING ON INPUT - NEEDS TESTING
void pidDirection() {
  pitchPID.SetControllerDirection(pitchDirection); 
  rollPID.SetControllerDirection(rollDirection); 
}
*/

//ZERO ALL SERVOS TO INITIAL POSITION
void zeroServos() {
  xServo1.write(90);
  xServo2.write(90);
  yServo1.write(90);
  yServo2.write(90);
  //debugLog(F("Servos Zeroed"));
}

//initialise servos
void startServos()  {
  xServo1.attach(xServo_1_PIN);
  xServo2.attach(xServo_2_PIN);
  yServo1.attach(yServo_1_PIN);
  yServo2.attach(yServo_2_PIN);
  debugLog(F("Servos Initialised"));
}
