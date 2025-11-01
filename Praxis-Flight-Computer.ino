/************************************************************************************************/
/*======================= PRAXIS FLIGHT COMPUTER & STABILITY CONTROL SYSTEM ====================*/
/************************************************************************************************/

//PINS
const int LED_PIN = 3;
const int BUZZER_PIN = 4;
const int BUTTON_PIN = 2;
const int csPIN = 10;                           //SPI chip select pin
const int xServo_1_PIN = 9;
const int xServo_2_PIN = 8;
const int yServo_1_PIN = 7;
const int yServo_2_PIN = 6;

//TIMERS
const unsigned long logTime = 10;               // HOW OFTEN TO LOG DATA (ms) - 100 HZ
const unsigned long buttonDelay = 2000;         //safety delay on button press (ms)
const unsigned long stopDelay = 5000;           //timer before stopping system after landing

//STATE DETECTION PHYSICAL PARAMETERS
int detectLaunchAlt=10, detectLaunchG=2;        //G's and ALTITUDE (M) TO DETECT LIFTOFF
int detectLandAlt=5,detectLandSpeed=5;          //altitude AND VELOCITY to detect landing
float burnoutSpeed = 1.2;                       //gs to detect burnout

//PID CONTROL & SERVO VARIABLES
double pidPitchTarget = 0;                      // target angle (vertical)
double pidRollTarget = 0;                       // target angle (vertical)
double Kp=2, Ki=5, Kd=1;                        //INITIAL TUNING PARAMETERS
int minPitchAngle=-15,maxPitchAngle=15;         //CONSTRAINED PITCH LIMITS
int minRollAngle=-15,maxRollAngle=15;           //CONSTRAINED ROLL LIMITS
float servoDeadband = 1;                        //how much tilt (deg) for servos to actuate

//KALMAN FILTER PARAMETERS
float e_mea = 0.3;                              // Measurement noise
float e_est = 0.5;                              // Estimation error
float q     = 0.01;                             // Process noise
float noiseThresh = 0.2;                        //noise removal threshold for kalman filter

/*======================= STRUCTS & ENUMS ====================*/
//NET ACCELERATION CALCULATION
struct AccelData {
  float x,y,z;
  float magnitude;
};
AccelData filtAccel;  //filtered acceleration

struct GyroData {
  float x,y,z;
}


//ORIENTATION CALCULATION
struct Orientation {
  double pitch;
  double roll;
  double yaw;
};
Orientation heading;  //heading 

// Struct to store velocity components
struct Velocity {
  float x;
  float y;
  float z;
  float magnitude;
};
Velocity velocity = {0, 0, 0, 0};   //velocity

//ARMING STATE MACHINE
enum class ArmState {on,initialised,armed,locked};
ArmState armState = ArmState::on;

//FLIGHT STATE MACHINE
enum class FlightState {ground,burn,coast,apogee,descent,landed};
FlightState flightState = FlightState::ground;

/*======================= LIBRARIES ====================*/
//SD card
#include <SPI.h>
#include <SD.h>

//BMP280 barometer
#include <Wire.h>
#include <Adafruit_BMP280.h>
#define BMP280_ADDRESS 0x76                //I2C ADDRESS
Adafruit_BMP280 bmp; 
Adafruit_Sensor *bmp_temp;                 //POINTERS for using bmp temp in similar format to accelerometer
Adafruit_Sensor *bmp_pressure;

/*
//MMA8451 3 axis accelerometer
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#define MMA8451_ADDRESS 0x1D                ////////VERIFY I2C ADDRESS - could be 0x1C////////
Adafruit_MMA8451 mma = Adafruit_MMA8451();
*/

///////////////////////////////////////////////////////////////////////////////////////////////////
// MPU6050 3-axis accelerometer and gyro
#include <mpu6050.h>
#define MPU_ADDRESS 0x68  // AD0 pin tied to GND

//setup
wakeSensor(MPU_ADDRESS); // wakes sensor from sleep mode

//initialise
calculateGyroOffset(MPU_ADDRESS, gyroOffsetX, gyroOffsetY, gyroOffsetZ); // provide MPU6050 address and gyroscope values are written to 3 provided variables
calculateAccelOffset(MPU_ADDRESS, accelOffsetX, accelOffsetY);

AccelData mpuAccelRaw;
AccelData mpuAccelGforce;
GyroData mpuGyroRaw;
GyroData mpuGyroDegPS;
Orientation gyroOrientation;
Orientation accelOrientation;

//loop
readAccelData(MPU_ADDRESS, mpuAccelRaw.x, mpuAccelRaw.y, mpuAccelRaw.z); // pass MPU6050 address and gyroscope values are written to 3 provided variables
rawAccelToGForce(mpuAccelRaw.x, mpuAccelRaw.y, mpuAccelRaw.z, mpuAccelGforce.x, mpuAccelGforce.y, mpuAccelGforce.z); // provide the 3 raw gyroscope values and returns them in their dps (degrees per second) values

readGyroData(MPU_ADDRESS, mpuGyroRaw.x, mpuGyroRaw.y, mpuGyroRaw.z); // pass MPU6050 address and accelerometer values are written to 3 provided variables
rawGyroToDPS(mpuGyroRaw.x, mpuGyroRaw.y, mpuGyroRaw.z, mpuGyroDegPS.x, mpuGyroDegPS.y, mpuGyroDegPS.z); // provide the 3 raw accelerometer values and returns them in their g force values

  mpuGyroDegPS.x = mpuGyroDegPS.x - gyroOffsetX; // adjust gyroscope values to compensate for offset values
  mpuGyroDegPS.y = mpuGyroDegPS.y - gyroOffsetY;
  mpuGyroDegPS.z = mpuGyroDegPS.z - gyroOffsetZ;

dpsToAngles(mpuGyroDegPS.x, mpuGyroDegPS.y, mpuGyroDegPS.z, gyroOrientation.pitch, gyroOrientation.roll, gyroOrientation.yaw);
calculateAnglesFromAccel(mpuAccelGforce.x, mpuAccelGforce.y, mpuAccelGforce.z, accelOrientation.pitch, accelOrientation.roll); // uses trigonometry to calculate angles with accelerometer values    // prints mpu6050 values in the terminal
   
  accelOrientation.pitch = accelOrientation.pitch - accelOffsetX; // adjust accelerometer values to compensate for offset values
  accelOrientation.roll = accelOrientation.roll - accelOffsetY;

//start accelerometer and validate connection
void startAccelerometer() {
 /* if (!mma.begin(MMA8451_ADDRESS))  {
    debugLog(F("Could not find a valid MMA8451 sensor"));
    while (1) delay(10); //if no valid sensor it gets stuck
  }
  mma.setRange(MMA8451_RANGE_8_G);    ////set 8g range
  */
  wakeSensor(MPU_ADDRESS); // wakes sensor from sleep mode
  debugLog(F("Accelerometer Initialised"));
}
///////////////////////////////////////////////////////////////////////////////////////////////////
//Buzzer noises and other libraries
#include <CuteBuzzerSounds.h>
#include <math.h>
#include <Arduino.h>

//Servos
#include <Servo.h>
  Servo xServo1;
  Servo xServo2;
  Servo yServo1;
  Servo yServo2;

//kalman filter
#include <SimpleKalmanFilter.h>
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter accelKalmanX(e_mea, e_est, q);
SimpleKalmanFilter accelKalmanY(e_mea, e_est, q);
SimpleKalmanFilter accelKalmanZ(e_mea, e_est, q);

//PID CONTROL
#include <PID_v1.h>
//SERVO TARGET ANGLES
double xAngle;
double yAngle;
PID pitchPID(&heading.pitch, &xAngle, &pidPitchTarget, Kp, Ki, Kd, DIRECT);
PID rollPID(&heading.roll, &yAngle, &pidRollTarget, Kp, Ki, Kd, DIRECT);

/*======================= GLOBAL VARIABLES ====================*/
float initPres;            //initlisation pressure
float groundPres;           //init pres converted to hPa
float initAccel;            //initialistaion net acceleration
bool dataLogging = false;   //TO TURN ON DATALOGGING
int detectLaunchAccel;      //calculated value for liftoff accel
sensors_event_t accelEvent,presEvent,tempEvent; //data recording event
float filtAlt;    //filtered altitude
float speed;    //speed
bool servoLock = 1;   //locks servo actuation
unsigned long flightStart;
unsigned long apogeeTime;
bool ended = 0; //flight ended
bool simulateCheck = 0;

String debugFilePath;       //file path for debug logs
String logFilePath;         //file path for log file on sd card

//calibration values for filtering accel
float initAccelX = 0;
float initAccelY = 0;
float initAccelZ = 0;

//FOR FLASHING LED
bool ledState = 0;
unsigned long flashDelay;

//TIMERS
unsigned long buttonTimer; 
unsigned long buzzerTimer;
unsigned long flashTimer;
unsigned long dataTimer;
unsigned long landTimer;
unsigned long simTimer;

//SPEED CALC
float currentSpeed = 0.0;           //speed in m/s
unsigned long lastSpeedUpdate = 0;  //timestamp of last update

//APOGEE DETECTION
float prevAlt = 0;                     // last filtered altitude
float maxAlt = 0;                      // running maximum altitude
const int apogeeConfirmSamples = 3;    // number of consecutive decreases to confirm
int apogeeCounter = 0;                 // counter of decreases
float apogeeDeadband = 1;            // minimum decrease (meters) to count
bool apogeeDetected = false;           // flag

//debug logging
File debugFile;
File logFile;
unsigned long lastFlushTime = 0;
const unsigned long flushInterval = 500;  // flush every 1s

/*======================= COMMANDS ====================*/
//check serial for commands
void commandCheck() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equalsIgnoreCase("simulate")) {       ///simulate command
      simulate();   
      debugLog(F("Simulate command received"));
    }
    if (command.equalsIgnoreCase("apogee")) {       //apogee cmd - only triggers after simulate
      if (!simulateCheck) {
        debugLog(F("Apogee command not accepted - simulate not triggered"));
      } else  {
        if (millis() - simTimer >= 5000) {
          flightState = FlightState::apogee;
          debugLog(F("Apogee command accepted"));
        }
      }
    }
  }
}

//simulate  - override altitude for testing?
void simulate() {
  detectLaunchAlt = 1;
  detectLaunchG = 0.1;
  simTimer = millis();
  simulateCheck = 1;
}

//in loop to detect burnout after 3s
void simulateBurnout()  {
  if (simulateCheck == 1)  {
    if (millis() - simTimer >= 3000) burnoutSpeed = 0.1;    //burnout after 3s
  }
}

/*======================= MISC OUTPUTS ====================*/
// continuous buzzer beeps for when armed 
void armBuzzer(unsigned long &buzzerTimer) {
  if (millis()-buzzerTimer>=2000)  {
    cute.play(S_MODE1);
    buzzerTimer = millis();
  }
}
// flash led function - takes flashdelay input
void flashLED(unsigned long flashDelay,unsigned long &flashTimer) {
  if (millis()-flashTimer>=flashDelay)  {
    ledState = !ledState;
    digitalWrite(LED_PIN,ledState);
    flashTimer = millis();
  }
}
/*======================= SENSOR DATA / FILTERING / ORIENTATION / VELOCITY ====================*/
//detect apogee
bool detectApogee(float filtAlt,Velocity velocity) {
  float verticalVelocity = velocity.z;    //vertical velocity component

  // Update running maximum
  if (filtAlt > maxAlt) {
      maxAlt = filtAlt;   // if climbing, update max altitude
  } 
  else if ((maxAlt - filtAlt) > apogeeDeadband && verticalVelocity < 0) {
      apogeeCounter++;    // note descending altitude
      if (apogeeCounter >= apogeeConfirmSamples) {    //if multiple descending calls, trigger apogee
          apogeeDetected = true;
          prevAlt = filtAlt;
          return true;
      }
  } 
  else if ((filtAlt - prevAlt) > -0.2) {
    // Slowly decay descent counter instead of full reset
    apogeeCounter = max(0, apogeeCounter - 1);
  }
  prevAlt = filtAlt;  // update for next call
  return false;
}

//calibrate kalman filter for acceleration
void calibrateAccel() {
  const int samples = 100;
  float sumX = 0, sumY = 0, sumZ = 0;
  sensors_event_t event;
  for (int i = 0; i < samples; i++) {
    mma.getEvent(&event);
    sumX += event.acceleration.x;
    sumY += event.acceleration.y;
    sumZ += event.acceleration.z;
    delay(10);
  }
  initAccelX = sumX / samples;
  initAccelY = sumY / samples;
  initAccelZ = sumZ / samples;
}
/*
//Calculate pitch and roll
Orientation getOrientation(AccelData filtAccel)  {
  Orientation oriResult;

  // Pitch = rotation around X-axis
  oriResult.pitch = atan2(filtAccel.x, sqrt(filtAccel.y*filtAccel.y + filtAccel.z*filtAccel.z)) * 180.0 / PI;

  // Roll = rotation around Y-axis
  oriResult.roll  = atan2(filtAccel.y, filtAccel.z) * 180.0 / PI;

  return oriResult;
}
*/
//filter acceleration data
AccelData filterAccel(sensors_event_t accelEvent) {
  // --- Step 1: Remove static sensor bias (from calibration) ---
  float rawX = accelEvent.acceleration.x - initAccelX;
  float rawY = accelEvent.acceleration.y - initAccelY;
  float rawZ = accelEvent.acceleration.z - initAccelZ;

  // --- Step 2: Estimate gravity vector from current orientation ---
  // Convert pitch/roll from degrees to radians
  float pitchRad = heading.pitch * DEG_TO_RAD;
  float rollRad  = heading.roll  * DEG_TO_RAD;

  // These are the gravity components in the sensor frame
  float gX = 9.81 * sin(pitchRad);
  float gY = -9.81 * sin(rollRad);
  float gZ = 9.81 * cos(pitchRad) * cos(rollRad);

  // --- Step 3: Subtract gravity from raw acceleration ---
  float linX = rawX - gX;
  float linY = rawY - gY;
  float linZ = rawZ - gZ;

  // --- Step 4: Zero out very small noise ---
  if (fabs(linX) < noiseThresh) linX = 0;
  if (fabs(linY) < noiseThresh) linY = 0;
  if (fabs(linZ) < noiseThresh) linZ = 0;

  // --- Step 5: Apply Kalman filtering per axis ---
  float filtX = accelKalmanX.updateEstimate(linX);
  float filtY = accelKalmanY.updateEstimate(linY);
  float filtZ = accelKalmanZ.updateEstimate(linZ);

  // --- Step 6: Package filtered results ---
  AccelData result;
  result.x = filtX;
  result.y = filtY;
  result.z = filtZ;
  result.magnitude = sqrt(filtX*filtX + filtY*filtY + filtZ*filtZ);

  return result;
}

//CALCULATE VELOCITY from accel data
Velocity calculateVelocity(AccelData filtAccel) {
  unsigned long now = millis();
  float deltaT;
  if (lastSpeedUpdate == 0) {
    deltaT = 0;
  } else {
    deltaT = (now - lastSpeedUpdate) / 1000.0;
    if (deltaT > 0.1) deltaT = 0.1; // cap
  }
  lastSpeedUpdate = now;

  if (deltaT > 0) {
    velocity.x += filtAccel.x * deltaT;
    velocity.y += filtAccel.y * deltaT;
    velocity.z += filtAccel.z * deltaT;
  }

  //zero out small drift values
  if (fabs(velocity.x) < noiseThresh) velocity.x = 0;
  if (fabs(velocity.y) < noiseThresh) velocity.y = 0;
  if (fabs(velocity.z) < noiseThresh) velocity.z = 0;

  //MAGNITUDE
  velocity.magnitude = sqrt(
    velocity.x * velocity.x +
    velocity.y * velocity.y +
    velocity.z * velocity.z
  );

  return velocity;
}

//update sensors
void sensorEvent()  {
  mma.getEvent(&accelEvent);
  bmp_temp->getEvent(&tempEvent);
  bmp_pressure->getEvent(&presEvent);
}

//kalman filter altitude
float filterAlt() {
  return pressureKalmanFilter.updateEstimate(bmp.readAltitude(groundPres));
}
/*======================= INITIALISATIONS ====================*/
//BUZZER INITIALISE
void startBuzzer()  {
  cute.init(BUZZER_PIN);
  cute.play(S_CONNECTION);
  debugLog(F("Buzzer Initialised"));
}
//Start barometer and validate connection
void startBarometer() {
  unsigned bmpStatus = bmp.begin(BMP280_ADDRESS);
  if (!bmpStatus) {
    debugLog(F("Could not find a valid BMP280 sensor"));
    while (1) delay(10);  // The code will get stuck here, instead of continuing with a bad barometer
  }
  /* Default BMP280 settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  bmp_temp = bmp.getTemperatureSensor();
  bmp_pressure = bmp.getPressureSensor();
  debugLog(F("Barometer Initialised"));
}
 
//SD card init
void startSD()  {
  if (!SD.begin(csPIN)) {
    Serial.println(F("Could not find a valid SD card"));
    while (1) delay(10); //if no valid sd card it gets stuck
  }
  if (!SD.exists("/PRAXIS/LOGS")) SD.mkdir("/PRAXIS/LOGS");
  if (!SD.exists("/PRAXIS/SUMMARY")) SD.mkdir("/PRAXIS/SUMMARY");
  if (!SD.exists("/PRAXIS/DEBUG")) SD.mkdir("/PRAXIS/DEBUG");
  debugLog(F("SD Card Started"));
}

//initialise pins
void startPins()  {
  pinMode(LED_PIN,OUTPUT);
  pinMode(BUTTON_PIN,INPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  debugLog(F("Pins Initialised"));
}

/*======================= SD CARD AND LOGGING BEHAVIOUR ====================*/
//funct to create files to write logs to - takes mode log or summary
String createLogFile(String mode = "LOG") {
  // --- Directory and naming configuration ---
  const char *baseDirLogs = "/PRAXIS/LOGS/";
  const char *baseDirSummary = "/PRAXIS/SUMMARY/";
  const char *prefixLog = "Log_";
  const char *prefixSummary = "Sum_";
  const char *extension = ".txt";

  const char *baseDir;
  const char *prefix;

  // --- Select directory & prefix based on mode ---
  if (mode.equalsIgnoreCase("SUMMARY")) {
    baseDir = baseDirSummary;
    prefix = prefixSummary;
  } else {
    baseDir = baseDirLogs;
    prefix = prefixLog;
  }

  // --- Ensure base directory exists ---
  if (!SD.exists(baseDir)) {
    Serial.print(F("Directory not found, creating: "));
    Serial.println(baseDir);
    SD.mkdir(baseDir);
  }

  // --- Find next available file number ---
  int fileNum = 1;
  String filePath;
  File testFile;

  //recursively check incrementing values until a file doesnt exist
  while (true) {
    filePath = String(baseDir) + prefix + String(fileNum) + extension;
    if (!SD.exists(filePath.c_str())) break;  // stop when file doesn’t exist
    fileNum++;
  }

  // --- Create and open the new file ---
  File file = SD.open(filePath.c_str(), FILE_WRITE);
  if (!file) {
    debugLog(F("Error: Could not create file: "));
    debugLog(String(filePath));
    return "";
  }

  // --- Write headers based on mode ---
  if (mode.equalsIgnoreCase("SUMMARY")) {
    file.println(F("Max Altitude (m),Apogee Timestamp (s),Flight Duration (s)"));
  } else {
    file.println(F("Temp (C),Pressure (Pa),Altitude (m),X acceleration (m/s^2),Y acceleration (m/s^2),Z acceleration (m/s^2),Magnitude of Acceleration (m/s^2),Pitch (deg),Roll (deg),Magnitude of Velocity (m/s),X velocity (m/s),Y velocity (m/s),Z velocity (m/s),X Servo Target Angle (deg),Y Servo Target Angle (deg)"));
  }

  // --- Print confirmation ---
  debugLog(F("Created new "));
  debugLog(String(mode)); 
  debugLog(F(" file: "));
  debugLog(String(filePath));

  return filePath;
}
// --- Flash string overload (for F() macro) //////////////////debug log
void debugLog(const __FlashStringHelper *message) {
  unsigned long t = millis();

  // Print to serial
  Serial.print(F("["));
  Serial.print(t);
  Serial.print(F(" ms] "));
  Serial.println(message);

  // Also write to SD card (if available)
  if (debugFile) {
    debugFile.print("[");
    debugFile.print(t);
    debugFile.print(" ms] ");
    debugFile.println(message);
  }
}

// --- Regular String overload (for dynamic text) /////////// debug log---
void debugLog(String message) {
  unsigned long t = millis();

  // Print to serial
  Serial.print("[");
  Serial.print(t);
  Serial.print(" ms] ");
  Serial.println(message);

  // Also write to SD card (if available)
  if (debugFile) {
    debugFile.print("[");
    debugFile.print(t);
    debugFile.print(" ms] ");
    debugFile.println(message);
  }
}

//create debug log file
String createDebugFile() {
  const char *basePath = "/PRAXIS/DEBUG/";
  const char *prefix = "Debug_";
  const char *extension = ".txt";
  int logNum = 1;

  while (true) {
    debugFilePath = String(basePath) + prefix + String(logNum) + extension;
    if (!SD.exists(debugFilePath.c_str())) debugFile = SD.open(debugFilePath.c_str(), FILE_WRITE);
    if (debugFile) {
      debugFile.println(F("=== DEBUG LOG START ==="));
      debugFile.print(F("Created new debug file: "));
      debugFile.println(debugFilePath);
      return debugFilePath;
    }
    logNum++;
  }
}

//OPEN LOG FILE FOR USE
void openLogFile()  {
  logFile = SD.open(logFilePath.c_str(), FILE_WRITE);
  if (!logFile) {
    debugLog(F("Error opening data log file"));
  } else {
    debugLog(F("Data log file opened successfully"));
  }
}
//datalogging function
void dataLog()  {
  if (dataLogging) {
    if (millis()-dataTimer>=logTime)  {

      //write results to sd in csv format
      if (logFile)  {
        String csvLog = String(tempEvent.temperature) + "," +
                        String(presEvent.pressure) + "," +
                        String(filtAlt) + "," +
                        String(filtAccel.x) + "," + 
                        String(filtAccel.y) + "," +
                        String(filtAccel.z) + "," +
                        String(filtAccel.magnitude) + "," +
                        String(heading.pitch) + "," +
                        String(heading.roll) + "," +
                        String(speed) + "," +
                        String(velocity.x) + "," +
                        String(velocity.y) + "," +
                        String(velocity.z) + "," +
                        String(xAngle) + "," +
                        String(yAngle);

        logFile.println(csvLog);
      } else {
        debugLog(F("Error Logging Data"));
      }
      dataTimer = millis();
    }
  }
}
//generate summary file at end of flight
void generateSummary()  {
  String summaryFilePath = createLogFile("SUMMARY");
  // -> /PRAXIS/SUMMARY/Summary_2.txt
  File summaryFile = SD.open(summaryFilePath.c_str(),FILE_WRITE);
  String summaryPrint = String(maxAlt,3) + "," +
                        String(apogeeTime / 1000,3) + "," +
                        String((millis() - flightStart) / 1000,3);
  if (summaryFile)  {
    summaryFile.println(summaryPrint);
    summaryFile.close();
    debugLog(F("Summary File Written"));
  }
}
//flush log buffer to sd card
void flushLogs()  {
  unsigned long now = millis();
  if (now - lastFlushTime >= flushInterval) {
    if (SD.exists("/PRAXIS"))  {
      if (debugFile) debugFile.flush();
      if (logFile) logFile.flush();
      lastFlushTime = now;
    } else {
      debugLog(F("Warning: SD card disconnected!"));
    }
  }
}

//flush debug logs immediately if required
void flushNow() {
  if (debugFile) debugFile.flush();
  if (logFile) logFile.flush();
  lastFlushTime = millis();
}

//end debug logs
void endLogging() {
  if (logFile)  {
    debugLog(F("Data Log file closing"));
    logFile.flush();
    logFile.close();
  }
  if (debugFile) {
    debugLog(F("Debug Log file closing"));
    debugFile.flush();
    debugFile.close();
  }
}
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
  pitchPID.SetOutputLimits(minPitchAngle, maxPitchAngle);
  rollPID.SetOutputLimits(minRollAngle, maxRollAngle);
}

//compute new pid values
void updatePID() {
  //pidDirection();
  pitchPID.Compute();
  rollPID.Compute();
  if (!servoLock) {
    adjustServos();   //only actuate servos if not locked
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
  debugLog(F("Servos Zeroed"));
}

//initialise servos
void startServos()  {
  xServo1.attach(xServo_1_PIN);
  xServo2.attach(xServo_2_PIN);
  yServo1.attach(yServo_1_PIN);
  yServo2.attach(yServo_2_PIN);
  debugLog(F("Servos Initialised"));
}

/*======================= TIMED CALLS ====================*/
//arm system
void systemArm()  {
  lockServos();
  zeroServos();
  cute.play(S_BUTTON_PUSHED);
  dataLogging = true;
  armState = ArmState::armed;
  debugLog(F("Datalogging Started"));
  debugLog(F("System State --> Armed"));
}

//system landed calls
void systemLanded() {
          dataLogging = false;
          debugLog(F("Datalogging Paused"));
          generateSummary();
          armState = ArmState::locked;
          debugLog(F("Sytem State --> Locked"));
          debugLog(F("Flight Complete! <3 "));
          endLogging();
          cute.play(S_HAPPY);
          ended = 1;
}

//initialise on 1st button press 
void initialise() {
  logFilePath = createLogFile();    //create log file with header
  openLogFile();

  //Read pressure anc convert to hPa
  initPres = bmp.readPressure();      
  groundPres = initPres / 100.0;        

  //read net accel and calculate launch accel
  initAccel = filtAccel.magnitude;   
  detectLaunchAccel = initAccel * detectLaunchG;   

  //calibrate acceleration for kalman filtering
  calibrateAccel();

  armState = ArmState::initialised;     
  cute.play(S_BUTTON_PUSHED);
  buttonTimer = millis();
  debugLog(F("System State --> Initialised")); 
}

/*======================= SETUP & LOOP ====================*/

void setup() {
  Serial.begin(9600);
  //startSPI();
  //delay(200);   //allow sd to power up before responding to spi
  startSD();
  createDebugFile();
  startBarometer();
  startAccelerometer();
  startPins();
  startServos();
  startBuzzer();
  zeroServos();
  startPID();
  debugLog(F("System State --> On"));
}


void loop() {
  commandCheck();                           //check for serial command input
  sensorEvent();                            //sensors update
  filtAccel = filterAccel(accelEvent);      //clean acceleration
  heading = getOrientation(filtAccel);      //orientation  
  filtAlt = filterAlt();                    //clean altitude
  velocity = calculateVelocity(filtAccel);  //velocity
  speed = velocity.magnitude;               //speed
  updatePID();                              //update pid controller
  flushLogs();                              //write log buffers to sd card
  simulateBurnout();                        //simulate timer check for burnout 
  dataLog();                                //log data


  //DETECT APOGEE
  if (!apogeeDetected && detectApogee(filtAlt, velocity)) {
    flightState = FlightState::apogee;
    apogeeTime = millis() - flightStart;
    debugLog(F("Apogee detected! Max Altitude: "));
    debugLog(String(maxAlt));
    debugLog(F("Flight State --> Apogee"));
    flushNow();
  }

  // arming state machine
  switch (armState) {

    // on - 1 hz flash, waiting for init
    case ArmState::on:
    flashLED(1000,flashTimer);
    if (digitalRead(BUTTON_PIN) == HIGH) {
      initialise();
      unlockServos();
    }
    break;

    //init - solid, waiting for arm
    case ArmState::initialised:
    digitalWrite(LED_PIN,HIGH);
    if (digitalRead(BUTTON_PIN) == HIGH && millis() - buttonTimer >= buttonDelay) {
      systemArm();
    }
    break;

    case ArmState::armed:
    flashLED(100,flashTimer);
    armBuzzer(buzzerTimer);
    break;

    case ArmState::locked:
    digitalWrite(LED_PIN,HIGH);
    break;

  }

  // flight state machine - only activate if armed
  if (armState == ArmState::armed)  {
    switch (flightState)  {

      case FlightState::ground:
      if (filtAlt >= detectLaunchAlt && filtAccel.magnitude >= detectLaunchAccel)  {
        flightState = FlightState::burn;
        flightStart = millis();
        debugLog(F("Flight State --> Burn"));
      }
      break;

      case FlightState::burn:
      if (filtAccel.magnitude <= initAccel * burnoutSpeed)  { //////DETECT MOTOR BURNOUT
        flightState = FlightState::coast;
        debugLog(F("Flight State --> Coast"));
        unlockServos();
      }
      break;

      case FlightState::coast:
      break;

      case FlightState::apogee:
      //CAN ADD EXTRA STUFF LATER, EJECTION CHARGES ETC
      flightState = FlightState::descent;
      debugLog(F("Flight State --> Descent"));
      break;

      case FlightState::descent:
      lockServos();
      zeroServos();
      if (filtAlt <= detectLandAlt && speed <= detectLandSpeed) {
        landTimer = millis();
        flightState = FlightState::landed;
        debugLog(F("Flight State --> Landed"));
      }
      break;

      case FlightState::landed:
      if (!ended) {
        if (millis() - landTimer >= stopDelay) {
          systemLanded();
        }
      }
      break;
      
    }
  }
}