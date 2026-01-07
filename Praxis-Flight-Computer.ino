/************************************************************************************************/
/*======================= PRAXIS FLIGHT COMPUTER & STABILITY CONTROL SYSTEM ====================*/
/************************************************************************************************/
//debug tools
bool isDebug = 0;                                 //activate for heartbeat + debugging features

//PINS
const int BUTTON_PIN = 2;
const int LED_PIN = 7;
const int BUZZER_PIN = 4;

const int csPIN = 10;                           //SPI chip select pin

const int xServo_1_PIN = 9;
const int xServo_2_PIN = 6;
const int yServo_1_PIN = 5;
const int yServo_2_PIN = 3;

//TIMERS
const unsigned long logTime = 10;               // HOW OFTEN TO LOG DATA (ms) - 100 HZ
const unsigned long buttonDelay = 2000;         //safety delay on button press (ms)
const unsigned long stopDelay = 5000;           //timer before stopping system after landing

//STATE DETECTION PHYSICAL PARAMETERS
int detectLaunchAlt=10, detectLaunchG=2;        //G's and ALTITUDE (M) TO DETECT LIFTOFF
int detectLandAlt=5,detectLandSpeed=5;          //altitude AND VELOCITY to detect landing
float burnoutSpeed = 1.2;                       //gs to detect burnout

//PID CONTROL & SERVO VARIABLES
double pidPitchTarget=0, pidRollTarget=0;       //target angle (vertical)
double Kp=2, Ki=5, Kd=1;                        //INITIAL TUNING PARAMETERS
int minPitchAngle=-15,maxPitchAngle=15;         //CONSTRAINED PITCH LIMITS
int minRollAngle=-15,maxRollAngle=15;           //CONSTRAINED ROLL LIMITS
float servoDeadband = 1;                        //how much tilt (deg) for servos to actuate
const int maxPIDOutput = 30;                    //max internal pid correction - in valueless pid units, not angles

//KALMAN FILTER PARAMETERS
float e_mea = 0.3;                              // Measurement noise
float e_est = 0.5;                              // Estimation error
float q     = 0.01;                             // Process noise
float noiseThresh = 0.2;                        //noise removal threshold for kalman filter

/*======================= STRUCTS & ENUMS ====================*/
//unified sensor struct
struct SensorData {
  float timeSec;
  float tempC;
  float pressure_hPa;
  float altitude;
  float accel[3];
  float gyro[3];
};
SensorData rawData;

//NET ACCELERATION CALCULATION
struct AccelData {
  float x,y,z;
  float magnitude;
};
AccelData filtAccel;  //filtered acceleration

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

//bmp state machine - ensures bmp isnt called while processing
enum BMPState {
  BMP_IDLE,
  BMP_CONVERTING
};

/*======================= LIBRARIES ====================*/
//SD card
#include <SPI.h>
#include <SD.h>

//BMP280 barometer
#include <Wire.h>
#include <Adafruit_BMP280.h>
#define BMP280_ADDRESS 0x76                //I2C ADDRESS
Adafruit_BMP280 bmp; 

///////////////////////////////////////////////////////////////////////////////////////////////////
// MPU6050 3-axis accelerometer and gyro
#include <mpu6050.h>
#define MPU_ADDRESS 0x68  // AD0 pin tied to GND

AccelData mpuAccelRaw;
AccelData mpuAccelGforce;
Orientation accelOrientation;
//calibration values for filtering accel
float initAccelX = 0;
float initAccelY = 0;
float initAccelZ = 0;
/*
struct GyroData {
  float x,y,z;
}

GyroData mpuGyroRaw;
GyroData mpuGyroDegPS;
Orientation gyroOrientation;
calculateGyroOffset(MPU_ADDRESS, gyroOffsetX, gyroOffsetY, gyroOffsetZ); // provide MPU6050 address and gyroscope values are written to 3 provided variables
readGyroData(MPU_ADDRESS, mpuGyroRaw.x, mpuGyroRaw.y, mpuGyroRaw.z); // pass MPU6050 address and accelerometer values are written to 3 provided variables
rawGyroToDPS(mpuGyroRaw.x, mpuGyroRaw.y, mpuGyroRaw.z, mpuGyroDegPS.x, mpuGyroDegPS.y, mpuGyroDegPS.z); // provide the 3 raw accelerometer values and returns them in their g force values

  mpuGyroDegPS.x = mpuGyroDegPS.x - gyroOffsetX; // adjust gyroscope values to compensate for offset values
  mpuGyroDegPS.y = mpuGyroDegPS.y - gyroOffsetY;
  mpuGyroDegPS.z = mpuGyroDegPS.z - gyroOffsetZ;

dpsToAngles(mpuGyroDegPS.x, mpuGyroDegPS.y, mpuGyroDegPS.z, gyroOrientation.pitch, gyroOrientation.roll, gyroOrientation.yaw);
*/

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
double xAngle,yAngle;             //SERVO TARGET ANGLES
double pitchInput, pitchOutput;   //Input variables for pid
double rollInput, rollOutput;     //outputs for pid
PID pitchPID(&pitchInput, &pitchOutput, &pidPitchTarget, Kp, Ki, Kd, DIRECT);
PID rollPID(&rollInput, &rollOutput, &pidRollTarget, Kp, Ki, Kd, DIRECT);

/*======================= GLOBAL VARIABLES ====================*/
//float initPres;            //initlisation pressure
float groundPres;           //init pres converted to hPa
float initAccel;            //initialistaion net acceleration
bool dataLogging = false;   //TO TURN ON DATALOGGING
int detectLaunchAccel;      //calculated value for liftoff accel
float filtAlt;    //filtered altitude
float speed;    //speed
bool servoLock = 1;   //locks servo actuation
unsigned long flightStart;
unsigned long apogeeTime;
bool ended = 0; //flight ended
bool simulateCheck = 0;
bool apogeeCheck =0;

String debugFilePath;       //file path for debug logs
String logFilePath;         //file path for log file on sd card
String servoFilePath;       //file path for servo logs

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
unsigned long apogeeTimer;

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
File servoFile;
unsigned long lastFlushTime = 0;
const unsigned long flushInterval = 500;  // flush every 0.5s

//restart sd card if failed
unsigned long lastRestartAttempt = 0;
const unsigned long restartCooldown = 1000; // 1 sec

unsigned long lastBMPTrigger = 0;


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
/*======================= APOGEE / VELOCITY / CALCULATIONS ====================*/
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

void ifApogee() {
  //DETECT APOGEE
  if (!apogeeDetected && detectApogee(filtAlt, velocity)) {
    flightState = FlightState::apogee;
    apogeeTime = millis() - flightStart;
    debugLog(F("Apogee detected! Max Altitude: "));
    debugLog(String(maxAlt));
    debugLog(F("Flight State --> Apogee"));
    flushNow();
  }
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

//kalman filter altitude
float filterAlt() {
  return pressureKalmanFilter.updateEstimate(rawData.altitude);
}

/*======================= MPU DATA AND FILTERING + ORIENTATION DETECTION ====================*/

//calibrate acceleration values for kalman filtering
void calibrateAccel() {
  const int samples = 100;
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < samples; i++) {
    // Step 1: Read raw accelerometer data from MPU6050
    readAccelData(MPU_ADDRESS, mpuAccelRaw.x, mpuAccelRaw.y, mpuAccelRaw.z);

    // Step 2: Convert raw data to G-forces
    rawAccelToGForce(mpuAccelRaw.x, mpuAccelRaw.y, mpuAccelRaw.z,
                     mpuAccelGforce.x, mpuAccelGforce.y, mpuAccelGforce.z);

    // Step 3: Accumulate sums for averaging
    sumX += mpuAccelGforce.x;
    sumY += mpuAccelGforce.y;
    sumZ += mpuAccelGforce.z;

    delay(10); // give sensor time between reads
  }

  // Step 4: Calculate and store average (static bias)
  initAccelX = sumX / samples;
  initAccelY = sumY / samples;
  initAccelZ = sumZ / samples;

  // Step 5: Optional correction for gravity if calibrating flat
  // On a flat surface, Z should measure ~+1g. Adjust so that happens.
  initAccelZ -= 1.0;

  debugLog(F("Accelerometer calibration complete, initial values (x,y,z):"));
  debugLog(String(initAccelX));
  debugLog(String(initAccelY));
  debugLog(String(initAccelZ));
}

//CHECK MPU FOR NEW DATA
void getAccelData() {
  readAccelData(MPU_ADDRESS, mpuAccelRaw.x, mpuAccelRaw.y, mpuAccelRaw.z); 
  rawAccelToGForce(mpuAccelRaw.x, mpuAccelRaw.y, mpuAccelRaw.z, mpuAccelGforce.x, mpuAccelGforce.y, mpuAccelGforce.z); 
}

//CALCULATE PITCH AND ROLL FROM ACCELEROMETER
Orientation getOrientation(AccelData filtAccel) {
  Orientation result;
  float pitch,roll;

  calculateAnglesFromAccel(filtAccel.x, filtAccel.y, filtAccel.z, pitch, roll); // uses trigonometry to calculate angles with accelerometer values    // prints mpu6050 values in the terminal

  result.pitch = pitch - initAccelX; // adjust accelerometer values to compensate for offset values
  result.roll = roll - initAccelY;

  return result;
}

// Filtering function for MPU6050 accelerometer data
AccelData filterAccel(AccelData mpuAccelGforce) {
  // --- Step 1: Remove static sensor bias (from calibration) ---
  float rawX = mpuAccelGforce.x - initAccelX;
  float rawY = mpuAccelGforce.y - initAccelY;
  float rawZ = mpuAccelGforce.z - initAccelZ;

  // --- Step 2: Estimate gravity vector from current orientation ---
  float pitchRad = heading.pitch * DEG_TO_RAD;
  float rollRad  = heading.roll  * DEG_TO_RAD;

  // gravity components in sensor frame
  float gX = 9.81f * sin(pitchRad);
  float gY = -9.81f * sin(rollRad);
  float gZ = 9.81f * cos(pitchRad) * cos(rollRad);

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

/*======================= INITIALISATIONS ====================*/
//BUZZER INITIALISE
void startBuzzer()  {
  cute.init(BUZZER_PIN);
  cute.play(S_CONNECTION);
  debugLog(F("Buzzer Initialised"));
}

//initialise pins
void startPins()  {
  pinMode(LED_PIN,OUTPUT);
  pinMode(BUTTON_PIN,INPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  debugLog(F("Pins Initialised"));
}

/*======================= TIMED CALLS ====================*/
//arm system
void systemArm()  {
  armState = ArmState::armed;
  debugLog(F("System State --> Armed"));
  lockServos();
  zeroServos();
  cute.play(S_BUTTON_PUSHED);
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
  openLogFile();                    //open log and start recording
  servoFilePath = createServoFile();  //create servo log file
  openServoFile();
  dataLogging = true;
  debugLog(F("Datalogging Started"));

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

//detect if rising edge of button press
bool lastButtonState = LOW;
void detectButton() {
  bool currentButtonState = digitalRead(BUTTON_PIN);

  // Detect rising edge
  if (currentButtonState == HIGH && lastButtonState == LOW) {
    debugLog(F("Button Press Detected"));
    if (armState == ArmState::on)  {
      initialise();
      unlockServos();
    } else if (armState == ArmState::initialised && millis() - buttonTimer >= buttonDelay) {
      systemArm();
    }
  }

  lastButtonState = currentButtonState;
}

//print heartbeat in loop
void heartBeat()  {
  static unsigned long lastBeat = 0;
  if (millis() - lastBeat > 1000) {
    debugLog(F("[HEARTBEAT] Loop alive"));
    lastBeat = millis();
  }
}
/*================================================== SETUP & LOOP ===================================================*/

void setup() {
  Serial.begin(9600);
  startSD();
  delay(100);
  createDebugFile();
  openDebugFile();
  startBarometer();
  startAccelerometer();
  startPins();
  startServos();
  startBuzzer();
  zeroServos();
  startPID();
  debugLog(F("System State --> On"));
}

//loop
void loop() {
  if(isDebug) heartBeat();                    //heartbeat for debugging

  detectButton();                           //check for button press
  commandCheck();                           //check for serial command input
  simulateBurnout();                        //timer check for burnout after simulate command

  //read bmp data at set speed
  if (millis() - lastBMPTrigger >= logTime) {
    lastBMPTrigger = millis();
    updateBMP(rawData);
  }
  //read altitude
  filtAlt = filterAlt();                    //clean altitude

  getAccelData();                           //update mpu sensor

  filtAccel = filterAccel(mpuAccelGforce);  //clean acceleration
  heading = getOrientation(filtAccel);      //get orientation from filtered data
  velocity = calculateVelocity(filtAccel);  //check velocity
  speed = velocity.magnitude;               //find current speed

  updatePID();                              //update pid controller

  dataLog();                                //log data to sd card
  flushLogs();                              //write log buffers to sd card after set timer (flushTimer = 500ms)

  ifApogee();                               //check if we have reached apogee

  checkArmState();    //arming state machine for light & sound display

  checkFlightState();   //flight state machine
}