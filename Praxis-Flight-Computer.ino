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
const unsigned long BMPperiod = 100;            // BMP records at 10 hz
const unsigned long IMUperiod = 5;              // IMU records at 200 hz
const unsigned long buttonDelay = 2000;         //safety delay on button press (ms)
const unsigned long stopDelay = 5000;           //timer before stopping system after landing

//STATE DETECTION PHYSICAL PARAMETERS
int detectLaunchAlt=10, detectLaunchG=2;        //G's and ALTITUDE (M) TO DETECT LIFTOFF
int detectLandAlt=5,detectLandSpeed=5;          //altitude AND VELOCITY to detect landing
float burnoutSpeed = 1.2;                       //gs to detect burnout

const float alpha = 0.98f;                      //complementary filter weighting for heading


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
SensorData rawData; //Unfiltered data for logging
SensorData filterData;    //filtered data

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
Velocity velocity = {0, 0, 0, 0};  

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

//Buzzer
#include <ezBuzzer.h>
ezBuzzer buzzer(BUZZER_PIN, BUZZER_TYPE_ACTIVE, HIGH); // create ezBuzzer object: pin, type, activeLevel

//misc libraries
#include <math.h>
#include <Arduino.h>

/*======================= GLOBAL VARIABLES ====================*/
//float initPres;            //initlisation pressure
float groundPres;           //init pres converted to hPa
bool dataLogging = false;   //TO TURN ON DATALOGGING
int detectLaunchAccel;      //calculated value for liftoff accel
//float filterData.altitude;    //filtered altitude
bool servoLock = 1;   //locks servo actuation
unsigned long flightStart;
unsigned long apogeeTime;
bool ended = 0; //flight ended
bool simulateCheck = 0;
bool apogeeCheck =0;

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

//APOGEE DETECTION
float prevAlt = 0;                     // last filtered altitude
float maxAlt = 0;                      // running maximum altitude
const int apogeeConfirmSamples = 3;    // number of consecutive decreases to confirm
int apogeeCounter = 0;                 // counter of decreases
float apogeeDeadband = 1;            // minimum decrease (meters) to count
bool apogeeDetected = false;           // flag

String debugFilePath;       //file path for debug logs
String logFilePath;         //file path for log file on sd card
String servoFilePath;       //file path for servo logs

double xAngle,yAngle;             //SERVO TARGET ANGLES

//restart sd card if failed
unsigned long lastRestartAttempt = 0;
const unsigned long restartCooldown = 1000; // 1 sec

//so that sensors dont trigger too often
unsigned long lastBMPTrigger = 0;
unsigned long lastIMUTrigger = 0;


/*======================= MISC OUTPUTS ====================*/
// continuous buzzer beeps for when armed 
int armMelody[] = {NOTE_E5,NOTE_G5};
int armDuration[] = {8,16};
int armlength = sizeof(armDuration) / sizeof(int);
void armBuzzer(unsigned long &buzzerTimer) {
  if (millis()-buzzerTimer>=2000)  {
    buzzer.stop();
    buzzer.playMelody(armMelody, armDuration, armlength); 
    buzzerTimer = millis();
  }
}

// play ending melody
int endMelody[] = {NOTE_G5,NOTE_E5,NOTE_C5};
int endDuration[] = {2,2,2};
int endlength = sizeof(endDuration) / sizeof(int);
void endNoise() {
  buzzer.stop();
  buzzer.playMelody(endMelody, endDuration, endlength); 
}

// play init noise
void initMelody() {
  buzzer.stop();
  buzzer.beep(100,100,2);
}

// play turn on melody
int onMelody[] = {NOTE_E5,NOTE_G5,NOTE_C5,NOTE_G5};
int onDuration[] = {8,8,4,2};
int onlength = sizeof(endDuration) / sizeof(int);
void onBuzzer() {
  buzzer.stop();
  buzzer.playMelody(onMelody, onDuration, onlength); 
}

// flash led function - takes flashdelay input
void flashLED(unsigned long flashDelay,unsigned long &flashTimer) {
  if (millis()-flashTimer>=flashDelay)  {
    ledState = !ledState;
    digitalWrite(LED_PIN,ledState);
    flashTimer = millis();
  }
}
/*======================= APOGEE  ====================*/
//detect apogee
bool detectApogee(SensorData filterData,Velocity velocity) {
  float verticalVelocity = velocity.z;    //vertical velocity component

  // Update running maximum
  if (filterData.altitude > maxAlt) {
      maxAlt = filterData.altitude;   // if climbing, update max altitude
  } 
  else if ((maxAlt - filterData.altitude) > apogeeDeadband && verticalVelocity < 0) {
      apogeeCounter++;    // note descending altitude
      if (apogeeCounter >= apogeeConfirmSamples) {    //if multiple descending calls, trigger apogee
          apogeeDetected = true;
          prevAlt = filterData.altitude;
          return true;
      }
  } 
  else if ((filterData.altitude - prevAlt) > -0.2) {
    // Slowly decay descent counter instead of full reset
    apogeeCounter = max(0, apogeeCounter - 1);
  }
  prevAlt = filterData.altitude;  // update for next call
  return false;
}

void ifApogee() {
  //DETECT APOGEE
  if (!apogeeDetected && detectApogee(filterData, velocity)) {
    flightState = FlightState::apogee;
    apogeeTime = millis() - flightStart;
    debugLog(F("Apogee detected! Max Altitude: "));
    debugLog(String(maxAlt));
    debugLog(F("Flight State --> Apogee"));
    flushNow();
  }
}

/*======================= INITIALISATIONS ====================*/

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
}

//system landed calls
void systemLanded() {
          armState = ArmState::locked;
          debugLog(F("Sytem State --> Locked"));

          generateSummary();
          endLogging();

          endNoise();
          ended = 1;
          debugLog(F("Flight Complete! <3 "));
}

//initialise on 1st button press 
void initialise() {
  logFilePath = createLogFile();    //create log file with header
  openLogFile();                    //open log and start recording
  servoFilePath = createServoFile();  //create servo log file
  openServoFile(); 

  //start logging data
  dataLogging = true;
  debugLog(F("Datalogging Started"));

  //move to initialised state, play noise
  armState = ArmState::initialised;     
  initMelody();
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
      //first press
      initialise();
      unlockServos();
    } else if (armState == ArmState::initialised && millis() - buttonTimer >= buttonDelay) {
      //2nd press
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

  recoverI2CBus();          //reset the i2c bus to fix hanging issue on reset
  delay(10);
  startBarometer();
  startAccelerometer();

  startPins();

  startServos();
  zeroServos();
  startPID();

  calibrateGyro();      //calibrate MPU6050 gyro
  initOrientation();      //initialise orientation
  
  onBuzzer();
  debugLog(F("System State --> On"));
}

//loop
void loop() {
  unsigned long now = millis();
  if(isDebug) heartBeat();                  //heartbeat for debugging
  buzzer.loop();                            //buzzer trigger, must be in loop

  detectButton();                           //check for button press
  commandCheck();                           //check for serial command input
  simulateBurnout();                        //timer check for burnout after simulate command

  //read BMP sensor data at set speed
  if (now - lastBMPTrigger >= BMPperiod) {
    lastBMPTrigger = now;

    updateBMP(rawData);

    filterData.pressure_hPa = filterPres();          //update kalman filter with new data
    filterData.altitude = 44330.0 * (1.0 - pow((filterData.pressure_hPa / groundPres), 0.1903));
  }

  //read IMU sensor data at set speed
  if (now - lastIMUTrigger >= IMUperiod) {
    float dt = (now - lastIMUTrigger) * 0.001f;
    lastIMUTrigger = now;

    updateIMU();

    // Use dt here for:
    // - gyro integration
    // - complementary / Madgwick filter
    filterIMU();
    velocity = calculateVelocity(dt);  //check velocity

    updateOrientation(dt);    //complementary filtered orientation
  }

  //heading = getOrientation(filtAccel);      //get orientation from filtered data

  //updatePID();                              //update pid controller

  dataLog();                                //log data to sd card
  flushLogs();                              //write log buffers to sd card after set timer (flushTimer = 500ms)

  ifApogee();                               //check if we have reached apogee

  checkArmState();    //arming state machine for light & sound display
  checkFlightState();   //flight state 
}