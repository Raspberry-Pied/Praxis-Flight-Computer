#define SDA_PIN  SDA
#define SCL_PIN  SCL

// MPU6050 3-axis accelerometer and gyro
#include <mpu6050.h>
#define MPU_ADDRESS 0x68  // AD0 pin tied to GND

//BMP280 barometer
#include <Wire.h>
#include <Adafruit_BMP280.h>
#define BMP280_ADDRESS 0x76                //I2C ADDRESS
Adafruit_BMP280 bmp; 

//manually resets i2c bus on start to fix hung bus issue 
void recoverI2CBus() {
  // Release both lines
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(5);

  // If SDA is low, clock SCL until it releases
  pinMode(SCL_PIN, OUTPUT);

  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
  }

  // Generate a STOP condition
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);

  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);

  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(5);

  // Return control to I2C hardware
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  debugLog(F("I2C Bus reset"));
}

//Start barometer and validate connection
bool startBarometer() {

  if (!bmp.begin(BMP280_ADDRESS)) {
    debugLog(F("Could not find a valid BMP280 sensor"));
    flushNow();
    systemFaults = FAULT_BMP;
    return false;
  }

  /* Default BMP280 settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode --- FORCED. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500   /* Standby time. */
                  );

  delay(10);     //allow time for first conversion to complete

  bmp.takeForcedMeasurement();    //initialise ground pressure for altitude calcs
  groundPres = bmp.readPressure() / 100.0;

  debugLog(F("Barometer Initialised"));
  debugLog(F("Ground Pressure:"));
  debugLog(String(groundPres));
  return true;
}

//start accelerometer and validate connection
#define WHO_AM_I 0x75
void startAccelerometer() {
  //check the sensor is responding
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(WHO_AM_I);
  if (Wire.endTransmission(false) != 0) {
    debugLog(F("Accelerometer not detected!"));
    systemFaults |= FAULT_IMU;
    return;
  }

  wakeSensor(MPU_ADDRESS); // wakes sensor from sleep mode
  delay(200);
  debugLog(F("Accelerometer Initialised"));
}

//update bmp sensor
void updateBMP(SensorData &d)  {
  if (!bmp.takeForcedMeasurement()) {
   return;  // I2C error
  }

  float tempC = bmp.readTemperature();
  float pressure_hPa = bmp.readPressure() / 100.0;

  // Compute altitude from pressure (no extra I2C read)
  float altitude_m = 44330.0 * (1.0 - pow((pressure_hPa / groundPres), 0.1903));

  //update struct
  d.tempC = tempC;
  d.pressure_hPa = pressure_hPa;
  d.altitude = altitude_m;
}

float unprocessedGyro[3];
float preprocessedGyro[3];
float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

//calibrate gyro data to remove chip bias
void calibrateGyro() {
  const int samples = 1500;

  gyroBiasX = 0;
  gyroBiasY = 0;
  gyroBiasZ = 0;

  for (int i = 0; i < samples; i++) {
    readGyroData(MPU_ADDRESS, unprocessedGyro[0], unprocessedGyro[1], unprocessedGyro[2]);
    rawGyroToDPS(unprocessedGyro[0], unprocessedGyro[1], unprocessedGyro[2], preprocessedGyro[0], preprocessedGyro[1], preprocessedGyro[2]);

    gyroBiasX += preprocessedGyro[0];
    gyroBiasY += preprocessedGyro[1];
    gyroBiasZ += preprocessedGyro[2];

    delay(2);
  }

  gyroBiasX /= samples;
  gyroBiasY /= samples;
  gyroBiasZ /= samples;

  debugLog(F("Gyro calibration complete, initial biases (x,y,z):"));
  debugLog(String(gyroBiasX));
  debugLog(String(gyroBiasY));
  debugLog(String(gyroBiasZ));
}

//check current gyro readings
void readGyro(float &gX, float &gY, float &gZ) {
    readGyroData(MPU_ADDRESS, unprocessedGyro[0], unprocessedGyro[1], unprocessedGyro[2]);
    rawGyroToDPS(unprocessedGyro[0], unprocessedGyro[1], unprocessedGyro[2], preprocessedGyro[0], preprocessedGyro[1], preprocessedGyro[2]);

  gX = preprocessedGyro[0] - gyroBiasX;
  gY = preprocessedGyro[1] - gyroBiasY;
  gZ = preprocessedGyro[2] - gyroBiasZ;
}

//read current accelerometer
float unprocessedAccel[3];
float preprocessedAccel[3];
void readAccel(float &aX, float &aY, float &aZ) {
  readAccelData(MPU_ADDRESS, unprocessedAccel[0], unprocessedAccel[1], unprocessedAccel[2]);
  rawAccelToGForce(unprocessedAccel[0], unprocessedAccel[1], unprocessedAccel[2], aX, aY, aZ );
}

//updates imu sensor data to rawData struct
void updateIMU()  {
    readAccel(rawData.accel[0], rawData.accel[1], rawData.accel[2]);      //update mpu sensor
    readGyro(rawData.gyro[0],rawData.gyro[1],rawData.gyro[2]);
}