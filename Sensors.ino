#define SDA_PIN  SDA
#define SCL_PIN  SCL

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
}

//Start barometer and validate connection
bool startBarometer() {
  recoverI2CBus();
  Wire.begin(); //test if using this fixes hung i2c bus issue
  delay(10);

  if (!bmp.begin(BMP280_ADDRESS)) {
    debugLog(F("Could not find a valid BMP280 sensor"));
    flushNow();
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

  debugLog(F("Barometer Initialised in FORCED mode"));
  return true;
}

//start accelerometer and validate connection
void startAccelerometer() {
  wakeSensor(MPU_ADDRESS); // wakes sensor from sleep mode
  delay(200);
  debugLog(F("Accelerometer Initialised"));
}

BMPState bmpState = BMP_IDLE;
unsigned long bmpStartTime = 0;

const unsigned long BMP_CONVERSION_TIME_MS = 40;

//update bmp sensor
bool updateBMP(SensorData &d)  {
  switch (bmpState) {

    case BMP_IDLE:
      // Start a new measurement
      if (!bmp.takeForcedMeasurement()) {
        return false;  // I2C error
      }

      bmpStartTime = millis();
      bmpState = BMP_CONVERTING;
      return false;   // not ready yet

    case BMP_CONVERTING:
      // Check if conversion is finished
      if (millis() - bmpStartTime < BMP_CONVERSION_TIME_MS) {
        return false; // still converting
      }

      // Read results
      float tempC = bmp.readTemperature();
      float pressure_hPa = bmp.readPressure() / 100.0;

      // Compute altitude from pressure (no extra I2C read)
      float altitude_m = 44330.0 * (1.0 - pow((pressure_hPa / groundPres), 0.1903));

      //update struct
      d.tempC = tempC;
      d.pressure_hPa = pressure_hPa;
      d.altitude = altitude_m;

      bmpState = BMP_IDLE;
      return true;    // new data available
  }

  return false; // should never happen
}