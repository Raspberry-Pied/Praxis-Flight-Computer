/*======================= SD CARD AND LOGGING BEHAVIOUR ====================*/

//debug logging
File debugFile;
File logFile;
File servoFile;
unsigned long lastFlushTime = 0;
const unsigned long flushInterval = 500;  // flush every 0.5s

//SD card init
void startSD()  {                                     //keep trying to start sd card until it works
  if (!SD.begin(csPIN)) {
    while (true)  {
      Serial.println(F("Could not find a valid SD card, retrying..."));
      systemFaults |= FAULT_SD_CARD;
      tone(BUZZER_PIN, 1000, 150);
      delay(200);
      tone(BUZZER_PIN, 800, 150);
      delay(500);
    }
  }
  Serial.println(F("SD Card Started"));
  if (!SD.exists("/PRAXIS/LOGS")) SD.mkdir("/PRAXIS/LOGS");
  if (!SD.exists("/PRAXIS/DEBUG")) SD.mkdir("/PRAXIS/DEBUG");
}

//Helper funct to create a file with headers based on input path
String createFileWithHeader(const char* baseDir, const char* prefix, const char* header) {
    // Ensure directory exists
    if (!SD.exists(baseDir)) {
        Serial.print(F("Directory not found, creating: "));
        Serial.println(baseDir);
        SD.mkdir(baseDir);
    }

    // Find next available file number
    int fileNum = 1;
    String filePath;
    while (true) {
        filePath = String(baseDir) + prefix + String(fileNum) + ".txt";
        if (!SD.exists(filePath.c_str())) break;
        fileNum++;
    }

    // Open file for writing
    File file = SD.open(filePath.c_str(), FILE_WRITE);
    if (!file) {
        Serial.print(F("Error: Could not create file: "));
        Serial.println(filePath);
        return "";
    }

    // Write header and close
    file.println(header);
    file.close();

    Serial.print(F("Created file: "));
    Serial.println(filePath);

    return filePath;
}

//creates a log file for datalogging
String createLogFile() {
  const char* baseDirLogs = "/PRAXIS/LOGS/";
  const char* prefixLog = "Sens_";
  const char* logHeader = "Temp (C),Pressure (Pa),Altitude (m),X Accel,Y Accel,Z Accel,X Gyro,Y Gyro,Z Gyro,Filter Pres,Filter Alt,Filter X Accel,Filter Y Accel,Filter Z Accel,Filter X Gyro,Filter Y Gyro,Filter Z Gyro,Speed (m/s),X velocity (m/s),Y velocity (m/s),Z velocity (m/s)";
    
  return createFileWithHeader(baseDirLogs, prefixLog, logHeader);
}

//Create a servo log file
String createServoFile()  {
  const char* baseDirServo = "/PRAXIS/LOGS/";
  const char* prefixServo = "Servo_";
  const char* servoHeader = "Pitch,Roll,Yaw,X Servo Target Angle (deg),Y Servo Target Angle (deg)";

  return createFileWithHeader(baseDirServo, prefixServo, servoHeader);
}

//creates a debug log file
String createDebugFile() {
    const char* baseDir = "/PRAXIS/DEBUG/";
    const char* prefix = "Debug_";
    const char* header = "=== DEBUG LOG START ===\nCreated new debug file:";

    String path = createFileWithHeader(baseDir, prefix, header);

    //Store path in a global variable
    debugFilePath = path;

    return path;
}


//open debug log for use
void openDebugFile()  {
  debugFile = SD.open(debugFilePath.c_str(), FILE_WRITE);
  if (!debugFile) {
    Serial.println(F("Error opening debug file"));
    systemFaults |= FAULT_DEBUGLOG;
  }
  debugLog(F("Debug file opened for writing"));
}

//Flash string overload for F() macro - allows calling debugLog(F("String"))
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

//Regular String overload for dynamic text - allows calling debugLog(variable)
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

//OPEN LOG FILE FOR USE
void openLogFile()  {
  logFile = SD.open(logFilePath.c_str(), FILE_WRITE);
  if (!logFile) {
    debugLog(F("Error opening data log file"));
    systemFaults |= FAULT_SENSORLOG;
  } else {
    debugLog(F("Data log file opened successfully"));
  }
}

//open servo log file
void openServoFile()  {
  servoFile = SD.open(servoFilePath.c_str(), FILE_WRITE);
  if (!servoFile) {
    debugLog(F("Error opening servo log file"));
    systemFaults |= FAULT_SERVOLOG;
  } else {
    debugLog(F("Servo log file opened successfully"));
  }
}

//writes line of csv to logfile
void writeLogFile() {
  String csvLog = String(rawData.tempC) + "," +
                  String(rawData.pressure_hPa) + "," +
                  String(rawData.altitude) + "," +
                  String(rawData.accel[0]) + "," +
                  String(rawData.accel[1]) + "," +
                  String(rawData.accel[2]) + "," +
                  String(rawData.gyro[0]) + "," +
                  String(rawData.gyro[1]) + "," +
                  String(rawData.gyro[2]) + "," +
                  String(filterData.pressure_hPa) + "," +
                  String(filterData.altitude) + "," +
                  String(filterData.accel[0]) + "," +
                  String(filterData.accel[1]) + "," +
                  String(filterData.accel[2]) + "," +
                  String(filterData.gyro[0]) + "," +
                  String(filterData.gyro[1]) + "," +
                  String(filterData.gyro[2]) + "," +
                  String(velocity.magnitude) + "," +
                  String(velocity.x) + "," +
                  String(velocity.y) + "," +
                  String(velocity.z);
  logFile.println(csvLog);
}

//writes line of csv to servo file
void writeServoFile() {
  String csvLog = String(heading.pitch) + "," +
                  String(heading.roll) + "," +
                  String(heading.yaw) + "," +
                  String(xAngle) + "," +
                  String(yAngle);
  servoFile.println(csvLog);
}

//datalogging function
void dataLog()  {
  if (dataLogging) {
    if (millis()-dataTimer>=logTime)  {

      //write results to sd in csv format
      if (logFile)  {
        writeLogFile();
      } else debugLog(F("Error Logging Sensor Data"));

      if (servoFile)  {
        writeServoFile();
      } else debugLog(F("Error Logging Servo Data"));

      dataTimer = millis();
    }
  }
}



//flush log buffer to sd card
void flushLogs()  {
  unsigned long flushTimer = millis();
  if (flushTimer - lastFlushTime >= flushInterval) {
    if (SD.exists("/PRAXIS"))  {
      if (debugFile) debugFile.flush();
      if (logFile) logFile.flush();
      if (servoFile) logFile.flush();
      lastFlushTime = flushTimer;
    } else {
      if (millis() - lastRestartAttempt > restartCooldown) {  //restart sd card if not detected
        lastRestartAttempt = millis();
        debugLog(F("Warning: SD card disconnected, attempting to restart"));
        if (SD.begin(csPIN)) {
          debugLog(F("SD card restarted successfully"));
        } else {
          debugLog(F("SD card couldn't be started"));
        }
      }
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
  dataLogging = false;
  debugLog(F("Datalogging Paused"));
  
  if (logFile)  {
    debugLog(F("Data Log file closing"));
    logFile.flush();
    logFile.close();
  }
  if (servoFile)  {
    debugLog(F("Servo Log file closing"));
    servoFile.flush();
    servoFile.close();
  }
  if (debugFile) {
    debugLog(F("Debug Log file closing"));
    debugFile.flush();
    debugFile.close();
  }
}
