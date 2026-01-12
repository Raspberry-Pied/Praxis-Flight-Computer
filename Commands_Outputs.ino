/*======================= COMMANDS ====================*/
//check serial for commands
void commandCheck() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equalsIgnoreCase("simulate")) {       ///simulate command - starts launch sequence
      debugLog(F("Simulate command received"));
      simulate();   
    }
    if (command.equalsIgnoreCase("apogee")) {       //apogee cmd - only triggers after simulate
      if (!simulateCheck) {
        debugLog(F("Apogee command not accepted - simulate not triggered"));
      } else  {
        if (millis() - simTimer >= 5000) {
          debugLog(F("Apogee command accepted"));
          flightState = FlightState::apogee;
          debugLog(F("Simulated State --> Apogee"));
        }
      }
    }
    if (command.equalsIgnoreCase("landed")) {       ///simulate command - starts launch sequence
      debugLog(F("Landed command received"));
      flightState = FlightState::landed;
      landTimer = millis();
      debugLog(F("Simulated State --> Landed"));
    }
    if (command.equalsIgnoreCase("debug"))  {
      isDebug = 1;
      debugLog(F("debug command received"));
    }
    if (command.equalsIgnoreCase("end"))  {
      debugLog(F("end command received"));
      endLogging();
    }
  }
}

//simulate  - override altitude for testing?
void simulate() {
  simTimer = millis();
  simulateCheck = 1;
  debugLog(F("Simulated State --> Burn"));
  flightState = FlightState::burn;
}

//in loop to detect burnout after 3s
void simulateBurnout()  {
  if (simulateCheck == 1 && flightState == FlightState::burn)  {
    if (millis() - simTimer >= 3000) {
      debugLog(F("Simulated State --> Coast"));
      flightState = FlightState::coast;
      simulateCheck = 0;
      apogeeCheck = 1;
    }
  }
} 

/*======================= BUZZER OUTPUTS ====================*/
// startup
 int startupMelody[]   = { NOTE_C5, NOTE_E5, NOTE_G5 };
 int startupDuration[] = { 8, 8, 8 };

// success
 int successMelody[]   = { NOTE_G5, NOTE_C6 };
 int successDuration[] = { 8, 4 };

// warning
 int warningMelody[]   = { NOTE_C5, NOTE_C5 };
 int warningDuration[] = { 8, 8 };

// error
 int errorMelody[]   = { NOTE_E5, NOTE_C5 };
 int errorDuration[] = { 8, 8 };

// critical
 int criticalMelody[]   = { NOTE_G5, NOTE_G5, NOTE_G5 };
 int criticalDuration[] = { 16, 16, 16 };

//end
int endMelody[] = {NOTE_G5,NOTE_E5,NOTE_C5};
int endDuration[] = {2,2,2};


void playSound(SoundType sound) {
  buzzer.stop();

  switch (sound) {
    case SOUND_STARTUP:
      buzzer.playMelody(
        startupMelody,
        startupDuration,
        sizeof(startupDuration) / sizeof(startupDuration[0])
      );
      break;

    case SOUND_SUCCESS:
      buzzer.playMelody(
        successMelody,
        successDuration,
        sizeof(successDuration) / sizeof(successDuration[0])
      );
      break;

    case SOUND_WARNING:
      buzzer.playMelody(
        warningMelody,
        warningDuration,
        sizeof(warningDuration) / sizeof(warningDuration[0])
      );
      break;

    case SOUND_ERROR:
      buzzer.playMelody(
        errorMelody,
        errorDuration,
        sizeof(errorDuration) / sizeof(errorDuration[0])
      );
      break;

    case SOUND_CRITICAL:
      buzzer.playMelody(
        criticalMelody,
        criticalDuration,
        sizeof(criticalDuration) / sizeof(criticalDuration[0])
      );
      break;

    case SOUND_END:
      buzzer.playMelody(
        endMelody,
        endDuration,
        sizeof(endDuration) / sizeof(endDuration[0])
      );
      break;
  }
}

/*======================= LED OUTPUTS ====================*/

// flash led function - takes flashdelay input
void flashLED(unsigned long flashDelay,unsigned long &flashTimer) {
  if (millis()-flashTimer>=flashDelay)  {
    ledState = !ledState;
    digitalWrite(LED_PIN,ledState);
    flashTimer = millis();
  }
}
/*======================= POST & FAULT CHECKING ====================*/
StartupStatus POSTstatus(uint16_t faults)  {
  printFaults(faults);      //log any detected faults

  if (faults & (FAULT_IMU | FAULT_BMP)) {
    return STARTUP_CRITICAL;
  }

  if (faults & (FAULT_DEBUGLOG | FAULT_SERVOLOG | FAULT_SENSORLOG| FAULT_SD_CARD)) {
    return STARTUP_ERROR;
  }

  return STARTUP_OK;
}

bool handleStartupResult() {
  StartupStatus status = POSTstatus(systemFaults);

  switch (status) {
    case STARTUP_OK:
      playSound(SOUND_STARTUP);
      debugLog(F("System State --> On"));
      return true;

    case STARTUP_ERROR:
      playSound(SOUND_ERROR);
      return true;  // cancel normal startup

    case STARTUP_CRITICAL:
      playSound(SOUND_CRITICAL);
      return false;  // hard stop
  }

  return false;
}

//fail to start if critical issues
void failStart() {
  while (true)  {
    flashLED(50,flashTimer);
    delay(500);
    tone(BUZZER_PIN, 1000, 150);
  }
}

//log any faults to debug and serial
void printFaults(uint16_t faults) {
  if (faults == 0) {
    debugLog(F("No faults detected."));
    return;
  }

  debugLog(F("Startup faults detected:"));

  if (faults & FAULT_IMU)        debugLog(F(" - IMU (Accelerometer/Gyro)"));
  if (faults & FAULT_BMP)        debugLog(F(" - BMP sensor"));
  if (faults & FAULT_SD_CARD)    debugLog(F(" - SD card"));
  if (faults & FAULT_DEBUGLOG)   debugLog(F(" - Debug log"));
  if (faults & FAULT_SERVOLOG)   debugLog(F(" - Servo log"));
  if (faults & FAULT_SENSORLOG)  debugLog(F(" - Sensor log"));
}
