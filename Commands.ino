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
