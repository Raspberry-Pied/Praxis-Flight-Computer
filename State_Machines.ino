// flight state machine - only activate if armed
void checkFlightState() {
  if (armState == ArmState::armed)  {
    switch (flightState)  {

      case FlightState::ground:
      if (filterData.altitude >= detectLaunchAlt) { //&& filtAccel.magnitude >= detectLaunchAccel)  {
        flightState = FlightState::burn;
        debugLog(F("Flight State --> Burn"));
      }
      break;

      case FlightState::burn:
      if (!ended) { //(velocity.magnitude <= initAccel * burnoutSpeed)  { //////DETECT MOTOR BURNOUT
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
      lockServos();
      zeroServos();
      break;

      case FlightState::descent:
      if (filterData.altitude <= detectLandAlt) { // && speed <= detectLandSpeed) {
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

// arming state machine
void checkArmState() {
  switch (armState) {

    // on - 1 hz flash, waiting for init
    case ArmState::on:
    flashLED(1000,flashTimer);
    break;

    //init - solid, waiting for arm
    case ArmState::initialised:
    digitalWrite(LED_PIN,HIGH);
    break;

    case ArmState::armed:
    flashLED(100,flashTimer);
    if (millis()-buzzerTimer>=2000)  {
      playSound(SOUND_WARNING);
      buzzerTimer = millis();
    }
    break;

    case ArmState::locked:
    digitalWrite(LED_PIN,HIGH);
    break;

  }
}