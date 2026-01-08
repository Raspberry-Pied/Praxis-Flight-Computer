void updateOrientation(float dt) {
  //calculate pitch & roll from acceleration
  float rollAccel  = atan2(kalmanAccel[1], kalmanAccel[2]);
  float pitchAccel = atan2(-kalmanAccel[0], sqrt(kalmanAccel[1]*kalmanAccel[1] + kalmanAccel[2]*kalmanAccel[2]));

  //gyro integration
  heading.roll  += kalmanGyro[0] * DEG_TO_RAD * dt;
  heading.pitch += kalmanGyro[1] * DEG_TO_RAD * dt;
  heading.yaw   += kalmanGyro[2] * DEG_TO_RAD * dt;

  //complementary filter
  heading.roll  = alpha * heading.roll  + (1.0f - alpha) * rollAccel;
  heading.pitch = alpha * heading.pitch + (1.0f - alpha) * pitchAccel;
}

//initialise orientation
void initOrientation() {
  heading.roll = atan2(rawData.accel[1], rawData.accel[2]);
  heading.pitch = atan2(-rawData.accel[0],
                           sqrt(rawData.accel[1]*rawData.accel[1] +
                                rawData.accel[2]*rawData.accel[2]));
  heading.yaw = 0;
  debugLog(F("Orientation Initialised"));
}



