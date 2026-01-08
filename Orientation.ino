/*
//CALCULATE PITCH AND ROLL FROM ACCELEROMETER
Orientation getOrientation(AccelData filtAccel) {
  Orientation result;
  float pitch,roll;

  calculateAnglesFromAccel(filtAccel.x, filtAccel.y, filtAccel.z, pitch, roll); // uses trigonometry to calculate angles with accelerometer values    // prints mpu6050 values in the terminal

  result.pitch = pitch - initAccelX; // adjust accelerometer values to compensate for offset values
  result.roll = roll - initAccelY;

  return result;
}
*/