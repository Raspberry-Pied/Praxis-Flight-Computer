//KALMAN FILTER PARAMETERS
float e_mea = 0.3;                              // Measurement noise
float e_est = 0.5;                              // Estimation error
float q     = 0.01;                             // Process noise
float noiseThresh = 0.2;                        //noise removal threshold for kalman filter

//kalman filter
#include <SimpleKalmanFilter.h>
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

SimpleKalmanFilter accelKalmanX(e_mea, e_est, q);
SimpleKalmanFilter accelKalmanY(e_mea, e_est, q);
SimpleKalmanFilter accelKalmanZ(e_mea, e_est, q);
SimpleKalmanFilter gyroKalmanX(e_mea, e_est, q);
SimpleKalmanFilter gyroKalmanY(e_mea, e_est, q);
SimpleKalmanFilter gyroKalmanZ(e_mea, e_est, q);


//kalman filter altitude
float filterPres() {
  return pressureKalmanFilter.updateEstimate(rawData.pressure_hPa);
}

//kalman filter imu data
float kalmanAccel[3];
float kalmanGyro[3];
void filterIMU() {
  kalmanAccel[0] =  accelKalmanX.updateEstimate(rawData.accel[0]);
  kalmanAccel[1] =  accelKalmanY.updateEstimate(rawData.accel[1]);
  kalmanAccel[2] =  accelKalmanZ.updateEstimate(rawData.accel[2]);

  kalmanGyro[0] =  gyroKalmanX.updateEstimate(rawData.gyro[0]);
  kalmanGyro[1] =  gyroKalmanY.updateEstimate(rawData.gyro[1]);
  kalmanGyro[2] =  gyroKalmanZ.updateEstimate(rawData.gyro[2]);
}

/*======================= VELOCITY CALC ====================*/

// dt: time in seconds since last IMU update
Velocity calculateVelocity(float dt) {
    if (dt <= 0) return velocity; // nothing to integrate

    // integrate acceleration
    velocity.x += rawData.accel[0] * dt;
    velocity.y += rawData.accel[1] * dt;
    velocity.z += rawData.accel[2] * dt;

    // zero out small drift values
    if (fabs(velocity.x) < noiseThresh) velocity.x = 0;
    if (fabs(velocity.y) < noiseThresh) velocity.y = 0;
    if (fabs(velocity.z) < noiseThresh) velocity.z = 0;

    // calculate magnitude
    velocity.magnitude = sqrt(
        velocity.x * velocity.x +
        velocity.y * velocity.y +
        velocity.z * velocity.z
    );

    return velocity;
}





