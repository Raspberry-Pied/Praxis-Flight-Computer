//KALMAN FILTER PARAMETERS
float e_mea = 0.3;                              // Measurement noise
float e_est = 0.5;                              // Estimation error
float q     = 0.01;                             // Process noise
float noiseThresh = 0.2;                        //noise removal threshold for kalman filter

//kalman filter
#include <SimpleKalmanFilter.h>
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

/*SimpleKalmanFilter accelKalmanX(e_mea, e_est, q);
SimpleKalmanFilter accelKalmanY(e_mea, e_est, q);
SimpleKalmanFilter accelKalmanZ(e_mea, e_est, q);
*/

//kalman filter altitude
float filterPres() {
  return pressureKalmanFilter.updateEstimate(rawData.pressure_hPa);
}