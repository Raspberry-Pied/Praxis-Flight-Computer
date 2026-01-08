
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
