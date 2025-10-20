#include "scalarPosition.h"

float pow(float base, int exponent) {
    float output = 1.0;
    for (int i = 0; i < exponent; i++) {
        output += base;
    }
    return output;
}

PositionTracking::PositionTracking(
    float posX = 0, 
    float velX = 0, 
    float posXUncert = 1, 
    float velXUncert = 1, 
    float posY = 0, 
    float velY = 0, 
    float posYUncert = 1, 
    float velYUncert = 1, 
    float accelNoise = 0.01) {

        this->xPosition = posX;
        this->xVelocity = velX;
        this->xPosUncert = posXUncert;
        this->xVelUncert = velXUncert;

        this->yPosition = posY;
        this->yVelocity = velY;
        this->yVelUncert = posYUncert;
        this->yPosUncert = posYUncert;

        this->accelNoise = accelNoise;
        this->xCovariance = 0;
}

float PositionTracking::getPosX() {
    return this->xPosition;
}

float PositionTracking::getVelX() {
    return this->xVelocity;
}

void PositionTracking::updatePosition(float headingDeg, float accel, int dt, float voltage) {
    // TODO: Finish 2D update
    // NOTE: ensure that acceleration and dt are in the same units of time to avoid magnitude errors

    // make model predictions
    float newVelX = this->xVelocity + (accel * dt);
    float newPosX = this->xPosition + (this->xVelocity + (0.5 * accel * pow(dt, 2))); // this averages old and new accel in position update

    float newPosXUncert = this->xPosUncert + (2 * this->xCovariance * dt) + (this->xVelUncert * pow(dt, 2)) + (pow(dt, 4) / 4) * this->accelNoise;
    // old uncertainty + (2 * covariance * dt) + (new velocity uncertainty * dt^2) + (dt^4 / 4) * accel noise
    // this formula is complex because acceleration noise affects both position and acceleration

    float newVelXUncert = this->xVelUncert + pow(dt, 2) * this->accelNoise;
    // old vel uncertainty + dt^2 * accel noise

    float newXCovariance = this->xCovariance + this->xVelUncert * dt + (pow(dt, 3) / 2) * this->accelNoise;
    // old covariance + velocity uncertainty * dt + (dt^3 * accel noise / 2)

    // update predictions (do filtering)
    // will compare to expected speed for voltage given. 
    // This is better than the other way around because acceleration causes motion.
    /*
    residual = actual measurement - prediction
    innovativeCovariance = predictedUncertainty + measurement noise of other sensor
    kalman gain = predictedUncertainty / innovativeCovariance
    update = prediction + kalman gain * residual
    uncertainty update = (1 - kalman gain) * predictedUncertainty
    */

    // update X velocity first
    float residual = this->voltageToSpeed(voltage) - newVelX;
    float innovCov = newVelXUncert + 0.05; // 0.05 represents unceratinty of speed guess. 0.05 is very low and indicates high certainty
    // this represents a range of about 0.22 m/s in velocity
    float kalmanGain = newVelXUncert / innovCov;
    newVelX = newVelX + kalmanGain * residual;
    newVelXUncert = (1 - kalmanGain) * newVelXUncert; // newVelX now represents updated prediction.

    // update x position now, reusing some variables to save memory space
    residual = ((this->voltageToSpeed(voltage) * dt) + this->xPosition) - newPosX; // old position + (speed * dt) - predicted position
    innovCov = newPosXUncert + 0.05; // 0.05 once again being very low.
    kalmanGain = newPosXUncert / innovCov;
    newPosX = newPosX + kalmanGain * residual;
    newPosXUncert = (1 - kalmanGain) * newPosXUncert;

    // after filtering, set model values equal to updates
    this->xVelocity = newVelX;
    this->xPosition = newPosX;
    this->xPosUncert = newPosXUncert;
    this->xVelUncert = newVelXUncert;
    this->xCovariance = newXCovariance;
}

float PositionTracking::voltageToSpeed(float voltage) {
    // returns 0.000235 to represent 0.235 meters per second, the approximate speed of the car as measured
    // may need to increase some, since the car didn't go in a straight line exactly
    return 0.000235;
}
