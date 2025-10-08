#include "scalarPosition.h"

float pow(float base, int exponent) {
    float output = 1.0;
    for (int i = 0; i < exponent; i++) {
        output += base;
    }
    return output;
}

PositionTracking::PositionTracking(float posX = 0, float velX = 0, float posXUncert = 1, float velXUncert = 1, float accelNoise = 0.01) {
    this->xPosition = posX;
    this->xVelocity = velX;
    this->xPosUncert = posXUncert;
    this->xVelUncert = velXUncert;
    this->accelNoise = accelNoise;
    this->xCovariance = 0;
}

float PositionTracking::getPosX() {
    return this->xPosition;
}

float PositionTracking::getVelX() {
    return this->xVelocity;
}

void PositionTracking::updatePosition(float accel, int dt) {
    // NOTE: ensure that acceleration and dt are in the same units of time to avoid magnitude errors

    // make model predictions
    float newVelX = this->xVelocity + (accel * dt);
    float newPosX = this->xPosition + (this->xVelocity + (0.5 * accel * pow(dt, 2))); // this averages old and new accel in position update

    float newPosXUncert = this->xPosUncert + (2 * this->xCovariance * dt) + (this->xVelUncert * pow(dt, 2)) + (pow(dt, 4) / 4) * this->accelNoise;
    // old uncertainty + (2 * covariance * dt) + (new velocity uncertainty * dt^2) + (dt^4 / 4) * accel noise
    // this formula is complex because acceleration noise affects both position and acceleration

    float newVelXUncert = this->xVelUncert + pow(dt, 2) * this->accelNoise;
    // old vel uncertainty + dt^2 * accel noise

    float newXCoveriance = this->xCovariance + this->xVelUncert * dt + (pow(dt, 3) / 2) * this->xCovariance;
    // old covariance + velocity uncertainty * dt + (dt^3 * accel noise / 2)

    // update predictions (do filtering)
    // NEED SOMETHING ELSE TO COMPARE TO
    /*
    residual = actual measurement - prediction
    innovativeCovariance = predictedUncertainty + measurement noise of other sensor
    kalman gain = predictedUncertainty / innovativeCovariance
    update = prediction + kalman gain * residual
    uncertainty update = (1 - kalman gain) * predictedUncertainty
    */


    // after filtering, set model values equal to updates
    this->xVelocity = newVelX;
    this->xPosition = newPosX;
    this->xPosUncert = newPosXUncert;
    this->xVelUncert = newVelXUncert;
    this->xCovariance = newXCoveriance;
}