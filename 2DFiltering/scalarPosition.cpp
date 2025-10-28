#include "scalarPosition.h"
#include<math.h>
#include "2DVelocity.h"

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
        this->yCovariance = 0;
}

float PositionTracking::getPosX() {
    return this->xPosition;
}

float PositionTracking::getVelX() {
    return this->xVelocity;
}

float PositionTracking::getPosY() {
    return this->yPosition;
}

float PositionTracking::getVelY() {
    return this->yVelocity;
}

void PositionTracking::updatePosition(float headingDeg, float accel, int dt, float voltage) {
    // NOTE: ensure that acceleration and dt are in the same units of time to avoid magnitude errors

    // make model predictions
    float magVelocity = sqrt(pow(this->xVelocity, 2) + pow(this->yVelocity, 2)); // a^2 = b^2 + c^2
    magVelocity = magVelocity + (accel * dt); // update with accelerometer info

    float newVelX, newVelY, accelX, accelY;

    motion::velPerAxis(magVelocity, headingDeg, newVelX, newVelY); // sets newVelX and newVelY to appropriate values based on trig
    motion::velPerAxis(accel, headingDeg, accelX, accelY); // sets accelX and accelY appropriately

    float newPosX = this->xPosition + (this->xVelocity + (0.5 * accelX * pow(dt, 2))); // this averages old and new accel in position update
    float newPosY = this->xPosition + (this->xVelocity + (0.5 * accelY * pow(dt, 2)));

    float newPosXUncert = this->xPosUncert + (2 * this->xCovariance * dt) + (this->xVelUncert * pow(dt, 2)) + (pow(dt, 4) / 4) * this->accelNoise;
    float newPosYUncert = this->yPosUncert + (2 * this->yCovariance * dt) + (this->yVelUncert * pow(dt, 2)) + (pow(dt, 4) / 4) * this->accelNoise;
    // old uncertainty + (2 * covariance * dt) + (new velocity uncertainty * dt^2) + (dt^4 / 4) * accel noise
    // this formula is complex because acceleration noise affects both position and acceleration
    
    float newVelXUncert = this->xVelUncert + pow(dt, 2) * this->accelNoise;
    float newVelYUncert = this->yVelUncert + pow(dt, 2) * this->accelNoise;
    // old vel uncertainty + dt^2 * accel noise

    float newXCovariance = this->xCovariance + this->xVelUncert * dt + (pow(dt, 3) / 2) * this->accelNoise;
    float newYCovariance = this->yCovariance + this->yVelUncert * dt + (pow(dt, 3) / 2) * this->accelNoise;
    // old covariance + velocity uncertainty * dt + (dt^3 * accel noise / 2)

    // update predictions (do filtering)
    // will compare to expected speed for voltage given. 
    // This is better than the other way around because acceleration causes motion.
    /*
    Filtering algorithm:
    residual = actual measurement - prediction
    innovativeCovariance = predictedUncertainty + measurement noise of other sensor
    kalman gain = predictedUncertainty / innovativeCovariance
    update = prediction + kalman gain * residual
    uncertainty update = (1 - kalman gain) * predictedUncertainty
    */

    float vSpeedX, vSpeedY; // speed in x and y determined by voltage
    motion::velPerAxis(this->voltageToSpeed(voltage), headingDeg, vSpeedX, vSpeedY);

    // update X velocity
    float residual = vSpeedX - newVelX;
    float innovCov = newVelXUncert + 0.05; // 0.05 represents unceratinty of speed guess. 0.05 is very low and indicates high certainty
    // this represents a range of about 0.22 m/s in velocity
    float kalmanGain = newVelXUncert / innovCov;
    newVelX = newVelX + kalmanGain * residual;
    newVelXUncert = (1 - kalmanGain) * newVelXUncert; // newVelX now represents updated prediction.

    // update X position, reusing some variables to save memory space
    residual = ((vSpeedX * dt) + this->xPosition) - newPosX; // old position + (speed * dt) - predicted position
    innovCov = newPosXUncert + 0.05; // 0.05 once again being very low.
    kalmanGain = newPosXUncert / innovCov;
    newPosX = newPosX + kalmanGain * residual;
    newPosXUncert = (1 - kalmanGain) * newPosXUncert;

    // update Y velocity
    residual = vSpeedY - newVelY;
    innovCov = newVelYUncert + 0.05;
    kalmanGain = newVelYUncert / innovCov;
    newVelY = newVelY + kalmanGain * residual;
    newVelYUncert = (1 - kalmanGain) * newVelYUncert;

    // update Y position
    residual = ((vSpeedY * dt) + this->yPosition) - newPosY;
    innovCov = newPosYUncert + 0.05;
    kalmanGain = newPosYUncert / innovCov;
    newPosY = newPosY + kalmanGain * residual;
    newPosYUncert = (1 - kalmanGain) * newPosYUncert;

    // after filtering, set model values equal to updates
    this->xVelocity = newVelX;
    this->xPosition = newPosX;
    this->xPosUncert = newPosXUncert;
    this->xVelUncert = newVelXUncert;
    this->xCovariance = newXCovariance;

    this->yVelocity = newVelY;
    this->yPosition = newPosY;
    this->yPosUncert = newPosYUncert;
    this->yVelUncert = newVelYUncert;
    this->yCovariance = newYCovariance;
}

float PositionTracking::voltageToSpeed(float voltage) {
    // returns 0.000235 to represent 0.235 meters per second, the approximate speed of the car as measured
    // may need to increase some, since the car didn't go in a straight line exactly
    return 0.000235;
}
