#include "scalarPosition.h"

PositionTracking::PositionTracking(float posX = 0, float velX = 0, float posXUncert = 1, float velXUncert = 1) {
    this->xPosition = posX;
    this->xVelocity = velX;
    this->xPosUncert = posXUncert;
    this->xVelUncert = velXUncert;
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
    float newPosX = this->xPosition + (this->xVelocity + (0.5 * accel * dt * dt)); // this averages old and new accel in position update


}