#include "dummyPosition.h"

PositionTracking::PositionTracking(float posX = 0, float velX = 0) {
    this->xPosition = posX;
    this->xVelocity = velX;
}

float PositionTracking::getPosX() {
    return this->xPosition;
}

float PositionTracking::getVelX() {
    return this->xVelocity;
}

void PositionTracking::updatePosition(float accel, int dt) {
    // updates position and velocity in model
    // NOTE: ensure that acceleration and dt are in the same units of time to avoid magnitude errors
    float newVelX = this->xVelocity + (accel * dt);
    float newPosX = this->xPosition + (newVelX * dt);

    this->xPosition = newPosX;
    this->xVelocity = newVelX;
}

