#include"PositionTracking.h"

void PositionTracking::updatePosition(int speed, int accel, int timeDiff) {
    // the following represents updating the model
    // update based on previous state
    int predictedPositionX = positionX + velocityX * timeDiff;
    int predictedVelocityX = velocityX;

    // update based on measured acceleration
    predictedPositionX += 0.5 * accel * (timeDiff * timeDiff);
    predictedVelocityX += accel * timeDiff;

    // increase uncertainty due to model/process noise
    posXUncertainty += processNoisePosX;
    velXUncertainty += processNoiseVelX;

    // calculate kalman gain 
    float kalmanGain = posXUncertainty / (posXUncertainty + measurementNoise);

    // find z using existing gausian distribution random number generator
    float z = positionX + dist(gen);

    // correct position
    positionX = predictedPositionX + kalmanGain * (z - predictedPositionX);

    // correct velocity (indirectly)
    velocityX = predictedVelocityX + kalmanGain * (z - predictedPositionX) / timeDiff;

    // reduce uncertainty after measurement
    posXUncertainty = (1 - kalmanGain) * posXUncertainty;
    velXUncertainty = (1 - kalmanGain) * velXUncertainty;
}

PositionTracking::PositionTracking(int posx, int velX) {
    positionX = posx;
    velocityX = velX;

    // generate random device for later use
    std::random_device rd;
    gen = std::mt19937(rd()); // Makes a Mersenne Twister Engine

    // make a gausian distribution with a mean of 0 and stddev of sqrt(measurementNoise)
    dist = std::normal_distribution<> (0.0, std::sqrt(measurementNoise));
}

std::pair<int, int> PositionTracking::getLocSpe() {
    return {positionX, velocityX};
}