#include"PositionTracking.h"
#include<vector>

using namespace std;

void PositionTracking::updatePosition(int accel, int timeDiff) {
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

void PositionTracking::updatePosition(int accel, int timeDiff) {
    // make initial estimate of model
    /*
    x_pred = F @ x_prev                - predicted state, will need to add control inputs in future
    P_pred = F @ P_prev @ F.T + Q      - predicted uncertainty

    F is vector model that predicts how the state moves forward
    Q is process noise vector, represents uncertainty of model
    */
    // modify with new measurements to make final estimate
    // calculate Kalman gain (between -1 and 1)
    // correct prediction with Kalman gain and surprise term

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