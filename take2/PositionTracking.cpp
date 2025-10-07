#include"PositionTracking.h"

/*
This version uses BasicLinearAlgebra library
https://github.com/tomstewart89/BasicLinearAlgebra
*/
PositionTracking::PositionTracking(int posX = 0, int velX = 0) {
    stateVector << posX, velX; // initial position and velocity are 0
    covariance << 1, 0,
                  0, 1; // initial uncertainty is small
    
    stateTransition << 1, 
}


void PositionTracking::updatePosition(int accel, int timeDiff) {
    // make prediction
    // statePred = stateTransition * stateVector + (controlMatrix * accel)
    // uncertPred = stateTransition * uncertainty * stateTransition.T + processNoise

    // update
    // NOTE: measurment matrix differs depending on which model variable I'm measuring?
    // surprise = accel - measurementMatrix * statePred
    // sumUncert = measurementMatrix * uncertPred * measurementMatrix.T + measurementNoise
    // kalmanGain = uncertPred * measurementMatrix.T * sumUncert^-1
    // stateUpdate = statePred + kalmanGain * surprise
    // uncertUpdate = (identityMatrix - kalmanGain * measurementMatrix) * uncertPred
}

std::pair<int, int> PositionTracking::getLocSpe() {
    return {positionX, velocityX};
}