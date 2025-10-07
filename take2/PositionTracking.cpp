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
    
}

std::pair<int, int> PositionTracking::getLocSpe() {
    return {positionX, velocityX};
}