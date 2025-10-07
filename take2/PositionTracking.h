#ifndef POSITIONTRACKING
#define POSITIONTRACKING
#include <utility>
#include <random>
#include <BasicLinearAlgebra.h> // need to put on include path in compiler

using namespace std;
// A class to keep track of the robot's position.
// A class is used because location and velocity need to be maintained for the entire uptime of the car
// Currently this only works in the X dimension and assumes all speeds are positive.

class PositionTracking {
public:
    PositionTracking(int posX, int velX, int dt);
    ~PositionTracking(); // not implemented for now
    void updatePosition(int accel, int timeDiff);
    std::pair<int, int> getLocSpe();
private:
    Matrix<2>    stateVector; // position, velocity
    Matrix<2,2>  uncertainty; // uncertainty of position, velocity
    Matrix<2,2>  controlMatrix; // converts accel into delta pos and delta v
    Matrix<2,2>  stateTransition;
    Matrix<2,2>  processNoise;
    Matrix<2,1>  controlInput;
};


#endif