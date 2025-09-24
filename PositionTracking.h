#ifndef POSITIONTRACKING
#define POSITIONTRACKING
#include <utility>
#include <random>


// A class to keep track of the robot's position.
// A class is used because location and velocity need to be maintained for the entire uptime of the car
// Currently this only works in the X dimension and assumes all speeds are positive.

class PositionTracking {
public:
    PositionTracking(int posx, int velX);
    ~PositionTracking(); // not implemented for now
    void updatePosition(int speed, int accel, int timeDiff);
    std::pair<int, int> getLocSpe();
private:
    // state consists of both position and velocity
    int positionX = 0;
    int velocityX = 0;

    // uncertainty in position and velocity. 
    // float between 0 and 1
    float posXUncertainty = 1.0;
    float velXUncertainty = 1.0;

    // uncertainty in model
    float processNoisePosX = 0.0001;
    float processNoiseVelX = 0.0001;

    // uncertainty in observations
    float measurementNoise = 0.1;

    std::normal_distribution<> dist; // gausian distribution object
    std::mt19937 gen; // Mersenne Twister Engine used for random numbers
};


#endif