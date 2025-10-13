#ifndef POSITIONTRACKING
#define POSITIONTRACKING

/* 
A class to keep track of the robot's position.
This version performs Kalman filtering through scalarized calculations
*/


class PositionTracking {
public:
    PositionTracking(float posX, float velX, float posXUncert, float velXUncert, float accelNoise);
    void updatePosition(float accel, int dt, float voltage);
    float voltageToSpeed(float voltage);
    float getPosX();
    float getVelX();
private:
    float xPosition, xVelocity, xPosUncert, xVelUncert, accelNoise, xCovariance;
};


#endif