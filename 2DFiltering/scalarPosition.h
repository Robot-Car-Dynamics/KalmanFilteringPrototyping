#ifndef POSITIONTRACKING
#define POSITIONTRACKING

/* 
A class to keep track of the robot's position.
This version performs Kalman filtering through scalarized calculations
*/


class PositionTracking {
public:
    PositionTracking(float posX, float velX, float posXUncert, float velXUncert, float posY, float velY, float posYUncert, float velYUncert, float accelNoise);
    void updatePosition(float headingDeg, float accel, int dt, float voltage);
    float voltageToSpeed(float voltage);
    float getPosX();
    float getPosY();
    float getVelX();
    float getVelY();
private:
    float xPosition, xVelocity, xPosUncert, xVelUncert, yPosition, yVelocity, yPosUncert, yVelUncert, velMagnitude, accelNoise, xCovariance, yCovariance;
};


#endif