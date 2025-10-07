#ifndef POSITIONTRACKING
#define POSITIONTRACKING

/* 
A class to keep track of the robot's position.
This version trusts the accelerometer implicitly and does no filtering
Positions reported by this class may actually be true, depending on the reliability of the accelerometer used
*/


class PositionTracking {
public:
    PositionTracking(float posX, float velX);
    ~PositionTracking(); // currently unused
    void updatePosition(float accel, int dt);
    float getPosX();
    float getVelX();
private:
    float xPosition, xVelocity;
};


#endif