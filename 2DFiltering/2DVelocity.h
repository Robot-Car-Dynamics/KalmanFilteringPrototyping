#ifndef TWODVELOCITY
#define TWODVELOCITY

#include<cmath>
namespace motion {
    inline void velPerAxis(double speed, double headingDeg, double& velX, double& velY) {
        double radians = (3.14159 / 180.0) * headingDeg;
        velX = speed * std::cos(radians); // flip these assignments if heading is displacement from north, rather than from x axis
        velY = speed * std::sin(radians);
    }
}

#endif