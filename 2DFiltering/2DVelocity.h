#ifndef TWODVELOCITY
#define TWODVELOCITY

#include<cmath>
namespace motion {
    inline void velPerAxis(float speed, float headingDeg, float& velX, float& velY) {
        float radians = (3.14159 / 180.0) * headingDeg;
        velX = speed * std::cos(radians); // flip these assignments if heading is displacement from north, rather than from x axis
        velY = speed * std::sin(radians);
    }
}

#endif