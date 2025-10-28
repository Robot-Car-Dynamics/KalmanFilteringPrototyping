#include "2DVelocity.h"
#include "scalarPosition.h"
#include <queue>
#include <tuple>
#include <iostream>
#include <windows.h> // NOTE: only works on windows, linux equivalent is unistd.h. This is for the sleep() call
#include <cstdlib> // contains rand()

using namespace std;

int main() {
    // PositionTracking(float posX, float velX, float posXUncert, float velXUncert, float posY, float velY, float posYUncert, float velYUncert, float accelNoise);
    PositionTracking scalarPos = PositionTracking(0, 0, 0, 0, 0, 0, 0, 0, 0.25);

    float heading = 285;

    // create vector of accel, dt, voltage values
    queue<tuple<float, float, int, float>> updateVals = queue<tuple<float, float, int, float>>();

    tuple<float, float, int, float> start = { heading, 0.0, 0, 0.0 }; // stopped
    updateVals.push(start);

    tuple<float, float, int, float> move = { heading, 0.00134, 500, 1}; // voltage number irrelevant in this case, due to dummy function
    updateVals.push(move);

    for (int i = 0; i < 150; i++) {
        int randomOffset = rand() % 10; // makes a random int between 0 and 9
        float offsetFloat = 0.1 * randomOffset;

        updateVals.push(tuple<float, float, int, float>({heading, offsetFloat, 500, 1})); // adds zero acceleration updates. Should have constant speed.
    }

    cout << "updateVals length: " << updateVals.size() << endl;

    while(!updateVals.empty()) {
        Sleep(500); // sleep 500ms, need usleep(microseconds) in unix
        tuple<float, float, int, float> element = updateVals.front();
        updateVals.pop();
        float heading = get<0>(element);
        float accel = get<1>(element);
        int dt = get<2>(element);
        float voltage = get<3>(element);

        cout << "heading: " << heading << " accel: " << accel << " dt: " << dt << " voltage: " << voltage << endl;
        scalarPos.updatePosition(heading, accel, dt, voltage);

        float positionX = scalarPos.getPosX();
        float positionY = scalarPos.getPosY();
        float velocityX = scalarPos.getVelX();
        float velocityY = scalarPos.getVelY();

        cout << "predicted position X, Y (m): " << positionX << ", " << positionY << " predicted velocity X, Y: " << velocityX << ", " << velocityY << endl;
    }
    return 1;
}