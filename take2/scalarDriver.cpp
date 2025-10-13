#include "scalarPosition.h"
#include <queue>
#include <tuple>
#include <iostream>
#include <windows.h> // NOTE: only works on windows, linux equivalent is unistd.h. This is for the sleep() call

using namespace std;

int main() {
    PositionTracking scalarPos = PositionTracking(0, 0, 0, 0, 0.25);

    // create vector of accel, dt, voltage values
    queue<tuple<float, int, float>> updateVals = queue<tuple<float, int, float>>();

    tuple<float, int, float> start = { 0.0, 0, 0.0 }; // stopped
    updateVals.push(start);

    tuple<float, int, float> move = { 0.00134, 500, 1}; // voltage number irrelevant in this case, due to dummy function
    updateVals.push(move);

    for (int i = 0; i < 150; i++) {
        updateVals.push(tuple<float, int, float>({0, 500, 1})); // adds zero acceleration updates. Should have constant speed.
    }

    cout << "updateVals length: " << updateVals.size() << endl;

    while(!updateVals.empty()) {
        Sleep(500); // sleep 500ms, need usleep(microseconds) in unix
        tuple<float, int, float> element = updateVals.front();
        updateVals.pop();
        float accel = get<0>(element);
        int dt = get<1>(element);
        float voltage = get<2>(element);

        cout << "accel: " << accel << " dt: " << dt << " voltage: " << voltage << endl;
        scalarPos.updatePosition(accel, dt, voltage);

        float position = scalarPos.getPosX();
        float velocity = scalarPos.getVelX();

        cout << "predicted position (m): " << position << " predicted velocity: " << velocity << endl;
    }
}