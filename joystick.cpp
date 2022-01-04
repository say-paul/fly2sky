// function so sensitivity graph
// f(x) = (atan(0.2x - k))/10
// period = -5pi/2 to 5pi/2
// a = sensitivity input from 1 - 33
// k = shifting co-efficenent incase of any error bias
// map joystick input between -2pi and 2pi
// so f(x) ranges between -0.5/0.5 and -10/10 for various sensistivity 

#include <math.h>
#include "config.hpp"

tuning Tune;

float transform(short int &x) {
    return 0.1 * Tune.joystickSensitivity * tan(0.2 * x); //k is set to 0.
}


void main() {
    Tune.joystickSensitivity = 4; //
    Tune.obstructionWarning = 150; //unit in cm
    Tune.maxCruiseVelocity = 10; //unit in km/h
    Tune.maxAcceleration = 1; //unit in m/s^2
}