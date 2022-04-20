#include "main.h"

void goForDistance(meia::ChassisController* drive, double distance) {
    meia::p::go(drive, 24, 8, 8);
}

void goalrush(meia::ChassisController* drive) {
    std::cout << "autotime" << std::endl;
    clamp.deactivate();
    goForDistance(drive, 30);
    pros::delay(10000);
    // clamp.activate();
    // goForDistance(drive, -50);
}