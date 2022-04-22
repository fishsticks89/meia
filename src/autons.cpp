#include "main.h"

void goalrush(meia::ChassisController* drive) {
    std::cout << "autotime" << std::endl;
    clamp.deactivate();
    std::cout << meia::p::debting_go(drive, 8, 20, 90) << std::endl;
}