#include "main.h"
pros::Motor lift(1, true);
pros::Motor take(9);

void setupMotors() {
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

Pneumatic clamp('A', false);