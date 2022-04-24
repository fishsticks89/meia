#include "main.h"
meia::Pid drive_pid (200, 0, 90);
Motor lift(1, 9000.0, true);
pros::Motor take(9);

void setupMotors() {
    std::cout << "lifttemp: " << lift.get_temp() << std::endl;
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    lift.set_fac(lift.get_motor_fac(36, (60.0/12.0)));
}

Pneumatic clamp('A', true);
Pneumatic mogo('F', false);
Pneumatic shtick('D', false);
Pneumatic hpost('G', false);