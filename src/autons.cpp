#include "main.h"

const double drive_width = 14.1;

void goalrush(meia::ChassisController* drive) {
    shtick.activate();
    std::cout << meia::p::debting_go(drive, 34, 70, 240) << std::endl;
    std::cout << meia::p::debting_go(drive, -34, 70, 240) << std::endl;
}
void goalrushrings(meia::ChassisController* drive) {
    shtick.activate();
    // shtick nmogo
    std::cout << meia::p::debting_go(drive, 34, 60, 150) << std::endl;
    std::cout << meia::p::debting_go(drive, -34, 60, 90) << std::endl;
    drive->set_pid_constants(drive_pid);
    lift.set_voltage(-4000);
    clamp.deactivate();
    shtick.deactivate();
    pros::delay(200);
    // get nmogo in clamp
    meia::p::turn(drive, 10, 30, 100, drive_width);
    meia::p::debting_go(drive, 16.5, 50, 240);
    clamp.activate();
    lift.set_target(20);
    pros::delay(200);
    // get rmogo in
    meia::p::turn(drive, -100, 140, 300, drive_width);
    lift.set_voltage(-4000);
    meia::p::debting_go(drive, -18, 30, 240);
    pros::delay(200);
    mogo.activate();
    lift.set_target(40);
    meia::p::debting_go(drive, 3, 30, 6000);
    meia::p::turn(drive, 91, 90, 360, drive_width);
    lift.set_target(50);
    take.move_voltage(12000);
    meia::p::debting_go(drive, 45, 13, 50);
    meia::p::debting_go(drive, -50, 70, 180);
}

void auton(meia::ChassisController* drive) {
    goalrushrings(drive);
    clamp.deactivate();
}