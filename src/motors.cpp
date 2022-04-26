#include "main.h"
meia::Pid drive_pid (200, 0, 90);
Imu imu (11);
Motor lift(1, true, meia::Pid(1500.0, 0, -90000));
pros::Motor take(9);
bool imucalibrated = false;

void awaitCalibrate() {
    while (!imucalibrated) {
        pros::delay(500);
    }
}

void setupMotors() {
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    lift.set_fac(lift.get_motor_fac(36, (60.0/12.0)));
    lift.tare();
    pros::Task([]() { 
        imu.calibrate(); imucalibrated = true;
        std::cout << "lifttemp: " << lift.motor.get_temperature() << std::endl;
    });
}

Pneumatic clamp('A', true);
Pneumatic mogo('F', true);
Pneumatic shtick('D', false);
Pneumatic hpost('G', false);