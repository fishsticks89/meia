#include "main.h"
pros::Controller con(pros::E_CONTROLLER_MASTER);
meia::ChassisController drive(
    // Driving
    {-2, -5, -4},         // left motor ports
    {7, 8, 10},           // right motor ports
    2.75,                 // wheel diameter (inches)
    600,                  // motor rpm
    2.333,                // gear ratio
    meia::Pid(200, 50, 100), // drive PID constants

    // // Turning
    // 16,                 // IMU port
    // meia::Pid(1, 0, 0), // Turn PID Constants

    // Options
    5 // delay time
);

void initialize() {
    setupMotors();
    drive.tare();
}

void disabled() {
    drive.tare();
}

void competition_initialize() {
}

void autonomous() {
    drive.tare();
    goalrush(&drive);
}

void opcontrol() {
    autonomous();
    pros::delay(800000);
    ControlScheme control(&con);
    control.addDirectional(
        {pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L2},
        []() { lift.move_voltage(12000); },  // if first button is pressing
        []() { lift.move_voltage(-12000); }, // if second button is pressing
        []() { lift.move_voltage(0); }       // if neither are pressing
    );
    control.addDirectional(
        {pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_DOWN},
        []() { take.move_voltage(12000); },  // if first button is pressing
        []() { take.move_voltage(-12000); }, // if second button is pressing
        []() { take.move_voltage(0); }       // if neither are pressing
    );
    control.addToggleable(pros::E_CONTROLLER_DIGITAL_R2, [](bool act) { clamp.set(act); }); // sets the clamp to toggled state
    control.addToggleable(pros::E_CONTROLLER_DIGITAL_X, [](bool act) { mogo.set(act); });   // sets the mogo to toggled state
    control.addToggleable(pros::E_CONTROLLER_DIGITAL_A, [](bool act) { shtick.set(act); }); // sets the shtick to toggled state
    while (true) {
        control();
        drive.tank_control(con, pros::E_MOTOR_BRAKE_COAST, 3.2); // controller, brake mode, curve intensity
        pros::delay(5);
    }
}