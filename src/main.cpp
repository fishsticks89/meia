#include "main.h"

pros::Controller con(pros::E_CONTROLLER_MASTER);
meia::ChassisController drive(
    // Driving
    {-2, -5, -4},          // left motor ports
    {7, 8, 10},            // right motor ports
    2.7,                   // wheel diameter (inches)
    600,                   // motor rpm
    (48.0 / 36.0),         // gear ratio
    meia::Pid(200, 0, 90), // drive PID constants

    // Options
    5 // delay time
);

void initialize() {
    pros::delay(200);
    drive.tare();
    console.init();
    // die
    // pros::screen::touch_callback([](int16_t x, int16_t y) { exit(0); }, pros::E_TOUCH_PRESSED);
    setupMotors();
    pros::Task die([](void* p) {
        pros::delay(100);
        std::cout << "dtemp: " << drive.get_motor_temps().first[0] << std::endl;
        // ControlScheme control(&con);
        // control.addButton(pros::E_CONTROLLER_DIGITAL_UP, []() {
        //     // die
        //     // exit(0);
        // });
        // while (true) {
        //     control();
        //     pros::delay(5);
        // }
        pros::delay(3000);
        autons.log();

        while (true) {
            pros::delay(8000);
            console.init();
        }
    },
        nullptr, "water");
    pros::screen::touch_callback([](int16_t x, int16_t y) {
        switch (console.to_button(x, y)) {
        case meia::left:
            autons.switch_left();
            break;
        case meia::right:
            autons.switch_right();
            break;
        case meia::center:
            console.log("calibrating imu");
            pros::delay(20);
            imu.calibrate();
            console.log("imu calibrated");
            break;
        }
    },
        pros::E_TOUCH_PRESSED);
}

void disabled() {
    drive.tare();
}

void competition_initialize() {
}

void autonomous() {
    awaitCalibrate();
    imu.zero();
    drive.tare();
    autons.run(&drive);
}

void opcontrol() {
    ControlScheme control(&con);
    control.addDirectional(
        {pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L2},
        []() { lift.set_voltage(12000); },  // if first button is pressing
        []() { lift.set_voltage(-12000); }, // if second button is pressing
        []() { lift.set_voltage(0); }       // if neither are pressing
    );
    control.addDirectional(
        {pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_DOWN},
        []() { take.move_voltage(12000); },  // if first button is pressing
        []() { take.move_voltage(-12000); }, // if second button is pressing
        []() { take.move_voltage(0); }       // if neither are pressing
    );
    mogo.deactivate();
    control.addButton(pros::E_CONTROLLER_DIGITAL_X, []() { mogo.toggle(); });       // sets the mogo to toggled state
    control.addButton(pros::E_CONTROLLER_DIGITAL_R2, []() { clamp.toggle(); });     // sets the clamp to toggled state
    control.addButton(pros::E_CONTROLLER_DIGITAL_A, []() { shtick.toggle(); });     // sets the shtick to toggled state
    control.addButton(pros::E_CONTROLLER_DIGITAL_RIGHT, []() { shtick.toggle(); }); // sets the shtick to toggled state
    control.addButton(pros::E_CONTROLLER_DIGITAL_Y, []() { hpost.toggle(); });      // sets the high post mech to toggled state
    drive.set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
    control.addToggleable(pros::E_CONTROLLER_DIGITAL_B, [](bool act) { drive.set_drive_brake((act) ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_COAST); });
    while (true) {
        control();
        drive.tank_control(&con, pros::E_MOTOR_BRAKE_COAST, 3.2); // controller, brake mode, curve intensity
        pros::delay(5);
    }
}