#include "main.h"
meia::Drive dogo(
    // Driving
    {20, -17},             // left motor ports
    {-19, 18},             // right motor ports
    4.125,                 // wheel diameter (inches)
    200,                   // motor rpm
    2.333,                 // gear ratio
    meia::Pid(17.5, 0, 0), // Drive PID constants

    // Turning
    9,                  // IMU port
    meia::Pid(1, 0, 0), // Turn PID Constants

    // Options
    5 // delay time
);
pros::Controller con(pros::E_CONTROLLER_MASTER);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::delay(500);
    dogo.tare(); // resets imu
    pros::lcd::initialize();
    pros::lcd::set_text(0, "meia - A PROS library for");
    pros::lcd::set_text(1, "creating reliable autons with");
    pros::lcd::set_text(2, "beginners in mind, and Worlds");
    pros::lcd::set_text(3, "on the horizon.");
    // tasks go here
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    pros::delay(200);
    dogo.tare();
    pros::lcd::set_text(4, std::to_string(dogo.get_motor_temps().first[0]));
    dogo.move(
        meia::drive, // the action to take (turn/drive)
        1,         // the total distance to g o (in)
        1,           // The speed to go at max (1 inch per second)
        meia::Curve( // accel curve
            1,       // max acceleration (in/sec^2 ; zero if no curve)
            0,       // the speed to accelerate from (0 in/sec)
            15       // anti-jerk percent, the percent of the curve that is rounded
            ),
        meia::Curve( // decel curve
            1,       // max deceleration in in/sec^2
            0.5,     // the speed to decelerate to (0.5 in/sec)
            15       // anti-jerk percent, the percent of the curve that is rounded
            ));
    pros::delay(50000);
}

/**
 * If no competition control is connected, this function will run immediately
 * following initialize(), otherwise it will run after opcontrol is enabled on the field switch.
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    while (true) {
        dogo.tank_control(con, pros::E_MOTOR_BRAKE_COAST, 3);
        pros::delay(5);
    }
}
