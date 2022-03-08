#include "main.h"
bool dev = false; // false if working on real robot
pros::Controller con(pros::E_CONTROLLER_MASTER);
meia::Drive dogo(
    // Driving
    {20, -17},             // left motor ports
    {-19, 18},             // right motor ports
    4.125,                 // wheel diameter (inches)
    200,                   // motor rpm
    2.333,                 // gear ratio
    meia::Pid(17.5, 0, 0), // Drive PID constants

    // Turning
    16,                 // IMU port
    meia::Pid(1, 0, 0), // Turn PID Constants

    // Options
    5 // delay time
);

// meia::Console console;

void initialize() {
    pros::delay(600);
    std::cout << "we made it this far" << std::endl;
    dogo.init_imu(); // resets imu
    std::cout << "imu!" << std::endl;
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    std::cout << "disbale?????" << std::endl;
    dogo.end();
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
    std::cout << "auton!" << std::endl;
    pros::delay(200);
    dogo.tare();
    // pros::lcd::set_text(4, std::to_string(dogo.get_motor_temps().first[0]));
    pros::delay(8000);
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
    std::cout << "op!" << std::endl;
    // while (true) {
    //     dogo.tank_control(con, pros::E_MOTOR_BRAKE_COAST, 3);
    //     pros::delay(5);
    // }
    std::cout << "start: " << std::endl;
    dogo.move(
        meia::advance, // the action to take (turn/advance)
        30,            // the total distance to go (in)
        1,             // The speed to go at max (1 inch per second)
        1,
        1);
    std::cout << "end: ";
}
