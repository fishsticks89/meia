#include "main.h"
meia::ChassisController dogo({ 18,-19 }, { 16,-17 }, 4.125, 200, 2.333, 4, 2, 0);
pros::Controller mainish(pros::E_CONTROLLER_MASTER);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::delay(500);
	dogo.tare(); // no reason, just yes
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
void competition_initialize() {}

void bad() {
	
	dogo.tare();
	for (int i = 0; i < 7; i++)
	{
		// test pid
		// dogo.set_pid_constants((i + 1), 0, 0);
		// for (int suba = 0; suba < 100; suba++) {
		// 	dogo.change_target(suba/-1000, suba/-1000);
		// 	pros::delay(10);
		// }
		pros::delay(2000);
		for (int subb = 0; subb < 200; subb++) {
			dogo.change_target(0.1, 0.1);
			pros::delay(10);
		}
		pros::delay(700);
		printf(std::to_string(dogo.get_total_error()).c_str());
		mainish.print(0, 0, (std::to_string(i) + ": " + std::to_string(std::round(dogo.get_total_error()))).c_str());
		// return to beginning
		dogo.set_pid_constants(5, 0, 0);
		for (int subd = 0; subd < 200; subd++) {
			dogo.change_target(-0.1, -0.1);
			pros::delay(20);
		}
		pros::delay(700);
	}
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
	dogo.tare();
	pros::delay(2000);
	double fac = 1;
	while (true) {
		dogo.change_target(0.01 * fac, 0.03 * fac);
		pros::delay(5);
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	dogo.end();
	while (true) {
		dogo.tank_control(mainish, pros::E_MOTOR_BRAKE_COAST, 3);
		pros::delay(5);
	}
}
