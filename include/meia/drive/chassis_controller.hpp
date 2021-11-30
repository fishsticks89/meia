#include "main.h"
namespace meia {
    /**
     * A Chassis that you can set the target position of each side and get error on
     *  \param left_motors
     *      Vector of left side's motor ports, negative if reversed
     *  \param left_motors
     *      Vector of right side's motor ports, negative if reversed
     *  \param wheel_diameter
     *      Remember that 4" wheels are actually 4.125"!
     *      Have the robot go 8ft forward and adjust this value until the robot actually goes 8ft
     *  \param motor_rpm
     *      Output RPM of the cartrage inside of your motor.
     *      Turbo (Blue) - 600;
     *      Speed (Green) - 200;
     *      Torque (Red) - 100;
     *  \param gear_ratio
     *      External drive ratio (MUST BE DECIMAL).
     *      If your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
     *      If your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
     *  \param p
     *      The p gain to correct the drive wheels with
     *  \param i
     *      The i gain to correct the drive wheels with
     *  \param d
     *      The d gain to correct the drive wheels with
     *  \param delay_time
     *      The delay between executions of the pid controller
     */
    class ChassisController {
    private:
        struct Pid_task_messenger_struct {
            /**
             * Struct used to pass messages to pid_task
             * \param chassis_ptr
             *      a pointer to a chassis for the PID task to control
             * \param drive_task_delay_factor
             *      how many milliseconds the drive waits
            */
            Pid_task_messenger_struct(Chassis* chassis_ptr, double p, double i, double d, double ticks_per_inch, int drive_task_delay_factor) :
                chassis_ptr{ chassis_ptr },
                delta_time{ drive_task_delay_factor },
                ticks_per_inch{ ticks_per_inch },
                p{ p }, i{ i }, d{ d } {}
            Chassis* chassis_ptr; // a pointer to the chassis the task controls
            pros::Mutex mutex; // the mutex to hold while modifying data
            int delta_time;
            double ticks_per_inch; // motor.get_position()/inch
            double p; // the Proportional gain of the pid controller
            double i; // Integral gain
            double d; // Derivative gain
            double total_error; // the total error experienced
            double left_target = 0; // target in inches of the motor
            double right_target = 0; // target in inches of the motor
            bool reset = false;
        };
        Pid_task_messenger_struct pid_task_messenger; // an instance of the pid task messenger struct used to commuicate with the pid task
        Chassis chassis; // the chassis the controller controls
        pros::Task pid_loop_task; // the task that controlls the chassis
        static void pid_loop(void* p); // the function the task uses to control the chassis
        static std::pair<double, double> normalize(double l_volt, double r_volt, double max); // a function that scales drive voltages down to a maximum
        static std::pair<double, std::pair<double, double>> pid(double current, double target, double p, double i, double d, std::pair<double, double> prev, int delta_time);
    public:
        explicit ChassisController(std::vector<int> left_motors, std::vector<int> right_motors, double wheel_diameter, int motor_rpm, double gear_ratio, double p, double i, double d, int delay_time = 5) :
            chassis(left_motors, right_motors),
            pid_task_messenger(&chassis, p, i, d, (
                // ticks per inch
                ((50 * (3600 / motor_rpm))* gear_ratio) // Ticks per revolution
                /
                (wheel_diameter * M_PI) // Circumference of wheel
                ), delay_time),
            pid_loop_task(pid_loop, &pid_task_messenger, "pid_task")
        {};
        // a function to change the targets of the pid task
        void change_target(double l, double r);
        // a function to change the correctional constants on the controller for the drive
        void set_pid_constants(double p, double i, double d);
        // a function to get the total error of the chassis
        double get_total_error();
        // expose functions from meia::Chassis
        void tank_control(pros::Controller con, pros::motor_brake_mode_e_t brake_mode, double curve_intensity = 0, int deadzone = 0);
        std::pair<std::vector<double>, std::vector<double>> get_motor_temps();
        void set_drive_brake(pros::motor_brake_mode_e_t input);
        void tare();
        void end();
    };
}