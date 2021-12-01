#include "main.h"
namespace meia {
    enum class Axis { x = 0, y = 1, z = 2};
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
    class Drive {
    private:
        struct Profiling_task_messenger_struct {
            /**
             * Struct used to pass messages to pid_task
             * \param chassis_ptr
             *      a pointer to a chassis for the Profiling task to control
             * \param drive_task_delay_factor
             *      how many milliseconds the drive waits
            */
            Profiling_task_messenger_struct(ChassisController* chassis_ptr, pros::Imu* imu, double p, double i, double d, double max_speed, int drive_task_delay_factor = 5) :
                chassis_ptr{ chassis_ptr },
                delta_time{ drive_task_delay_factor },
                max_speed{ max_speed },
                imu{imu},
                p{ p }, i{ i }, d{ d } {}
            ChassisController* chassis_ptr; // a pointer to the chassis the task controls
            pros::Imu* imu; // the imu to use for course correction
            pros::Mutex mutex; // the mutex to hold while modifying data
            int delta_time;
            double max_speed; // motor.get_position()/inch
            double p; // the Proportional gain of the pid controller
            double i; // Integral gain
            double d; // Derivative gain
            double total_error; // the total error experienced
            double target = 0; // rotational target in degrees
            bool reset = false;
        };
        Profiling_task_messenger_struct profiling_task_messenger; // an instance of the pid task messenger struct used to commuicate with the pid task
        ChassisController chassis; // the chassis the controller controls
        pros::Task profile_loop_task; // the task that controlls the chassis
        static void pid_loop(void* p); // the function the task uses to control the chassis
        static std::pair<double, double> normalize(double l_volt, double r_volt, double max); // a function that scales drive voltages down to a maximum
        static std::pair<double, std::pair<double, double>> pid(double current, double target, double p, double i, double d, std::pair<double, double> prev, int delta_time);
    public:
        explicit Drive(std::vector<int> left_motors, std::vector<int> right_motors, double wheel_diameter, int motor_rpm, double gear_ratio, double p, double i, double d, pros::Imu imu, double turn_p, double turn_i, double turn_d, int delay_time = 10) :
            chassis(left_motors, right_motors, wheel_diameter, motor_rpm, gear_ratio, p, i, d, delay_time),
            profiling_task_messenger(&chassis, &imu, turn_p, turn_i, turn_d, (
                (motor_rpm * gear_ratio) * (wheel_diameter * M_PI) // max speed
                ), delay_time),
            profile_loop_task(pid_loop, &profiling_task_messenger, "pid_task")
        {};
        // a function to change the target angle of the pid task
        void turn(double target);
        // a function to change the correctional constants on the controller for the drive
        void set_drive_pid_constants(double p, double i, double d);
        // a function to change the correctional constants on the controller for the drive
        void set_turn_pid_constants(double p, double i, double d);
        // a function to get the total error of the chassis
        double get_total_error();
        // expose functions from meia::Chassis
        void tank_control(pros::Controller con, pros::motor_brake_mode_e_t brake_mode, double curve_intensity = 0, int deadzone = 0);
        std::pair<std::vector<double>, std::vector<double>> get_motor_temps();
        void tare();
        void end();
    };
}