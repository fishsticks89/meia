#include "main.h"
namespace meia {
    /**
     * A Chassis that you can set the target position of each side and get error on
     *  \param left_motors
     *      Vector of left side's motor ports, negative if reversed
     *  \param right_motors
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
     *      External drive ratio in wheel revolutions / motor revolution.
     *      If your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
     *      If your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
     *  \param pid
     *      The pid controller to correct the drive wheels with
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
                    Pid_task_messenger_struct(Chassis* chassis_ptr, Pid pid, double ticks_per_inch, int drive_task_delay_factor)
                        : chassis_ptr{chassis_ptr},
                          delta_time{drive_task_delay_factor},
                          ticks_per_inch{ticks_per_inch},
                          p{pid.p},
                          i{pid.i},
                          d{pid.d} {
                    }
                    Chassis* chassis_ptr; // a pointer to the chassis the task controls
                    pros::Mutex mutex;    // the mutex to hold while modifying data
                    int delta_time;
                    double ticks_per_inch; // motor.get_position()/inch
                    double p;                     // the Proportional gain of the pid controller
                    double i;                     // Integral gain
                    double d;                     // Derivative gain
                    double total_error;           // the total error experienced
                    double left_target = 0;       // target in inches of the motor
                    double right_target = 0;      // target in inches of the motor
                    bool reset = true;
                    bool allowderivative = true;
            } pid_task_messenger;          // an instance of the pid task messenger struct used to commuicate with the pid task
            Chassis chassis;               // the chassis the controller controls
            pros::Task pid_loop_task;      // the task that controls the chassis
            static void pid_loop(void* p); // the function the task uses to control the chassis

        public:
            explicit ChassisController(std::vector<int> left_motors, std::vector<int> right_motors, double wheel_diameter, int motor_rpm, double gear_ratio, Pid pid, int delay_time = 5)
                : chassis(left_motors, right_motors),
                  pid_task_messenger(&chassis, pid, (
                    // ticks per revolution
                    // ticks per internal gear revolution * interior gear ratio * exterior gear ratio
                    ((50.0 * (3600.0 / motor_rpm)) * gear_ratio) // with no cart, the encoder reads 50 counts per rotation, 3600.0/motor_rpm is = internal_gear_ratio
                    /
                    // inches per revolution
                    (wheel_diameter * m_pi) // 2nd grade math
                    ),
                      delay_time),
                  pid_loop_task(pid_loop, &pid_task_messenger, "pid_task"){};
            // a function to change the targets of the pid task
            void change_target(std::pair<double, double> deltatarget);
            // a function to change the correctional constants on the controller for the drive
            void set_pid_constants(double p, double i, double d);
            double get_p_constant();
            // a function to get the total error of the chassis
            double get_total_error();
            // expose functions from meia::Chassis
            void tank_control(pros::Controller* con = new pros::Controller(pros::E_CONTROLLER_MASTER), pros::motor_brake_mode_e_t brake_mode = pros::E_MOTOR_BRAKE_COAST, double curve_intensity = 0, int deadzone = 0);
            std::pair<std::vector<double>, std::vector<double>> get_motor_temps();
            void set_drive_brake(pros::motor_brake_mode_e_t input);
            void tare();
            int get_delay_time();
            void allowderivative(bool allow);
            std::pair<double, double> get_error();
    };
} // namespace meia