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
     */
    class ChassisController: private Chassis, private pros::Task {
        private:        
            double ticks_per_inch;
            std::pair<double, double> set_voltage_normalized(double l_volt, double r_volt);
            typedef struct Pid_task_messenger_struct {
                Pid_task_messenger_struct(int p_drive_task_delay_factor) : drive_task_delay_factor{p_drive_task_delay_factor} {}
                int drive_task_delay_factor = 1;
            };
            static void pid_loop(void* p);
            int pid_task_messenger;
        public:
            explicit ChassisController(std::vector<int> left_motors, std::vector<int> right_motors, double wheel_diameter, int motor_rpm, double gear_ratio, int delay_time = 1) : 
            Chassis(left_motors, right_motors),
            ticks_per_inch(
                ((50*(3600/motor_rpm)) * gear_ratio) // Ticks per revolution
                    /
                (wheel_diameter * M_PI) // Circumference of wheel
            ),
            pid_task_messenger(delay_time),
            pros::Task (pid_loop, &pid_task_messenger, "pid_task")
            {};
            // uses tank control and get_temps function from meia::Chassis
            using Chassis::tank_control;
            using Chassis::get_motor_temps;
    };
}