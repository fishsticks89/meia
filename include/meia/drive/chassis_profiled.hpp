#include "main.h"
namespace meia {
    enum move_type_e
    {
        none,
        hold,
        drive,
        turn
    };
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
     *  \param drive_pid
     *      The pid controller to correct the drive wheels with
     *  \param imu_port
     *      The port of the inertial measurement unit that will be used to get the orientation of the robot.
     *      Note that whichever axis is parallel to the ground will be used automatically
     *  \param turn_pid
     *      the pid controller to correct the orientation of the drive with
     *  \param delay_time
     *      The delay between executions of the pid controller
     */
    class Drive {
        private:
            static util_funcs util;
            enum phase_e
            {
                acc,
                cruise,
                decel
            };
            class MovementInfo {
                public:
                    MovementInfo(move_type_e type = none, Curve start_curve = Curve(0), Curve end_curve = Curve(0), double max_speed = 0, double distance = 0, int id = 0, int delay_time = 100)
                        : start(start_curve), end(end_curve), max_speed(max_speed), distance(distance), type(type), id(id) {
                        // gets the iterations of the start curve if it exists
                        if (start_curve.acceleration != 0) {
                            start_curve_iterations = util.get_curve_iterations(
                                (                            // how often the curve will be sampled
                                    (delay_time / 1000.0)    // the ms interval the curve will be sampled at (converted to sec)
                                    *                        // the acceleration of the curve (the multiplier for the in/ms normal acceleration)
                                    start_curve.acceleration // the acceleration of the curve in in/s^2
                                    ),
                                // the robot only needs to accelerate the difference between
                                max_speed - start_curve.endpoint_speed, // max value output of the curve to be sampled
                                start_curve.antijerk_percent            // the antijerk percent of the curve to be sampled
                            );
                        } else {
                            start_curve_iterations = 0;
                        }
                        start_curve_distance = (util.get_curve_distance(
                                                    (delay_time / 1000.0) *                 // the curve can only be sampled as often as the delay_time of the loop (to sec)
                                                        start_curve.acceleration,           // multiplied by the acceleration factor in in/sec
                                                    max_speed - start_curve.endpoint_speed, // max value output of the curve to be sampled
                                                    start_curve.antijerk_percent) +
                                                   (start_curve_iterations * start_curve.endpoint_speed)) // the block of endpoint speed upon which the
                                               * delay_time / 1000.0;                                     // the curve and endpoint speed output must be multiplied by delay (in sec) time to ensure a consistant output across all delay times
                        if (end_curve.acceleration != 0) {
                            end_curve_iterations = util.get_curve_iterations(
                                (                          // how often the curve will be sampled
                                    (delay_time / 1000.0)  // the ms interval the curve will be sampled at (sec)
                                    *                      // the acceleration of the curve (the multiplier for the in/ms normal acceleration)
                                    end_curve.acceleration // the acceleration of the curve in in/s^2
                                    ),
                                // the robot only needs to accelerate the difference between
                                max_speed - end_curve.endpoint_speed, // max value output of the curve to be sampled
                                end_curve.antijerk_percent            // the antijerk percent of the curve to be sampled
                            );
                        } else {
                            end_curve_iterations = 0;
                        }
                        end_curve_distance = (util.get_curve_distance(
                                                  (delay_time / 1000.0) *               // the curve can only be sampled as often as the delay_time of the loop (to sec)
                                                      end_curve.acceleration,           // multiplied by the acceleration factor in in/sec
                                                  max_speed - end_curve.endpoint_speed, // max value output of the curve to be sampled
                                                  end_curve.antijerk_percent) +
                                                 (end_curve_iterations * end_curve.endpoint_speed)) // the block of endpoint speed upon which the
                                             * delay_time / 1000.0;                                 // the curve and endpoint speed output must be multiplied by delay (in sec) time to ensure a consistant output across all delay times
                        int body_iterations = std::ceil((distance - (start_curve_distance + end_curve_distance)) / (max_speed * delay_time / 1000));
                        total_iterations = start_curve_iterations + end_curve_iterations + body_iterations;
                        debt = distance - ((body_iterations * max_speed * delay_time / 1000) + start_curve_distance + end_curve_distance);
                    };
                    move_type_e type;
                    int id;
                    double max_speed;
                    double distance;
                    Curve start;
                    int start_curve_iterations = 0;
                    double start_curve_distance = 0;
                    Curve end;
                    int end_curve_iterations;
                    double end_curve_distance;
                    int total_iterations;
                    double debt;
            };

            struct Profiling_task_messenger_struct {
                    ChassisController* chassis_ptr; // a pointer to the chassis the task controls
                    pros::Imu* imu;                 // the imu to use for course correction
                    pros::Mutex mutex;              // the mutex to hold while modifying data
                    int delta_time;
                    double max_speed;   // motor.get_position()/inch
                    double p;           // the Proportional gain of the pid controller
                    double i;           // Integral gain
                    double d;           // Derivative gain
                    double total_error; // the total error experienced
                    bool reset = false;
                    bool imu_calibrating = false;
                    bool imu_calibrated = false;
                    MovementInfo current;
                    MovementInfo next;
                    double amount_completed = 0;
                    double debt = 0;
                    void resetMove() {
                        amount_completed = 0;
                        debt = 0;
                    }
                    /**
                     * Struct used to pass messages to pid_task
                     * \param chassis_ptr
                     *      a pointer to a chassis for the Profiling task to control
                     * \param drive_task_delay_factor
                     *      how many milliseconds the drive waits
                     */
                    Profiling_task_messenger_struct(ChassisController* chassis_ptr, pros::Imu* imu, Pid pid, double max_speed, int drive_task_delay_factor = 5)
                        : chassis_ptr{chassis_ptr},
                          delta_time{drive_task_delay_factor},
                          max_speed{max_speed},
                          imu{imu},
                          p{pid.p},
                          i{pid.i},
                          d{pid.d},
                          current(),
                          next() {
                    }
            };
            Profiling_task_messenger_struct profiling_task_messenger; // an instance of the pid task messenger struct used to commuicate with the pid task
            ChassisController chassis;                                // the chassis the controller controls
            pros::Task profile_loop_task;                             // the task that controlls the chassis
            pros::Imu imu;
            static void pid_loop(void* p); // the function the task uses to control the chassis
            int delay_time;
            static int* delay_time_ptr;

        public:
            explicit Drive(std::vector<int> left_motors, std::vector<int> right_motors, double wheel_diameter, int motor_rpm, double gear_ratio, Pid drive_pid, int imu_port, Pid turn_pid, int delay_time = 10)
                : chassis(left_motors, right_motors, wheel_diameter, motor_rpm, gear_ratio, drive_pid, delay_time),
                  imu(imu_port),
                  profiling_task_messenger(&chassis, &imu, turn_pid, ((motor_rpm * gear_ratio) * (wheel_diameter * M_PI)),
                      delay_time),
                  profile_loop_task(pid_loop, &profiling_task_messenger, "profiling_task"),
                  delay_time(delay_time){};
            // gets if the imu is calibrating
            // a function to change the correctional constants on the controller for the drive
            void set_drive_pid_constants(double p, double i, double d);
            // a function to change the correctional constants on the controller for the drive
            void set_turn_pid_constants(double p, double i, double d);
            // a function to get the total error that the imu has experienced for turning purposes
            double get_turn_total_error();
            // a function to get the total error that the motor encoders have experienced for tuning purposes
            double get_drive_total_error();
            // expose functions from meia::Chassis
            void tank_control(pros::Controller con = pros::Controller(pros::E_CONTROLLER_MASTER), pros::motor_brake_mode_e_t brake_mode = pros::E_MOTOR_BRAKE_COAST, double curve_intensity = 0, int deadzone = 0);
            std::pair<std::vector<double>, std::vector<double>> get_motor_temps();
            /**
             * Stops autonomus control
             * Clears all targets
             * Clears total error
             */
            void tare();
            /**
             * Stops autonomus control
             * Clears all targets
             * Clears total error
             */
            void end();
            /**
             * Stops autonomus control
             * Clears all targets
             * Clears total error
             * resets the imu
             */
            void init_imu();
            MovementTelemetry move_telem;
            MovementTelemetry move(move_type_e type, double distance, double max_speed = 0, Curve start_curve = Curve(), Curve end_curve = Curve());
    };
} // namespace meia