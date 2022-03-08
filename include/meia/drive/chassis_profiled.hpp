#include "main.h"
namespace meia {
    enum move_type_e
    {
        none,
        advance,
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
            class FramalIncrement {
                public:
                    const double delta_turn;
                    const double delta_advance;
                    FramalIncrement(double delta_turn, double delta_advance)
                        : delta_turn(delta_turn), delta_advance(delta_advance){};
            };
            class Movement {
                public:
                    virtual int get_id() {
                        return 0;
                    };
                    virtual move_type_e type() {
                        return move_type_e::none;
                    };
                    /**
                     * A function to get the increment for the rotational and directional target for the profiled chassis
                     *      \param iter
                     *          the iterations into the movement (must start at one)
                     */
                    virtual FramalIncrement get_increment(int iter) {
                        return FramalIncrement(0, 0);
                    };
            };
            class Advance : public Movement {
                    int id = 0;
                    double delay_time = 0;
                    double dist = 0;
                    double start_acc = 0;
                    double cruise_vel = 0;
                    double end_acceleration = 0;

                public:
                    Advance(double dist, double start_acceleration, double peak_velocity, double end_acceleration, int delay_time, int id)
                        : dist(dist), start_acc(start_acceleration), cruise_vel(cruise_vel), end_acceleration(end_acceleration), delay_time(delay_time), id(id){};
                    virtual move_type_e type() override {
                        return move_type_e::advance;
                    }
                    int get_id() override {
                        return id;
                    }
                    virtual FramalIncrement get_increment(int iter) override;
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
                    std::shared_ptr<Movement> current = std::make_shared<Movement>();
                    std::shared_ptr<Movement> next = std::make_shared<Movement>();
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
                          d{pid.d} {
                    }
            };
            Profiling_task_messenger_struct profiling_task_messenger; // an instance of the pid task messenger struct used to commuicate with the pid task
            ChassisController chassis;                                // the chassis the controller controls
            pros::Task profile_loop_task;                             // the task that controlls the chassis
            pros::Imu imu;
            static void pid_loop(void* p); // the function the task uses to control the chassis
            /**
             * a function to get how far a turn needs to be
             * \param direction
             *      true if left
             */
            static double get_turn(bool direction, double current, double target);
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

            void move(move_type_e type, double distance, double max_speed, double start_acc, double end_acc);
    };
} // namespace meia