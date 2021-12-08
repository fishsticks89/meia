#include "main.h"
namespace meia {
    //! Util Funcs
    /**
     * a function to get how far a turn needs to be
     * \param direction
     *      true if left
     */
    static double get_turn(bool direction, double current, double target) {
        if (target < -180 || target > 180)
            throw "target must be -180 to 180";
        if ((target < current) == direction)
            return target - current;
        else if (direction)
            return (target - current) - 360;
        else if (!direction)
            return 360 + (target - current);
    }
    //! inherited from ChassisController
    void Drive::tank_control(pros::Controller con, pros::motor_brake_mode_e_t brake_mode, double curve_intensity, int deadzone) {
        profiling_task_messenger.mutex.take(10000);
        profile_loop_task.suspend();
        profiling_task_messenger.chassis_ptr->tank_control(con, brake_mode, curve_intensity, deadzone);
        profiling_task_messenger.mutex.give();
    }
    std::pair<std::vector<double>, std::vector<double>> Drive::get_motor_temps() {
        profiling_task_messenger.mutex.take(10000);
        std::pair<std::vector<double>, std::vector<double>> temps = chassis.get_motor_temps();
        profiling_task_messenger.mutex.give();
        return temps;
    }
    void Drive::tare() {
        profiling_task_messenger.mutex.take(10000);
        profile_loop_task.suspend();
        profiling_task_messenger.total_error = 0;
        profiling_task_messenger.reset = true;
        profiling_task_messenger.imu->tare();
        profiling_task_messenger.chassis_ptr->end();
        profiling_task_messenger.mutex.give();
        printf("imu reset\n");
    }

    void Drive::end() {
        tare();
    }
    void Drive::set_drive_pid_constants(double p, double i, double d) {
        profiling_task_messenger.mutex.take(1000);
        profiling_task_messenger.chassis_ptr->set_pid_constants(p, i, d);
        profiling_task_messenger.mutex.give();
    }
    double Drive::get_drive_total_error() {
        profiling_task_messenger.mutex.take(10000);
        double total_error = profiling_task_messenger.chassis_ptr->get_total_error();
        profiling_task_messenger.mutex.give();
        return total_error;
    }
    //! Set PID
    void Drive::set_turn_pid_constants(double p, double i, double d) {
        profiling_task_messenger.mutex.take(1000);
        profiling_task_messenger.p = p;
        profiling_task_messenger.i = i;
        profiling_task_messenger.d = d;
        profiling_task_messenger.mutex.give();
    }
    //! PID telemetry
    double Drive::get_turn_total_error() {
        profiling_task_messenger.mutex.take(10000);
        double total_error = profiling_task_messenger.total_error;
        profiling_task_messenger.mutex.give();
        return total_error;
    }
    void Drive::turn(double target_deg) {
        profiling_task_messenger.mutex.take(10000);
        profiling_task_messenger.target = target_deg;
        profiling_task_messenger.mutex.give();
        profile_loop_task.resume();
    }
    //! The Task
    void Drive::pid_loop(void* p) {
        pros::delay(200);

        (*(Profiling_task_messenger_struct*)p).mutex.take(2000); // holds the pid task messenger struct mutex so pid task messenger struct can be red in
        auto io = static_cast<Profiling_task_messenger_struct*>(p);
        Profiling_task_messenger_struct io_hold = *io;
        (*(Profiling_task_messenger_struct*)p).mutex.give(); // returns the mutex

        if (!(30 >= io_hold.delta_time <= 5)) // throws an error if people ask for over 30 millis of delay time
            throw "delay_time is measured in milliseconds and can only be 5 - 30";

        struct profiling_info_struct {
                profiling_info_struct(){};
                double imu_reading;
                std::pair<double, std::pair<double, double>> pid_held = {0, {0, 0}};
                std::pair<double, double> change_target = {0, 0};
        } info;

        while (true) {
            printf(util.dub_to_string(pros::millis()).c_str());
            printf("\n");
            io->mutex.take(3000);
            io->total_error += std::abs(info.pid_held.second.second) / (io_hold.delta_time * 100000);
            io->mutex.give();

            info.imu_reading = io_hold.imu->get_euler().yaw;

            if (io_hold.reset) {
                io->mutex.take(3000);
                info.pid_held = {0, {0, 0}};
                io->reset = false;
                io->mutex.give();
            }

            //! Motion Profiling Stuff

            info.change_target = {0, 0};

            // Todo: curve stuff

            // Turn Pid Stuff

            info.pid_held = {(info.imu_reading - io_hold.target) * io_hold.p, {0, 0}};
            info.change_target = {info.pid_held.first / -10000, info.pid_held.first / 10000};

            // Output

            io->mutex.take(3000);
            io->chassis_ptr->change_target(info.change_target.first, info.change_target.second);
            io->mutex.give();

            pros::lcd::set_text(5, "read: " + util.dub_to_string(info.imu_reading) + " target: " + util.dub_to_string(io_hold.target));
            pros::lcd::set_text(6, " pid-correction: " + util.dub_to_string(info.pid_held.first) + " {" + util.dub_to_string(info.change_target.first * 10000) + ", " + util.dub_to_string(info.change_target.first * 100) + "}");
            printf(util.dub_to_string(pros::millis()).c_str());
            printf("\n\n");
            pros::delay(io_hold.delta_time);
        }
    }
} // namespace meia