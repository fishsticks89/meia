#include "main.h"
namespace meia {
    template <typename T, typename U>
    std::pair<T, U> operator+(const std::pair<T, U>& l, const std::pair<T, U>& r) {
        return {l.first + r.first, l.second + r.second};
    }
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
    void Drive::end() {
        profiling_task_messenger.mutex.take(10000);
        profile_loop_task.suspend();
        profiling_task_messenger.total_error = 0;
        profiling_task_messenger.reset = true;
        profiling_task_messenger.chassis_ptr->end();
        profiling_task_messenger.mutex.give();
    }

    void Drive::tare() {
        end();
        profiling_task_messenger.mutex.take(10000);
        profiling_task_messenger.imu->tare();
        profiling_task_messenger.mutex.give();
        printf("imu reset\n");
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

    MovementTelemetry Drive::move(move_type_e type, double distance, double max_speed, Curve start, Curve end) {
        bool break_allowed = false;
        while (!break_allowed) {
            profiling_task_messenger.mutex.take(3000);
            break_allowed = profiling_task_messenger.next.type == none;
            profiling_task_messenger.mutex.give();
            if (!break_allowed)
                pros::delay(10);
        }
        profiling_task_messenger.mutex.take(3000);
        int id = pros::millis();
        profiling_task_messenger.next = MovementInfo(start, end, max_speed, distance, type, id, delay_time);
        profiling_task_messenger.mutex.give();
        // waits for movement to begin
        break_allowed = false;
        while (!break_allowed) {
            profiling_task_messenger.mutex.take(3000);
            break_allowed = profiling_task_messenger.current.id == id;
            profiling_task_messenger.mutex.give();
            if (!break_allowed)
                pros::delay(10);
        }
        return MovementTelemetry(distance, /*something in messenger*/ nullptr, &profiling_task_messenger.current.id, &profiling_task_messenger.mutex, id);
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
                double error;
                std::pair<double, std::pair<double, double>> pid_hold = {0, {0, 0}};
                std::pair<double, double> change_target = {0, 0};
                double target = 0;  // rotational target for turn pid in degrees
                MovementInfo move;
        } info;

        while (true) {
            printf(util.dub_to_string(pros::millis()).c_str());
            printf("\n");
            io->mutex.take(3000);
            // reset logic
            if (io->reset) {
                // resets pid
                info.pid_hold = {0, {0, 0}};
                // resets profiling
                io->resetMove();
                io->current = MovementInfo();
                io->next = MovementInfo();
                io->reset = false;
            }
            // new move logic
            if (std::abs(io->amount_completed) > std::abs(io->current.distance))
                io->current = io->next;
            io->next = MovementInfo();
            io_hold = *io;
            io->mutex.give();

            info.change_target = {0, 0}; // resets the amount the robot must move
            //* No mutex is held during IMU operation; all IMU references are unsafe unless task is disabled
            info.imu_reading = io_hold.imu->get_euler().yaw; // reads in global z axis from imu

            //! Motion Profiling Stuff

            if (io_hold.current.type == drive)
                info.change_target = info.change_target + std::make_pair(io_hold.current.distance, io_hold.current.distance);
            if (io_hold.current.type == turn)
                info.target += io_hold.current.distance;

            // Todo: curve stuff

            //! Turn Pid Stuff
            info.error = (std::abs(util.get_dist(true, info.imu_reading, info.target)) < std::abs(util.get_dist(false, info.imu_reading, info.target)) ? util.get_dist(true, info.imu_reading, info.target) : util.get_dist(false, info.imu_reading, info.target));

            info.pid_hold = util.pid(0, info.error, io_hold.p, io_hold.i, io_hold.d, info.pid_hold.second, io_hold.delta_time);
            info.change_target = {info.pid_hold.first / -10000, info.pid_hold.first / 10000};

            //! Output

            io->mutex.take(3000);
            io->chassis_ptr->change_target(info.change_target.first, info.change_target.second);
            io->total_error += std::abs(info.pid_hold.second.second) / (io_hold.delta_time * 100000);
            io->mutex.give();

            pros::lcd::set_text(5, "read: " + util.dub_to_string(info.imu_reading) + " target: " + util.dub_to_string(info.target));
            pros::lcd::set_text(6, " pid-correction: " + util.dub_to_string(info.pid_hold.first) + " {" + util.dub_to_string(info.change_target.first * 10000) + ", " + util.dub_to_string(info.change_target.first * 100) + "}");
            printf(util.dub_to_string(pros::millis()).c_str());
            printf("\n\n");
            pros::delay(io_hold.delta_time);
        }
    }
} // namespace meia