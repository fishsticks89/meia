#include "main.h"
namespace meia {
    std::pair<double, double> add_pair(std::pair<double, double> a, std::pair<double, double> b) {
        return {a.first + b.first, a.second + b.second};
    }
    std::pair<double, double> add_pair(std::pair<double, double> a, double b) {
        return {a.first + b, a.second + b};
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
        profiling_task_messenger.current = MovementInfo();
        profiling_task_messenger.next = MovementInfo();
        profiling_task_messenger.total_error = 0;
        profiling_task_messenger.reset = true;
        profiling_task_messenger.chassis_ptr->end();
        profiling_task_messenger.imu->tare();
        profiling_task_messenger.mutex.give();
    }

    void Drive::tare() {
        end();
    }

    void Drive::init_imu() {
        end();
        profiling_task_messenger.mutex.take(10000);
        profiling_task_messenger.imu->reset();
        profiling_task_messenger.imu_calibrating = true;
        while (profiling_task_messenger.imu->is_calibrating()) {
            pros::delay(10);
        }
        profiling_task_messenger.imu_calibrating = false;
        profiling_task_messenger.imu_calibrated = true;
        profiling_task_messenger.mutex.give();
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
        profile_loop_task.resume();
        bool break_allowed = false;
        while (!break_allowed) {
            profiling_task_messenger.mutex.take(3000);
            break_allowed = profiling_task_messenger.next.type == none;
            profiling_task_messenger.mutex.give();
            std::cout << "waiting for allowed" << std::endl;
            if (!break_allowed)
                pros::delay(10);
        }
        profiling_task_messenger.mutex.take(3000);
        int id = pros::millis();
        std::cout << "injecting >> " << id << std::endl;
        profiling_task_messenger.next = MovementInfo(type, start, end, max_speed, distance, id, delay_time);
        profiling_task_messenger.mutex.give();
        // waits for movement to begin
        break_allowed = false;
        while (!break_allowed) {
        std::cout << "awiatign begin >> " << id << std::endl;
            profiling_task_messenger.mutex.take(3000);
            break_allowed = profiling_task_messenger.current.id == id;
            std::cout << "current >> " << profiling_task_messenger.current.id << std::endl;
            profiling_task_messenger.mutex.give();
            if (!break_allowed)
                pros::delay(10);
        }
        std::cout << "dun" << std::endl;
        std::cout << profiling_task_messenger.current.id << std::endl;

        return MovementTelemetry(distance, &profiling_task_messenger.amount_completed, &profiling_task_messenger.current.id, &profiling_task_messenger.mutex, id);
    }

    //! The Task
    void Drive::pid_loop(void* p) {
        pros::delay(200);

        (*(Profiling_task_messenger_struct*)p).mutex.take(2000); // holds the pid task messenger struct mutex so pid task messenger struct can be red in
        auto io = static_cast<Profiling_task_messenger_struct*>(p);

        int delta_time = io->delta_time;

        if (!(30 >= io->delta_time <= 5)) // throws an error if people ask for over 30 millis of delay time
            throw "delay_time is measured in milliseconds and can only be 5 - 30";
        
        (*(Profiling_task_messenger_struct*)p).mutex.give(); // returns the mutex

        struct profiling_info_struct {
                profiling_info_struct(){};
                struct turn_struct {
                    turn_struct(){};
                    double imu_reading;
                    double correct;
                    std::pair<double, std::pair<double, double>> pid_hold = {0, {0, 0}};
                    double target = 0; // rotational target for turn pid in degrees
                } turn;
                struct profiling_struct {
                    MovementInfo move;
                    int i = 0;
                } profiling;
                std::pair<double, double> change_target = {0, 0};
        } info;
        
        while (true) {
            io->mutex.take(3000);
            // reset logic
            if (io->reset) {
                // resets pid
                info.turn.pid_hold = {0, {0, 0}};
                // resets profiling
                io->resetMove();
                io->reset = false;
            }
            // new move logic
            if (std::abs(io->amount_completed) >= std::abs(io->current.distance))
                io->current = io->next;
            std::cout << "current: " << io->current.distance << std::endl;
            std::cout << "completed: " << io->amount_completed << std::endl;

            info.change_target = {0, 0}; // resets the amount the robot must move
            info.turn.imu_reading = io->imu->get_euler().yaw; // reads in global z axis from imu

            //! Motion Profiling Stuff
            if (io->current.type == drive) {
                if (info.profiling.i <= io->current.start_curve_iterations)
                    info.change_target = add_pair(info.change_target, (util.curve(info.profiling.i * (io->current.start.acceleration / 1000), io->current.max_speed - io->current.start.endpoint_speed, io-> current.start.antijerk_percent) + io->current.start.endpoint_speed) * delta_time);
                else if (info.profiling.i < io->total_error - io->current.end_curve_iterations)
                    info.change_target = add_pair(info.change_target, io->current.max_speed * delta_time);
                else
                    info.change_target = add_pair(info.change_target, (util.curve((io->current.total_iterations - io->current.end_curve_iterations) * (io->current.end.acceleration / 1000), io->current.max_speed - io->current.end.endpoint_speed, io-> current.end.antijerk_percent) + io->current.end.endpoint_speed) * delta_time);
                io->amount_completed += info.change_target.first;
                info.profiling.i++;
            } else if (io->current.type == turn) {
                info.turn.target += io->current.distance;
            }

            //! Turn Pid Stuff
            if (io->current.type == drive || io->current.type == hold) {
                if (!io->imu_calibrated)
                    throw "imu not calibrated";
                info.turn.correct = (std::abs(util.get_dist(true, info.turn.imu_reading, info.turn.target)) < std::abs(util.get_dist(false, info.turn.imu_reading, info.turn.target)) ? util.get_dist(true, info.turn.imu_reading, info.turn.target) : util.get_dist(false, info.turn.imu_reading, info.turn.target));
                info.turn.pid_hold = util.pid(0, info.turn.correct, io->p, io->i, io->d, info.turn.pid_hold.second, io->delta_time);
                info.change_target = add_pair(info.change_target, {info.turn.pid_hold.first / -10000, info.turn.pid_hold.first / 10000});
            }

            // Todo: debting (so move amount_completed here)

            //! Output
            std::cout << "delta target: " << info.change_target.first << std::endl;
            io->chassis_ptr->change_target(info.change_target.first, info.change_target.second);
            io->total_error += std::abs(info.turn.pid_hold.second.second) / (io->delta_time * 100000);
            io->mutex.give();

            pros::lcd::set_text(5, "read: " + util.dub_to_string(info.turn.imu_reading) + " target: " + util.dub_to_string(info.turn.target));
            pros::lcd::set_text(6, " pid-correction: " + util.dub_to_string(info.turn.pid_hold.first) + " {" + util.dub_to_string(info.change_target.first * 10000) + ", " + util.dub_to_string(info.change_target.first * 100) + "}");
            pros::delay(delta_time);
        }
    }
} // namespace meia