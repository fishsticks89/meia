#include "chassis_util.cpp"
#include "main.h"
namespace meia {
    //! PID system
    struct pid_internal {
            pid_internal(double p, double i, double d)
                : kp(p), ki(i), kd(d){};
            const double kp;
            const double ki;
            const double kd;
            int accumulated_integral_error = 0;
            int prev_error = 0;
            void reset() {
                accumulated_integral_error = 0;
                prev_error = 0;
            }
            double calc(double diff) {
                // new pid is calculated
                const double p = diff;
                const double i = accumulated_integral_error + diff;
                const double d = prev_error - diff;
                // telem for next calc
                prev_error = diff;
                accumulated_integral_error += diff;
                // correction is calculated
                return (p * kp) + (i * ki) + (d * kd);
            }
    };

    std::pair<double, double> add_pair(std::pair<double, double> a, std::pair<double, double> b) {
        return {a.first + b.first, a.second + b.second};
    }
    std::pair<double, double> make_pair(double p) {
        return {p, p};
    }
    //! Util Funcs
    /**
     * a function to get how far a turn needs to be
     * \param direction
     *      true if left
     */
    double Drive::get_turn(bool direction, double current, double target) {
        if (target < -180 || target > 180)
            throw "target must be -180 to 180";
        if ((target < current) == direction)
            return target - current;
        else if (direction)
            return (target - current) - 360;
        else if (!direction)
            return 360 + (target - current);
        return 0;
    }
    Drive::FramalIncrement Drive::Advance::get_increment(int iter) {
        if (iter * (delay_time / 1000) * cruise_vel < dist)
            return FramalIncrement(0, (cruise_vel / 1000) * delay_time);
        else
            return FramalIncrement(0, 0);
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
        std::cout << "yep";
        profile_loop_task.suspend();
        std::cout << "yep";
        profiling_task_messenger.current = Movement();
        std::cout << "yep";
        profiling_task_messenger.next = Movement();
        std::cout << "yep";
        profiling_task_messenger.total_error = 0;
        std::cout << "yep";
        profiling_task_messenger.reset = true;
        profiling_task_messenger.chassis_ptr->end();
        std::cout << "yep" << std::endl;
        profiling_task_messenger.imu->tare();
        profiling_task_messenger.mutex.give();
    }

    void Drive::tare() {
        std::cout << "yep";
        end();
    }

    void Drive::init_imu() {
        std::cout << "beginning" << std::endl;
        end();
        std::cout << "ended tasks!" << std::endl;
        profiling_task_messenger.mutex.take(10000);
        profiling_task_messenger.imu->reset();
        std::cout << "imu reset started!" << std::endl;
        profiling_task_messenger.imu_calibrating = true;
        while (profiling_task_messenger.imu->is_calibrating()) {
            pros::delay(10);
        }
        std::cout << "imu reset finished!" << std::endl;
        pros::lcd::set_text(3, "yes");
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

    /* MovementTelemetry */ void Drive::move(move_type_e type, double distance, double max_speed, double start_acc, double end_acc) {
        profile_loop_task.resume();
        bool break_allowed = false;
        while (!break_allowed) {
            profiling_task_messenger.mutex.take(3000);
            break_allowed = profiling_task_messenger.next.type() == none;
            profiling_task_messenger.mutex.give();
            // std::cout << "move_req - waiting for allowed" << std::endl;
            if (!break_allowed)
                pros::delay(10);
        }
        profiling_task_messenger.mutex.take(3000);
        int id = pros::millis();
        std::cout << "move_req - injecting >> " << id << std::endl;
        if (type == advance) {
            profiling_task_messenger.next = Drive::Advance(distance, start_acc, max_speed, end_acc, delay_time, id);
            std::cout << "Next ID: " << profiling_task_messenger.next.get_id() << std::endl;
            std::cout << "Next Type: " << profiling_task_messenger.next.type() << std::endl;
            std::cout << "Current ID: " << profiling_task_messenger.current.get_id() << std::endl;
            std::cout << "Current Type: " << profiling_task_messenger.current.type() << std::endl;
        }
        profiling_task_messenger.mutex.give();
        // waits for movement to begin
        break_allowed = false;
        while (!break_allowed) {
            profiling_task_messenger.mutex.take(3000);
            // std::cout << "move_req - awiatign begin >> " << id << std::endl;
            break_allowed = profiling_task_messenger.current.get_id() == id;
            // std::cout << "move_req - current move: >> " << profiling_task_messenger.current.get_id() << std::endl;
            profiling_task_messenger.mutex.give();
            if (!break_allowed)
                pros::delay(10);
        }
        std::cout << "move_req - dun" << std::endl;
        std::cout << profiling_task_messenger.current.get_id() << std::endl;

        return;
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
                profiling_info_struct(double p, double i, double d)
                    : turn(p, i, d){};
                struct turn_struct {
                        turn_struct(double p, double i, double d)
                            : turn_pid(p, i, d){};
                        double imu_reading;
                        double error;
                        double correction;
                        pid_internal turn_pid;
                        double target = 0; // rotational target for turn pid in degrees
                } turn;
                struct profiling_struct {
                        int i = 0;
                } profiling;
                std::pair<double, double> change_target = {0, 0};
        } info(io->p, io->i, io->d);

        while (true) {
            io->mutex.take(3000);
            // reset logic
            if (io->reset) {
                // resets pid
                info.turn.turn_pid.reset();
                // resets profiling
                info.profiling = profiling_info_struct::profiling_struct();
                io->reset = false;
            }
            // new move logic
            // std::cout << "current: " << io->current.get_id() << std::endl;
            // std::cout << "next: " << io->next.get_id() << std::endl;

            if ((io->current.get_increment(info.profiling.i).delta_advance == 0 && io->current.get_increment(info.profiling.i).delta_turn == 0) || io->current.type() == none || io->current.get_id() == 0) {
                // std::cout<<"operating!\n";
                io->current = io->next;
                io->next = Movement();
                info.profiling.i = 0;
            }

            info.change_target = {0, 0};                      // resets the amount the robot must move
            info.turn.imu_reading = io->imu->get_euler().yaw; // reads in global z axis from imu

            //! Motion Profiling Stuff
            const FramalIncrement increment = io->current.get_increment(info.profiling.i);
            info.change_target = {increment.delta_advance, increment.delta_advance};
            info.profiling.i++;

            if (io->current.type() != none) {
                //! Turn Pid Stuff
                if (!io->imu_calibrated)
                    throw "imu not calibrated";
                info.turn.error = (std::abs(get_dist(true, info.turn.imu_reading, info.turn.target)) < std::abs(get_dist(false, info.turn.imu_reading, info.turn.target)) ? get_dist(true, info.turn.imu_reading, info.turn.target) : get_dist(false, info.turn.imu_reading, info.turn.target));
                info.turn.correction = info.turn.turn_pid.calc(info.turn.error) * delta_time;
                info.change_target = add_pair(info.change_target, {info.turn.correction / -10000, info.turn.correction / 10000});
            }

            //! Output
            // std::cout << "prof - delta target: " << info.change_target.first << std::endl;
            io->chassis_ptr->change_target(info.change_target.first, info.change_target.second);
            // io->total_error += std::abs(info.turn.pid_hold.second.second) / (io->delta_time * 100000);

            // pros::lcd::set_text(5, "prof - imu_read: " + dub_to_string(info.turn.imu_reading) + " target: " + dub_to_string(info.turn.target));
            // pros::lcd::set_text(6, "prof - imu_pid-correction: " + dub_to_string(info.turn.correction) + " {" + dub_to_string(info.change_target.first * 10000) + ", " + dub_to_string(info.change_target.first * 100) + "}");
            io->mutex.give();
            pros::delay(delta_time);
        }
    }
} // namespace meia