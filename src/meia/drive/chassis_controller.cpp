#include "chassis_util.cpp"
#include "main.h"
namespace meia {
    //! utility
    int ChassisController::get_delay_time() {
        return pid_task_messenger.delta_time;
    }
    std::pair<double, double> ChassisController::get_error() {
        auto motor_positions = pid_task_messenger.chassis_ptr->get_motor_positions();
        return { (pid_task_messenger.left_target - (motor_positions.first / pid_task_messenger.ticks_per_inch)), (pid_task_messenger.right_target - (motor_positions.second / pid_task_messenger.ticks_per_inch))};
    }
    //! inherited from Chassis
    void ChassisController::tank_control(pros::Controller* con, pros::motor_brake_mode_e_t brake_mode, double curve_intensity, int deadzone) {
        pid_loop_task.suspend();
        pid_task_messenger.mutex.take(10000);
        pid_task_messenger.chassis_ptr->set_drive_brake(brake_mode);
        pid_task_messenger.chassis_ptr->tank_control(con, curve_intensity, deadzone);
        pid_task_messenger.mutex.give();
    }
    std::pair<std::vector<double>, std::vector<double>> ChassisController::get_motor_temps() {
        pid_task_messenger.mutex.take(10000);
        std::pair<std::vector<double>, std::vector<double>> temps = chassis.get_motor_temps();
        pid_task_messenger.mutex.give();
        return temps;
    }
    void ChassisController::tare() {
        pid_task_messenger.mutex.take(10000);
        pid_loop_task.suspend();
        pid_task_messenger.total_error = 0;
        // when the target is changed, if this reset value is true, the motor targets will be set to their current positions
        pid_task_messenger.reset = true;
        pid_task_messenger.chassis_ptr->set_voltage({0, 0});
        pid_task_messenger.chassis_ptr->tare_motors();
        pid_task_messenger.mutex.give();
    }

    //! Set PID
    void ChassisController::change_target(std::pair<double, double> deltatarget) {
        pid_task_messenger.mutex.take(3000);
        // reset if nessessary
        if (pid_task_messenger.reset == true) {
            pid_task_messenger.left_target = chassis.get_motor_positions().first;
            pid_task_messenger.right_target = chassis.get_motor_positions().second;
        }
        pid_task_messenger.chassis_ptr->set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
        pid_loop_task.resume();
        pid_task_messenger.left_target += deltatarget.first;
        pid_task_messenger.right_target += deltatarget.second;
        pid_task_messenger.mutex.give();
    }
    void ChassisController::set_pid_constants(double p, double i, double d) {
        pid_task_messenger.mutex.take(1000);
        pid_task_messenger.p = p;
        pid_task_messenger.i = i;
        pid_task_messenger.d = d;
        pid_task_messenger.mutex.give();
    }
    //! PID telemetry
    double ChassisController::get_total_error() {
        pid_task_messenger.mutex.take(10000);
        double total_error = pid_task_messenger.total_error;
        pid_task_messenger.mutex.give();
        return total_error;
    }
    double ChassisController::get_p_constant() {
        return pid_task_messenger.p;
    }

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

    //! The Task
    void ChassisController::pid_loop(void* p) {
        const auto io = static_cast<Pid_task_messenger_struct*>(p);
        io->mutex.take(10000);
        int delta_time = io->delta_time;
        if (!(30 >= io->delta_time <= 5)) // throws an error if people ask for over 30 millis of delay time
            throw "delay_time is measured in milliseconds and can only be 5 - 30";

        struct pid_info_struct {
                pid_info_struct(double p, double i, double d)
                    : left_pid(p, i, d), right_pid(p, i, d){};
                std::pair<double, double> motor_positions = {0, 0};
                std::pair<double, double> pid_correct = {0, 0};
                pid_internal left_pid;
                pid_internal right_pid;
                void reset() {
                    left_pid.reset();
                    right_pid.reset();
                    std::pair<double, double> pid_correct = {0, 0};
                }
        } pid_info(io->p, io->i, io->d);

        io->mutex.give(); // returns the mutex
        while (true) {
            io->mutex.take(3000);
            // log error for total error sampling for tuning
            io->total_error += std::abs(pid_info.left_pid.prev_error + pid_info.right_pid.prev_error) / (io->delta_time * 100000);
            pid_info.motor_positions = io->chassis_ptr->get_motor_positions();
            if (io->reset) {
                pid_info.reset();
                io->reset = false;
            }

            pid_info.pid_correct.first = pid_info.left_pid.calc(io->left_target - (pid_info.motor_positions.first / io->ticks_per_inch));
            pid_info.pid_correct.second = pid_info.right_pid.calc(io->right_target - (pid_info.motor_positions.second / io->ticks_per_inch));

            // {
            //     const int time = pros::millis();
            //     if ((time - (time % delta_time)) % 1000 == 0) {
            //         std::cout << "pid - targeet: " << io->left_target << std::endl;
            //         std::cout << "pid - tpi: " << io->ticks_per_inch << std::endl;
            //         std::cout << "pid - current_pos: " << dub_to_string(pid_info.motor_positions.first / io->ticks_per_inch) << std::endl;
            //         std::cout << "pid - voltiage: " << normalize(pid_info.pid_correct.first, pid_info.pid_correct.second, 127).first << ", " << normalize(pid_info.pid_correct.first, pid_info.pid_correct.second, 127).second << std::endl;
            //     }
            // }

            io->chassis_ptr->set_voltage(normalize(pid_info.pid_correct.first, pid_info.pid_correct.second, 127));
            io->mutex.give();

            pros::delay(delta_time);
        }
    }
} // namespace meia