#include "main.h"
namespace meia {

    //! inherited from Chassis
    void ChassisController::tank_control(pros::Controller con, pros::motor_brake_mode_e_t brake_mode = pros::E_MOTOR_BRAKE_COAST, double curve_intensity, int deadzone) {
        pid_loop_task.suspend();
        pid_task_messenger.mutex.take(10000);
        chassis.set_drive_brake(brake_mode);
        chassis.tank_control(con, curve_intensity, deadzone);
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
        pid_task_messenger.reset = true;
        chassis.set_voltage({ 0, 0 });
        chassis.tare_motors();
        pid_task_messenger.mutex.give();
    }

    void ChassisController::end() {
        tare();
    }
    //! Set PID
    void ChassisController::change_target(double l, double r) {
        pid_task_messenger.mutex.take(3000);
        chassis.set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
        if (pid_task_messenger.reset == true) {
            pid_task_messenger.left_target = chassis.get_motor_positions().first;
            pid_task_messenger.right_target = chassis.get_motor_positions().second;
        }
        pid_task_messenger.left_target += l;
        pid_task_messenger.right_target += r;
        pid_loop_task.resume();
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
    //! The Task
    static std::pair<double, double> normalize(double l_volt, double r_volt, double max) {
        double l_dif = (std::abs(l_volt) > max) ? std::abs(l_volt) - max : 0;
        double r_dif = (std::abs(r_volt) > max) ? std::abs(l_volt) - max : 0;
        // scale down the voltages similarly if one is over 127
        bool greater_dif = l_dif > r_dif;
        double dif = (greater_dif) ? l_dif : r_dif;
        double l_out = (max / (max + dif)) * l_volt;
        double r_out = (max / (max + dif)) * r_volt;
        return { l_out, r_out };
    }
    // a function that returns the correction, along with info to be fed to it as prev. prev is a pair <error, integral>
    std::pair<double, std::pair<double, double>> ChassisController::pid(double current, double target, double p, double i, double d, std::pair<double, double> prev, int delta_time) {
        // p constant is translated into correction
        double p_correct = target - current;
        // new integral and prev error is calculated
        double new_i = prev.second + p_correct;
        double new_prev_error = p_correct;
        // i and d constants are translated into corrections
        double d_correct = prev.first - p_correct;
        double i_correct = new_i;
        // correction is calculated
        double error = (p_correct * p) + (i_correct * i) + (d_correct * d);
        return { error / delta_time, {new_prev_error, new_i} };
    }

    void ChassisController::pid_loop(void* p) {
        pros::delay(200);
        (*(Pid_task_messenger_struct*)p).mutex.take(2000); // holds the pid task messenger struct mutex so pid task messenger struct can be red in
        Pid_task_messenger_struct messaging = *(Pid_task_messenger_struct*)p; // reads in pid task messenger struct
        (*(Pid_task_messenger_struct*)p).mutex.give(); // returns the mutex
        if (!(30 >= messaging.delta_time <= 5)) // throws an error if people ask for over 30 millis of delay time
            throw "delay_time is measured in milliseconds and can only be 5 - 30";

        struct pid_info_struct {
            pid_info_struct();
            std::pair<double, double> motor_positions;
            std::pair<double, std::pair<double, double>> left_pid = { 0, {0, 0} };
            std::pair<double, std::pair<double, double>> right_pid = { 0, {0, 0} };
            Chassis chassis;
        } pid_info;
        while (true) {
            messaging.mutex.take(3000);
            (*(Pid_task_messenger_struct*)p).total_error += std::abs(pid_info.left_pid.second.second + pid_info.right_pid.second.second) / (messaging.delta_time * 100000);
            messaging = *(Pid_task_messenger_struct*)p;
            pid_info.motor_positions = (*messaging.chassis_ptr).get_motor_positions();
            messaging.mutex.give();
            if (messaging.reset) {
                messaging.mutex.take(2000);
                pid_info.left_pid = { 0, {0, 0} };
                pid_info.right_pid = { 0, {0, 0} };
                (*(Pid_task_messenger_struct*)p).reset = false;
                messaging.mutex.give();
            }
            pid_info.left_pid = pid(pid_info.motor_positions.first, messaging.left_target * messaging.ticks_per_inch, messaging.p, 0, 0, pid_info.left_pid.second, messaging.delta_time);
            pid_info.right_pid = pid(pid_info.motor_positions.second, messaging.right_target * messaging.ticks_per_inch, messaging.p, 0, 0, pid_info.right_pid.second, messaging.delta_time);

            pid_info.chassis.set_voltage(normalize(pid_info.left_pid.first, pid_info.right_pid.first, 127));

            pros::delay(messaging.delta_time);
        }
    }
}