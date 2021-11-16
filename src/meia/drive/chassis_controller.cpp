#include "main.h"
namespace meia {

    //! inherited from Chassis
    void ChassisController::tank_control(pros::Controller con, pros::motor_brake_mode_e_t brake_mode = pros::E_MOTOR_BRAKE_COAST, double curve_intensity, int deadzone) {
        pid_loop_task.suspend();
        chassis.set_drive_brake(brake_mode);
        chassis.tank_control(con, curve_intensity, deadzone);
    }
    std::pair<std::vector<double>, std::vector<double>> ChassisController::get_motor_temps() {
        return chassis.get_motor_temps();
    }
    void ChassisController::tare() {
        pid_loop_task.suspend();
        chassis.set_voltage({0, 0});
        chassis.tare_motors();
    }

    void ChassisController::suspend() {
        tare();
    }
    //! Set PID
    void ChassisController::change_target(double l, double r) {
        chassis.set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
        pid_task_messenger.mutex.take(3000);
        pid_task_messenger.left_target += l;
        pid_task_messenger.right_target += r;
        pid_task_messenger.mutex.give();
        pid_loop_task.resume();
    }
    //! The Task
    std::pair<double, double> ChassisController::normalize(double l_volt, double r_volt, double max) {
        double l_dif = (std::abs(l_volt) > max) ? std::abs(l_volt) - max : 0;
        double r_dif = (std::abs(r_volt) > max) ? std::abs(l_volt) - max : 0;
        // scale down the voltages similarly if one is over 127
        bool greater_dif = l_dif > r_dif;
        double dif = (greater_dif) ? l_dif : r_dif;
        double l_out = (max/(max + dif)) * l_volt;
        double r_out = (max/(max + dif)) * r_volt;
        return {l_out, r_out}; 
    }
    // a function that returns the correction, along with info to be fed to it as prev. prev is a pair <error, integral>
    std::pair<double, std::pair<double, double>> ChassisController::pid (double current, double target, double p, double i, double d, std::pair<double, double> prev) {
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
        return {error, {new_prev_error, new_i}};
    }

    struct pid_info_struct {
        pid_info_struct(Chassis chassis) : chassis{chassis} {};
        std::pair<double, double> motor_positions;
        std::pair<double, std::pair<double, double>> left_pid = {0, {0, 0}};
        std::pair<double, std::pair<double, double>> right_pid = {0, {0, 0}};
        Chassis chassis;
    };
    
    void ChassisController::pid_loop(void* p) {
        pros::delay(200);
        (*(Pid_task_messenger_struct*)p).mutex.take(2000); // holds the pid task messenger struct mutex so pid task messenger struct can be red in
        Pid_task_messenger_struct messaging = *(Pid_task_messenger_struct*)p; // reads in pid task messenger struct
        (*(Pid_task_messenger_struct*)p).mutex.give(); // returns the mutex
        if (!(30 >= messaging.delta_time <= 5)) // throws an error if people ask for over 30 millis of delay time
            throw "delay_time is measured in milliseconds and can only be 5 - 30";
        pid_info_struct pid_info (*messaging.chassis_ptr); // useful vars
        while (true) {
            messaging.mutex.take(3000);
            messaging = *(Pid_task_messenger_struct*)p;
            messaging.mutex.give();
            pid_info.motor_positions = pid_info.chassis.get_motor_positions();
            pid_info.left_pid = pid(pid_info.motor_positions.first, messaging.left_target * messaging.ticks_per_inch, messaging.p, 0, 0, pid_info.left_pid.second);
            pid_info.right_pid = pid(pid_info.motor_positions.second, messaging.right_target * messaging.ticks_per_inch, messaging.p, 0, 0, pid_info.right_pid.second);

            pid_info.chassis.set_voltage(normalize(pid_info.left_pid.first, pid_info.right_pid.first, 127));

            pros::delay(messaging.delta_time);
        }
    }
}