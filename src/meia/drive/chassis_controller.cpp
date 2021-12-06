#include "main.h"
namespace meia {
    //! inherited from Chassis
    void ChassisController::tank_control(pros::Controller con, pros::motor_brake_mode_e_t brake_mode, double curve_intensity, int deadzone) {
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

    void ChassisController::end() {
        tare();
    }
    //! Set PID
    void ChassisController::change_target(double l, double r) {
        pid_task_messenger.mutex.take(3000);
        pid_task_messenger.chassis_ptr->set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
        if (pid_task_messenger.reset == true) {
            pid_task_messenger.left_target = chassis.get_motor_positions().first;
            pid_task_messenger.right_target = chassis.get_motor_positions().second;
        }
        pid_loop_task.resume();
        pid_task_messenger.left_target += l;
        pid_task_messenger.right_target += r;
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

    void ChassisController::pid_loop(void* p) {
        pros::delay(200);
        (*(Pid_task_messenger_struct*)p).mutex.take(2000);                    // holds the pid task messenger struct mutex so pid task messenger struct can be red in
        Pid_task_messenger_struct messaging = *(Pid_task_messenger_struct*)p; // reads in pid task messenger struct
        (*(Pid_task_messenger_struct*)p).mutex.give();                        // returns the mutex
        if (!(30 >= messaging.delta_time <= 5))                               // throws an error if people ask for over 30 millis of delay time
            throw "delay_time is measured in milliseconds and can only be 5 - 30";

        struct pid_info_struct {
                pid_info_struct(){};
                std::pair<double, double> motor_positions = {0, 0};
                std::pair<double, std::pair<double, double>> left_pid = {0, {0, 0}};
                std::pair<double, std::pair<double, double>> right_pid = {0, {0, 0}};
        } pid_info;
        while (true) {
            (*(Pid_task_messenger_struct*)p).mutex.take(3000);
            // log error for total error sampling for tuning
            (*(Pid_task_messenger_struct*)p).total_error += std::abs(pid_info.left_pid.second.second + pid_info.right_pid.second.second) / (messaging.delta_time * 100000);
            messaging = *(Pid_task_messenger_struct*)p;
            pid_info.motor_positions = (*messaging.chassis_ptr).get_motor_positions();
            (*(Pid_task_messenger_struct*)p).mutex.give();
            if (messaging.reset) {
                messaging.mutex.take(2000);
                pid_info.left_pid = {0, {0, 0}};
                pid_info.right_pid = {0, {0, 0}};
                (*(Pid_task_messenger_struct*)p).reset = false;
                messaging.mutex.give();
            }
            pid_info.left_pid = util.pid(pid_info.motor_positions.first, messaging.left_target * messaging.ticks_per_inch, messaging.p, messaging.i, messaging.d, pid_info.left_pid.second, messaging.delta_time);
            pid_info.right_pid = util.pid(pid_info.motor_positions.second, messaging.right_target * messaging.ticks_per_inch, messaging.p, messaging.i, messaging.d, pid_info.right_pid.second, messaging.delta_time);

            (*(Pid_task_messenger_struct*)p).mutex.take(3000);
            (*messaging.chassis_ptr).set_voltage(util.normalize(pid_info.left_pid.first, pid_info.right_pid.first, 127));
            (*(Pid_task_messenger_struct*)p).mutex.give();

            pros::delay(messaging.delta_time);
        }
    }
} // namespace meia