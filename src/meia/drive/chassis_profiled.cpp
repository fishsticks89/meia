#include "main.h"
namespace meia {

    //! inherited from Chassis
    void Drive::tank_control(pros::Controller con, pros::motor_brake_mode_e_t brake_mode = pros::E_MOTOR_BRAKE_COAST, double curve_intensity, int deadzone) {
        profile_loop_task.suspend();
        profiling_task_messenger.mutex.take(10000);
        chassis.tank_control(con, brake_mode, curve_intensity, deadzone);
        profiling_task_messenger.mutex.give();
    }
    std::pair<std::vector<double>, std::vector<double>> Drive::get_motor_temps() {
        profiling_task_messenger.mutex.take(10000);        
        std::pair<std::vector<double>, std::vector<double>> temps = chassis.get_motor_temps();
        profiling_task_messenger.mutex.give();
        return temps;
    }
    void ChassisController::set_drive_brake(pros::motor_brake_mode_e_t input) {
        pid_task_messenger.mutex.take(10000);
        chassis.set_drive_brake(input);
        pid_task_messenger.mutex.give();
    }
    void Drive::tare() {
        profiling_task_messenger.mutex.take(10000);
        profile_loop_task.suspend();
        profiling_task_messenger.total_error = 0;
        profiling_task_messenger.reset = true;
        chassis.end();
        profiling_task_messenger.mutex.give();
    }

    void Drive::end() {
        tare();
    }
    void Drive::set_drive_pid_constants(double p, double i, double d) {
        profiling_task_messenger.mutex.take(1000);
        chassis.set_pid_constants(p, i, d);
        profiling_task_messenger.mutex.give();
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
    double Drive::get_total_error() {
        profiling_task_messenger.mutex.take(10000);
        double total_error = profiling_task_messenger.total_error;
        profiling_task_messenger.mutex.give();
        return total_error;
    }
    //! The Task
    std::pair<double, double> Drive::normalize(double l_volt, double r_volt, double max) {
        double l_dif = (std::abs(l_volt) > max) ? std::abs(l_volt) - max : 0;
        double r_dif = (std::abs(r_volt) > max) ? std::abs(l_volt) - max : 0;
        // scale down the voltages similarly if one is over 127
        bool greater_dif = l_dif > r_dif;
        double dif = (greater_dif) ? l_dif : r_dif;
        double l_out = (max / (max + dif)) * l_volt;
        double r_out = (max / (max + dif)) * r_volt;
        return { l_out, r_out };
    }

    void Drive::pid_loop(void* p) {
        pros::delay(200);
        (*(Profiling_task_messenger_struct*)p).mutex.take(2000); // holds the pid task messenger struct mutex so pid task messenger struct can be red in
        Profiling_task_messenger_struct messaging = *(Profiling_task_messenger_struct*)p; // reads in pid task messenger struct
        (*(Profiling_task_messenger_struct*)p).mutex.give(); // returns the mutex
        if (!(30 >= messaging.delta_time <= 5)) // throws an error if people ask for over 30 millis of delay time
            throw "delay_time is measured in milliseconds and can only be 5 - 30";

        struct profiling_info_struct {
            profiling_info_struct() {};
            double imu_reading;
            std::pair<double, std::pair<double, double>> pid_held = { 0, {0, 0} };
            std::pair<double, double> change_target = {0, 0};
        } info;
        while (true) {
            messaging.mutex.take(3000);
            (*(Profiling_task_messenger_struct*)p).total_error += std::abs(info.pid_held.second.second) / (messaging.delta_time * 100000);
            messaging = *(Profiling_task_messenger_struct*)p;
            info.imu_reading = messaging.imu->get_rotation();
            messaging.mutex.give();
            if (messaging.reset) {
                messaging.mutex.take(2000);
                info.pid_held = { 0, {0, 0} };
                (*(Profiling_task_messenger_struct*)p).reset = false;
                messaging.mutex.give();
            }
            
            //! Turn Pid Stuff

            info.change_target = {0, 0};

            info.pid_held = pid(info.imu_reading, messaging.target * messaging.max_speed, messaging.p, 0, 0, info.pid_held.second, messaging.delta_time);

            info.change_target = {info.change_target.first + info.pid_held.first, info.change_target.second - info.pid_held.first};

            pros::delay(messaging.delta_time);

            // Todo: curve stuff
        }
    }
}