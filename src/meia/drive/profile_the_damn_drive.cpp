#include "main.h"

double getBigger(std::pair<double, double> e) {
    return std::max(e.first, e.second);
}

namespace meia {
    namespace p {
        double profile(ChassisController* chassis, profiling_fn_t profiling_fn, int duration) {
            const int delay = chassis->get_delay_time();
            for (int time = 0; time < duration; time += delay) {
                chassis->change_target(profiling_fn(time, delay));
                pros::delay(delay);
            }
            return 0;
        }
        double profileuntil(ChassisController* chassis, profiling_fn_t profiling_fn) {
            const int delay = chassis->get_delay_time();
            std::pair<double, double> increment = {1, 1};
            for (int time = 0; increment != std::pair<double, double>{0, 0}; time += delay) {
                increment = profiling_fn(time, delay);
                if (increment == std::pair<double, double>{0, 0})
                    break;
                chassis->change_target(increment);
                pros::delay(delay);
            }
            return 0;
        }
        double go(ChassisController* chassis, double amount, double speed, double acc) {
            const int acc_time = (speed / acc) * 1000;                                         // ms
            const int cruise_time = ((amount - ((acc_time / 1000.0) * speed)) / speed) * 1000; // ms
            std::cout << acc_time << std::endl;
            std::cout << cruise_time << std::endl;
            if (cruise_time < 0) {
                throw " ur mom ";
            }
            // accelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = ((time / 1000.0) * acc);
                    const double increment = (tspeed / 1000) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment, increment};
                },
                acc_time);
            // cruise
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double increment = (speed / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment, increment};
                },
                cruise_time);
            // decelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = (((acc_time - time) / 1000.0) * acc);
                    const double increment = (tspeed / 1000) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment, increment};
                },
                acc_time);
            return 0;
        }
        double turn(ChassisController* chassis, double amount, double speed, double acc, double drive_width) {
            amount = (amount / 360.0) * drive_width * m_pi;
            const int acc_time = (speed / acc) * 1000;                              // ms
            const int cruise_time = ((amount - (acc_time * speed)) / speed) * 1000; // ms
            // accelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = ((time / 1000.0) * acc);
                    const double increment = (tspeed / 1000) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment, -increment};
                },
                acc_time);
            // cruise
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double increment = (speed / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment, -increment};
                },
                cruise_time);
            // decelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = (((acc_time - time) / 1000.0) * acc);
                    const double increment = (tspeed / 1000) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment, -increment};
                },
                acc_time);
            return 0;
        }

        double debting_go(ChassisController* chassis, double amount, double speed, double decel) {
            double debt = 0;
            const int acc_time = (speed / decel) * 1000;
            double cruise_dist = (amount - ((acc_time / 1000.0) * speed / 2));
            std::cout << "acctime" << acc_time << std::endl;
            std::cout << "cditst" << cruise_dist << std::endl;
            bool isneg = (cruise_dist < 0);
            profileuntil(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    double increment = (speed / 1000.0) * delta_time;
                    const double overature = (getBigger(chassis->get_error()) * chassis->get_p_constant() - 12000) / chassis->get_p_constant();
                    if (overature > 0) {
                        increment -= overature;
                        debt += overature;
                    }
                    cruise_dist -= increment;
                    std::cout << "ov: " << overature << std::endl;
                    std::cout << "con" << (((cruise_dist >= 0) ^ isneg) ? std::pair<double, double>{increment, increment} : std::pair<double, double>{0, 0}).first << std::endl;
                    return ((cruise_dist >= 0) ^ isneg) ? std::pair<double, double>{increment, increment} : std::pair<double, double>{0, 0};
                });
            // decelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = (((acc_time - time) / 1000.0) * decel);
                    std::cout << tspeed << std::endl;
                    const double increment = (tspeed / 1000) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment, increment};
                },
                acc_time);
            return debt;
        }
    } // namespace p
} // namespace meia