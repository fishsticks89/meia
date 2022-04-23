#include "chassis_util.cpp"
#include "main.h"

double getBigger(std::pair<double, double> e) {
    return std::max(e.first, e.second);
}
std::pair<double, double> multiply(std::pair<double, double> a, double b) {
    return {a.first * b, a.second * b};
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
        double profileuntil(ChassisController* chassis, pf_until_fn_t pf_until_fn) {
            const int delay = chassis->get_delay_time();
            for (int time = 0; pf_until_fn(time, delay); time += delay) {
                pros::delay(delay);
            }
            return 0;
        }
        double go(ChassisController* chassis, double amount, double speed, double acc) {
            const int acc_time = (speed / acc) * 1000;                                         // ms
            const int cruise_time = ((amount - ((acc_time / 1000.0) * speed)) / speed) * 1000; // ms
            if (cruise_time < 0) {
                throw " ur mom ";
            }
            // accelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = ((time / 1000.0) * acc);
                    const double increment = (tspeed / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
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
                    const double increment = (tspeed / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment, increment};
                },
                acc_time);
            return 0;
        }
        double turn(ChassisController* chassis, double amount, double speed, double acc, double drive_width) {
            const double amount_in = (amount / 360.0) * (drive_width * m_pi);
            const double speed_in = (speed / 360.0) * (drive_width * m_pi);
            const double acc_in = (acc / 360.0) * (drive_width * m_pi);
            std::cout << "amount: " << amount_in << std::endl;
            const int acc_time = (speed / acc) * 1000; // ((in/sec) / (in/sec^2)) * (ms/sec); ms
            const double acc_dist = (acc_time / 1000.0) * speed_in;
            std::cout << "acc_dist: " << acc_dist << std::endl;
            const double cruise_dist = std::abs(amount_in) - acc_dist; // in - ((sec * deg/sec)) * (drive_width * m_pi / 360.0))
            std::cout << "cdist: " << cruise_dist << std::endl;
            const int cruise_time = (cruise_dist / speed_in) * 1000;
            if (cruise_time < 0)
                throw "acc too low";
            bool ispos = amount_in > 0;
            // accelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = ((time / 1000.0) * acc_in);
                    const double increment = (tspeed / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment * get_fac(ispos), -increment * get_fac(ispos)};
                },
                acc_time);
            // cruise
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double increment = (speed_in / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment * get_fac(ispos), -increment * get_fac(ispos)};
                },
                cruise_time);
            // decelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = (((acc_time - time) / 1000.0) * acc_in); // ((sec) / (msec/sec)) * in/sec^2
                    const double increment = (tspeed / 1000.0) * delta_time;       // converts in/sec to in/msec, multiplies by delta_time
                    return {increment * get_fac(ispos), -increment * get_fac(ispos)};
                },
                acc_time);
            return 0;
        }

        double debting_go(ChassisController* chassis, double amount, double speed, double decel) {
            double debt = 0;
            const int acc_time = (speed / decel) * 1000.0;
            double cruise_dist = (std::abs(amount) - ((acc_time / 1000.0) * speed / 2.0));
            bool ispos = amount > 0;
            chassis->allowderivative(false);
            profileuntil(
                chassis, [&](int time, int delta_time) -> bool {
                    double increment = (speed / 1000.0) * delta_time;
                    const double overature = (getBigger(multiply(chassis->get_error(), get_fac(ispos))) * chassis->get_p_constant() - 135) / chassis->get_p_constant(); // 135 instead of 127 to ensure always some overature
                    if (overature > 0) {
                        increment -= overature;
                        debt += overature;
                    }
                    cruise_dist -= increment;
                    chassis->change_target({increment * get_fac(ispos), increment * get_fac(ispos)});
                    return (cruise_dist > 0);
                });
            chassis->allowderivative(true);
            // decelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = (((acc_time - time) / 1000.0) * decel);
                    const double increment = (tspeed / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment * get_fac(ispos), increment * get_fac(ispos)};
                },
                acc_time);
            return debt;
        }
        double debting_turn(ChassisController* chassis, double amount, double speed, double decel, double drive_width) {
            const double amount_in = (amount / 360.0) * (drive_width * m_pi);
            const double speed_in = (speed / 360.0) * (drive_width * m_pi);
            const double decel_in = (decel / 360.0) * (drive_width * m_pi);
            double debt = 0;
            const int acc_time = std::round((speed / decel) * 1000.0);                   // ms
            double cruise_dist = std::abs(amount_in) - ((acc_time / 1000.0) * speed_in); // ms
            if (cruise_dist < 0)
                throw "acc too low";
            bool ispos = amount > 0;
            chassis->allowderivative(false);
            profileuntil(
                chassis, [&](int time, int delta_time) -> bool {
                    double increment = (speed_in / 1000.0) * delta_time;
                    const double overature = (getBigger(multiply({chassis->get_error().first, -chassis->get_error().second}, get_fac(ispos))) * chassis->get_p_constant() - 135.0) / chassis->get_p_constant(); // 135 instead of 127 to ensure always some overature
                    if (overature > 0) {
                        increment -= overature;
                        debt += overature;
                    }
                    cruise_dist -= increment;
                    chassis->change_target({increment, -increment});
                    return (cruise_dist > 0);
                });
            chassis->allowderivative(true);
            // decelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = (((acc_time - time) / 1000.0) * decel_in);
                    const double increment = (tspeed / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment, -increment};
                },
                acc_time);
            return debt;
        }
    } // namespace p
} // namespace meia