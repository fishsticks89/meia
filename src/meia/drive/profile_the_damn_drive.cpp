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
                std::cout << "1ee" << std::endl;
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
            if (cruise_time < 0) {
                std::cout << "2ee" << std::endl;
                throw "acc too low";
            }
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
        double arcturn(ChassisController* chassis, bool left, double amount, double speed, double acc, double drive_width) {
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
            if (cruise_time < 0) {
                std::cout << "3ee" << std::endl;
                throw "acc too low";
            }
            bool ispos = amount_in > 0;
            // accelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = ((time / 1000.0) * acc_in);
                    const double increment = (tspeed / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    if (left) {
                        return {increment * get_fac(ispos) * 2, 0};
                    } else {
                        return {0, -increment * get_fac(ispos) * 2};
                    }
                },
                acc_time);
            // cruise
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double increment = (speed_in / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    if (left) {
                        return {increment * get_fac(ispos) * 2, 0};
                    } else {
                        return {0, -increment * get_fac(ispos) * 2};
                    }
                },
                cruise_time);
            // decelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = (((acc_time - time) / 1000.0) * acc_in); // ((sec) / (msec/sec)) * in/sec^2
                    const double increment = (tspeed / 1000.0) * delta_time;       // converts in/sec to in/msec, multiplies by delta_time
                    if (left) {
                        return {increment * get_fac(ispos) * 2, 0};
                    } else {
                        return {0, -increment * get_fac(ispos) * 2};
                    }
                },
                acc_time);
            return 0;
        }

        double debting_go(ChassisController* chassis, double amount, double speed, double decel) {
            double debt = 0;
            const int acc_time = (speed / decel) * 1000.0;
            double cruise_dist = (std::abs(amount) - ((acc_time / 1000.0) * speed / 2.0));
            bool ispos = amount > 0;
            if (cruise_dist < 0) {
                std::cout << "4ee" << std::endl;
                throw "acc too low";
            }
            chassis->allowderivative(false);
            profileuntil(
                chassis, [&](int time, int delta_time) -> bool {
                    double increment = (speed / 1000.0) * delta_time;
                    const double overature = (getBigger(multiply(chassis->get_error_legacy(), get_fac(ispos))) * chassis->get_p_constant() - 135) / chassis->get_p_constant(); // 135 instead of 127 to ensure always some overature
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
                    const double tspeed = (((acc_time - (time + (0.5 * delta_time))) / 1000.0) * decel);
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
            const int acc_time = std::round((speed / decel) * 1000.0);                           // ms
            double cruise_dist = std::abs(amount_in) - (((acc_time / 1000.0) * speed_in) / 2.0); // ms
            if (cruise_dist < 0) {
                std::cout << "5ee" << std::endl;
                throw "acc too low";
            }
            bool ispos = amount > 0;
            chassis->allowderivative(false);
            profileuntil(
                chassis, [&](int time, int delta_time) -> bool {
                    double increment = (speed_in / 1000.0) * delta_time;
                    const double overature = (getBigger(multiply({chassis->get_error_in().first, -chassis->get_error_in().second}, get_fac(ispos))) * chassis->get_p_constant() - 135.0) / chassis->get_p_constant(); // 135 instead of 127 to ensure always some overature
                    if (overature > 0) {
                        increment -= overature;
                        debt += overature;
                    }
                    cruise_dist -= increment;
                    chassis->change_target({increment * get_fac(ispos), -increment * get_fac(ispos)});
                    return (cruise_dist > 0);
                });
            chassis->allowderivative(true);
            // decelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = (((acc_time - time) / 1000.0) * decel_in);
                    const double increment = (tspeed / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment * get_fac(ispos), -increment * get_fac(ispos)};
                },
                acc_time);
            return debt;
        }

        void logtarg(std::string tt) {
            int t = pros::millis();
            if ((t - (t % 5)) % 1000 == 0) {
                std::cout << tt << std::endl;
            }
        }

        void imuturn(meia::ChassisController* chassis, bool left, Imu* imu, double target, double speed, double acc, double drive_width, meia::Pid pd) {
            target = imu->wrap(target);
            double amount = imu->get_dist(left, target);
            std::cout << "speed: " << speed << "acc: " << acc << std::endl;
            const int acc_time = (speed / acc) * 1000;
            std::cout << "acc_time: " << acc_time << std::endl;
            const double acc_angle = (acc_time / 1000.0) * speed;
            std::cout << "amount: " << amount << "acc_angle: " << acc_angle << std::endl;
            const double cruise_angle = std::abs(amount) - acc_angle;
            std::cout << "cdist: " << cruise_angle << std::endl;
            const int cruise_time = (cruise_angle / speed) * 1000;
            std::cout << "cruise_time: " << cruise_time << std::endl;
            if (cruise_time < 0) {
                std::cout << "6ee" << std::endl;
                throw "acc too low";
            }
            const double start_angle = imu->get_orientation();
            double prev_theoretical_err = 0;
            std::function<double(double)> pidloop = [&](double delta_time) -> double {
                delta_time /= 1000;
                const double err = imu->get_dist(imu->wrap(target + start_angle));
                auto chassis_err = chassis->get_error_in();
                double theoretical_error = err + (((chassis_err.second - chassis_err.first) / (drive_width * m_pi)) * 360);
                theoretical_error = imu->wrap(theoretical_error);
                const double deltatarget = ((pd.p * theoretical_error) + (pd.d * (theoretical_error - prev_theoretical_err)) * (drive_width * m_pi)) * delta_time;
                prev_theoretical_err = theoretical_error;
                std::cout << "theo: " << theoretical_error << " acc: " << err << " diff: " << err - theoretical_error << " derr: " << chassis_err.second << std::endl;
                chassis->change_target({deltatarget, -deltatarget});
                return err;
            };
            target = 0;
            // accelerate
            profileuntil(
                chassis, [&](int time, int delta_time) -> bool {
                    const double tspeed = ((time / 1000.0) * acc);
                    const double increment = (tspeed / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    target += increment * get_fac(!left);
                    pidloop(delta_time);
                    return std::abs(target) < (acc_angle / 2);
                });
            // cruise
            profileuntil(
                chassis, [&](int time, int delta_time) -> bool {
                    const double increment = (speed / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    target += increment * get_fac(!left);
                    pidloop(delta_time);
                    return std::abs(target) < std::abs(amount) - (acc_angle / 2);
                });
            // decelerate
            profileuntil(
                chassis, [&](int time, int delta_time) -> bool {
                    const double tspeed = (((acc_time - time) / 1000.0) * acc); // ((sec) / (msec/sec)) * in/sec^2
                    const double increment = (tspeed / 1000.0) * delta_time;
                    target += std::abs(increment) * get_fac(!left);
                    const double erroror = pidloop(delta_time);
                    return std::abs(target) < std::abs(amount);
                });
            int settlestart = pros::millis();
            double preverr = 0;
            profileuntil(
                chassis, [&](int time, int delta_time) -> bool {
                    const double error = pidloop(delta_time);
                    // within 3 deg of target and < 2 deg speed
                    bool should_continue = (std::abs(error) > 3 || std::abs(error - preverr) > ((2 / 1000.0) * delta_time)) && ((pros::millis() - settlestart < 1000) && std::abs(error) > 6);
                    preverr = error;
                    return should_continue;
                });
            std::cout << "Settletime " << pros::millis() - settlestart << std::endl;
        }
        double debting_go_heading_corr(ChassisController* chassis, Imu* imu, double amount, double speed, double decel, meia::Pid heading_pd, const double heading_target, const double drive_width) {
            double debt = 0;
            const int acc_time = (speed / decel) * 1000.0;
            double cruise_dist = (std::abs(amount) - ((acc_time / 1000.0) * speed / 2.0));
            bool ispos = amount > 0;
            double prev_theoretical_err = 0;
            std::function<double(double)> pidloop = [&](double delta_time) -> double {
                delta_time /= 1000;
                const double err = imu->get_dist(imu->wrap(heading_target));
                auto chassis_err = chassis->get_error_in();
                double theoretical_error = err + (((chassis_err.second - chassis_err.first) / (drive_width * m_pi)) * 360);
                theoretical_error = imu->wrap(theoretical_error);
                const double deltatarget = ((heading_pd.p * theoretical_error) + (heading_pd.d * (theoretical_error - prev_theoretical_err)) * (drive_width * m_pi)) * delta_time;
                prev_theoretical_err = theoretical_error;
                // std::cout << "theo: " << theoretical_error << " acc: " << err << " diff: " << err - theoretical_error << " derr: " << chassis_err.second << std::endl;
                chassis->change_target({deltatarget, -deltatarget});
                return err;
            };
            if (cruise_dist < 0) {
                std::cout << "7ee" << std::endl;
                throw "acc too low";
            }
            chassis->allowderivative(false);
            profileuntil(
                chassis, [&](int time, int delta_time) -> bool {
                    double increment = (speed / 1000.0) * delta_time;
                    const double overature = (getBigger(multiply(chassis->get_error_legacy(), get_fac(ispos))) * chassis->get_p_constant() - 135) / chassis->get_p_constant(); // 135 instead of 127 to ensure always some overature
                    if (overature > 0) {
                        increment -= overature;
                        debt += overature;
                    }
                    cruise_dist -= increment;
                    pidloop(delta_time);
                    chassis->change_target({increment * get_fac(ispos), increment * get_fac(ispos)});
                    return (cruise_dist > 0);
                });
            chassis->allowderivative(true);
            // decelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = (((acc_time - (time + (0.5 * delta_time))) / 1000.0) * decel);
                    const double increment = (tspeed / 1000.0) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment * get_fac(ispos), increment * get_fac(ispos)};
                },
                acc_time);
            return debt;
        }
    } // namespace p
} // namespace meia