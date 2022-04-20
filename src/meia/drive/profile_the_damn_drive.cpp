#include "main.h"

namespace meia {
    namespace p {
        void profile(ChassisController* chassis, profiling_fn_t profiling_fn, int duration) {
            std::cout << " pftime " << std::endl;
            const int delay = chassis->get_delay_time();
            for (int time = 0; time < duration; time += delay) {
                std::cout << profiling_fn(time, delay).first << std::endl;
                chassis->change_target(profiling_fn(time, delay));
                pros::delay(delay);
            }
        }
        void go(ChassisController* chassis, int amount, int speed, int acc) {
            const double acc_time = (speed / acc) * 1000; // ms
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
                ((amount - (acc_time * speed)) / speed) * 1000);
            // decelerate
            profile(
                chassis, [&](int time, int delta_time) -> std::pair<double, double> {
                    const double tspeed = (((acc_time - time) / 1000) * acc);
                    const double increment = (tspeed / 1000) * delta_time; // converts in/sec to in/msec, multiplies by delta_time
                    return {increment, increment};
                },
                (speed / acc) * 1000);
        }
    } // namespace p
} // namespace meia