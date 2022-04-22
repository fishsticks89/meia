#include "main.h"
namespace meia {
    namespace p {
        /**
         * A template to recieve a profiling function
         * @param int the time into the movement in ms
         * @param int the time since the last iteration
         * @return the delta chassis targets
         */
        using profiling_fn_t = std::function<std::pair<double, double>(int, int)>;

        /**
         * Executes a profiling function for a given time, and when completed returns the avg error.
         * @return avg error.
         */
        double profile(ChassisController* chassis, profiling_fn_t profiling_fn, int time);

        /**
         * Accelerates to a speed and decelerates at a given distance
         * @param amount dist in inches
         * @param speed the cruise speed in in/sec
         * @param acc acceleration
         * @return avg error
         */
        double go(ChassisController* chassis, double amount, double speed, double acc);

        /**
         * Accelerates to a speed and decelerates at a given angle
         * @param amount dist in deg
         * @param speed the cruise speed in deg/sec
         * @param acc acceleration dec/sec^2
         * @return avg error
         */
        double turn(ChassisController* chassis, double amount, double speed, double acc, double drive_width);
        double debting_go(ChassisController* chassis, int amount, int speed, int decel);
    } // namespace p
} // namespace meia