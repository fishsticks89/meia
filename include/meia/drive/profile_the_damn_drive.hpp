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
         * Executes a profiling function for a given time, and when completed
         */
        void profile(ChassisController* chassis, profiling_fn_t profiling_fn, int time);

        /**
         * Accelerates to a speed and decelerates at a given distance
         * @param amount dist in inches
         * @param speed the cruise speed in in/sec
         * @param acc acceleration
         */
        void go(ChassisController* chassis, int amount, int speed, int acc);
    } // namespace p
} // namespace meia