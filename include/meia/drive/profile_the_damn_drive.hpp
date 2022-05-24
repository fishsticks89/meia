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
         * A template to recieve a profiling function
         * @param int the time into the movement in ms
         * @param int the time since the last iteration
         * @return true to continue the profile
         */
        using pf_until_fn_t = std::function<bool(int, int)>;

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
        double arcturn(ChassisController* chassis, bool left, double amount, double speed, double acc, double drive_width);
        double debting_go(ChassisController* chassis, double amount, double speed, double decel);
        double debting_turn(ChassisController* chassis, double amount, double speed, double decel, double drive_width);
        void imuturn(meia::ChassisController* chassis, bool left, Imu* imu, double target, double speed, double acc, double drive_width, meia::Pid pd);
        double debting_go_heading_corr(ChassisController* chassis, Imu* imu, double amount, double speed, double decel, meia::Pid heading_pd, const double heading_target, const double drive_width);
    } // namespace p
} // namespace meia