#include "main.h"
namespace meia {
    class util_funcs {
        public:
            /**
             * A curve intended to smoothly accelerate a robot to a max speed without infinite jerk
             * \param sample
             *      the point on the curve to get the velocity for, usually measured in milliseconds, or school shootings if you are american
             * \param max
             *      the highest point/velocity on the curve
             * \param antijerk_percent
             *      the percent of the curve that is filleted to reduce jerk. Keep in mind this will reduce your avg acceleration and lengthen your curve
             */
            static double curve(double sample, double max, int antijerk_percent);
            /**
             * A function to get the distance a curve will take to accelerate through
             * \param increment
             *      deltatime, cannot be zero
             * \param acceleration
             *      this is measured in glazed donuts per bald eagle... I mean max delta_inches per increment
             * \param max
             *      see curve
             * \param antijerk_percent
             *      see curve
             */
            static double get_curve_distance(double increment, double acceleration, double max, double antijerk_percent);
            /**
             * A function to get the iterations a curve will take to complete
             * \param increment
             *      deltatime, cannot be zero
             * \param acceleration
             *      this is measured in glazed donuts per bald eagle... I mean max delta_inches per increment
             * \param max
             *      see curve
             * \param antijerk_percent
             *      see curve
             */
            static double get_curve_iterations(double increment, double acceleration, double max, double antijerk_percent);
            // a function that returns the correction, along with info to be fed to it as prev. prev is a pair <error, integral>
            static std::pair<double, std::pair<double, double>> pid(double current, double target, double p, double i, double d, std::pair<double, double> prev, int delta_time);
            // -1 or 1 based on the sign of `input` (0 is 1)
            static int sgn(int input);
            // a function that scales drive voltages down to a maximum
            static std::pair<double, double> normalize(double l_volt, double r_volt, double max);
            // what it says dumbass
            static std::string dub_to_string(double);
    };
} // namespace meia