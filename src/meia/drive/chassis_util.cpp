#include <cmath>
#include <string>
#include <utility>
namespace meia {
    // cos but degrees
    inline static const double degrees_to_radians = 0.01745329252;
    inline static double dcos(double x) {
        return std::cos(x * degrees_to_radians);
    }
    // sin but degrees
    inline static double dsin(double x) {
        return std::sin(x * degrees_to_radians);
    }

    inline double curve(double sample, double max, int antijerk_percent) {
        const double radius = antijerk_percent * ((max / (2 * (1 - dsin(45)))) / 100);
        if (sample < 0) {
            throw "out of bounds";
        } else if (sample < dcos(-45) * radius) {
            return radius - std::sqrt(std::pow(radius, 2) - std::pow(sample, 2));
        } else if (sample < (max - (radius - (std::abs(dsin(225))) * radius) - (radius - (dsin(45) * radius)) + (dcos(-45) * radius))) {
            return (sample - dcos(-45) * radius) + (radius - dsin(45) * radius);
        } else if (sample < (max - (radius - (std::abs(sin(225)) * radius) - (radius - sin(45) * radius)) + (cos(-45) * radius) + (cos(225) * radius))) {
            return (max - radius) + std::sqrt(std::pow(radius, 2) - std::pow((sample - (max - (radius - (std::abs(dsin(225)) * radius)) - (radius - (dsin(45) * radius)) + (dcos(-45) * radius) + (std::abs(dcos(225)) * radius))), 2));
        } else {
            return max;
        }
    }
    inline double get_curve_distance(double increment, double max, double antijerk_percent) {
        if (increment <= 0)
            throw "too small dumbass";
        double total = 0;
        double i = increment;
        while (curve(i, max, antijerk_percent) < max) {
            total += curve(i, max, antijerk_percent);
            i += increment;
        }
        return total;
    }
    /**
     * \param increment
     *      how often the curve is sampled
     * \param acceleration
     *      the multiplier on the speed of the curve
     * \param
     */
    inline int get_curve_iterations(double increment, double max, double antijerk_percent) {
        if (increment <= 0)
            throw "too small dumbass";
        double total = 0;
        double i = increment;
        while (curve(i, max, antijerk_percent) < max) {
            total += curve(i, max, antijerk_percent);
            i += increment;
        }
        return i / increment - 1;
    }

    inline std::pair<double, std::pair<double, double>> pid(double current, double target, double p, double i, double d, std::pair<double, double> prev, int delta_time) {
        // p constant is translated into correction
        double p_correct = target - current;
        // new integral and prev error is calculated
        double new_i = prev.second + p_correct;
        double new_prev_error = p_correct;
        // i and d constants are translated into corrections
        double d_correct = prev.first - p_correct;
        double i_correct = new_i;
        // correction is calculated
        double error = (p_correct * p) + (i_correct * i) + (d_correct * d);
        return {error / delta_time, {new_prev_error, new_i}};
    }

    inline int sgn(int input) {
        return (input > 0) ? 1 : -1;
    }

    inline std::pair<double, double> normalize(double l_volt, double r_volt, double max) {
        double l_dif = (std::abs(l_volt) > max) ? std::abs(l_volt) - max : 0;
        double r_dif = (std::abs(r_volt) > max) ? std::abs(r_volt) - max : 0;
        // scale down the voltages similarly if one is over 127
        bool greater_dif = l_dif > r_dif;
        double dif = (greater_dif) ? l_dif : r_dif;
        double l_out = (max / (max + dif)) * l_volt;
        double r_out = (max / (max + dif)) * r_volt;
        return {l_out, r_out};
    }

    inline std::string tokenize(std::string s, std::string del = " ") {
        int start = 0;
        int end = s.find(del);
        return s.substr(start, end);
    }

    inline std::string dub_to_string(double d) {
        return (tokenize(std::to_string(d), "."));
    }

    inline double get_dist(bool left, double current, double target) {
        if (target < current)
            return target - current;
        else if (left)
            return ((target - current) - 360);
        else if (!left)
            return (360 + (target - current));
        throw "internalerr";
    }

    inline int get_fac(bool fac) {
        return (2 * (fac - 0.5));
    }

} // namespace meia