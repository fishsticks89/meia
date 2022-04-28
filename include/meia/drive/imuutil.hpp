#include "main.h"
class Imu {
    private:
        double round(double num) {
            return std::floor(num * 10 + 0.5) / 10;
        }

    public:
        pros::Imu imu;
        Imu(u_int8_t port)
            : imu(port) {
        }
        void calibrate() {
            std::cout << "calibratey" << std::endl;
            imu.reset();
            pros::delay(20);
            while (imu.is_calibrating()) {
                pros::delay(500);
            }
        }
        void zero() {
            imu.tare();
        }
        int getfac(bool fac) {
            return (fac) ? 1 : -1;
        }
        /**
         * Get the Inertial Sensor's yaw angle bounded by (-180,180)
         */
        double get_orientation() {
            // std::cout << imu.get_yaw() << std::endl;
            return imu.get_yaw();
        }
        double get_dist(bool left, double current, double target) {
            target = wrap(round(target));
            double dist = std::abs(current - target);
            if (left) {
                if (wrap(round(current - dist)) == target) {
                    return -dist;
                } else if (wrap(round(current + dist)) == target) {
                    return -(360 - dist);
                }
            } else if (!left) {
                if (wrap(round(current - dist)) == target) {
                    return 360 - dist;
                } else if (wrap(round(current + dist)) == target) {
                    return dist;
                }
            }
            std::cout << "did not reach t: " << target << " c: " << current << std::endl;
            throw "error";
            return 0;
        }
        double get_dist(bool left, double target) {
            double current = get_orientation();
            return get_dist(left, current, target);
        }
        bool get_dir(double target) {
            double current = get_orientation();
            return std::abs(get_dist(true, target)) < std::abs(get_dist(false, target));
        }
        double get_dist(double target) {
            double current = get_orientation();
            return (std::abs(get_dist(true, target)) < std::abs(get_dist(false, target)) ? get_dist(true, target) : get_dist(false, target));
        }
        static double wrap(double angle) {
            if (angle <= -180) {
                angle += 360;
            } else if (angle > 180) {
                angle -= 360;
            }
            if (-180 < angle <= 180) {
                return angle;
            } else {
                return wrap(angle);
            }
        }
};