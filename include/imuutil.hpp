#include "main.h"
class Imu {
    public: 
    pros::Imu imu;
    Imu(u_int8_t port) : imu(port) {}
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
        std::cout << imu.get_yaw() << std::endl;
        return imu.get_yaw();
    }
    double get_dist(bool left, double target) {
        double current = get_orientation();
  if (target < -180 || target > 180)
      throw "out of bounds";
  double dist = std::abs(current - target);
  if ((current + dist == target) ^ !left) {
    std::cout << " first ";
    return (360 - dist) * getfac(!left);
  } else if ((current - dist == target) ^ !left) {
    std::cout << " second ";
    return dist * getfac(!left);
  }
  throw "did not reach";
}
};