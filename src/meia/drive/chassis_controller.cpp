#include "main.h"
namespace meia {
    double wheel_diameter;
    int motor_rpm;
    double gear_ratio;
    ChassisController::ChassisController(std::vector<int> p_left_motors, std::vector<int> p_right_motors, double p_wheel_diameter, int p_motor_rpm, double p_gear_ratio) {
        Chassis(p_left_motors, p_right_motors);
        wheel_diameter = p_wheel_diameter;
        motor_rpm = p_motor_rpm;
        gear_ratio = p_gear_ratio;
    }
}