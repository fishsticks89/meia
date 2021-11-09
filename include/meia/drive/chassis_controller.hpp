#include "main.h"
namespace meia {
    class ChassisController:private Chassis {
    public:
        /**
         * A Chassis that you can set the target position of each side and get error on
         *  \param left_motors
         *      Vector of left side's motor ports, negative if reversed
         *  \param left_motors
         *      Vector of right side's motor ports, negative if reversed
         *  \param wheel_diameter
         *      Remember that 4" wheels are actually 4.125"!
         *      Have the robot go 8ft forward and adjust this value until the robot actually goes 8ft
         *  \param motor_rpm
         *      Output RPM of the cartrage inside of your motor
         *      Turbo (Blue) - 600
         *      Speed (Green) - 200
         *      Torque (Red) - 100
         *  \param gear_ratio
         *      External drive ratio (MUST BE DECIMAL)
         *      eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
         *      eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
         */
        // ChassisController::ChassisController(std::vector<int> left_motors, std::vector<int> right_motors, double wheel_diameter, int motor_rpm, double gear_ratio);
    };
}