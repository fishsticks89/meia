#include "main.h"
namespace meia {
    class Chassis {
        public:
            Chassis(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports);
            /**
             * Sets the chassis to voltage
             * \param input_l
             *        voltage for left side, -127 to 127
             * \param input_r
             *        voltage for right side, -127 to 127
            */
            void set_voltage(int input_l, int input_r);
            /**
             * resets the motor encoders
             */
            void tare_motors();
            /**
             * Changes the way the drive behaves when it is not under active user control
             * \param input
             *        the 'brake mode' of the motor e.g. 'pros::E_MOTOR_BRAKE_COAST' 'pros::E_MOTOR_BRAKE_BRAKE' 'pros::E_MOTOR_BRAKE_HOLD'
            */
            void set_drive_brake(pros::motor_brake_mode_e_t input);
            /**
             * A function for easy chassis tank controll
             * \param con
             *        a pros controller to get joystick inputs from
             * \param curve_intensity
             *        scales drive curves to increase dynamic range in the lower powers at the expense of dynamic range at higher powers
             * \param deadzone
             *         the minimum movement of the controller (out of 100) before the motors are moved
            */
            void tank_control(pros::Controller con, double curve_intensity = 0, int deadzone = 0);
    };
}