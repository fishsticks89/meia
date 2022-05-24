#include "main.h"
namespace meia {
    /**
     * A Chassis that you can set the voltage of each side to and get telemetry on
     * \param left_motors
     *      Vector of left side's motor ports, negative if reversed
     *  \param left_motors
     *      Vector of right side's motor ports, negative if reversed
     */
    class Chassis {
        private:
            std::vector<int> left_motors;
            std::vector<int> right_motors;
            static std::vector<int>* p_left_motors;
            static std::vector<int>* p_right_motors;

        public:
            explicit Chassis(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports)
                : left_motors(left_motor_ports), right_motors(right_motor_ports){};
            /**
             * Sets the chassis to voltage
             * \param voltages
             *        voltages for each side, left first, -127 to 127
             */
            void set_voltage(std::pair<int, int> voltages);
            /**
             * resets the motor encoders
             */
            void tare_motors();
            /**
             * gets the avg motor encoder value
             */
            std::pair<double, double> get_motor_positions();
            /**
             * gets the temperature of each motor in the drive (left side first)
             */
            std::pair<std::vector<double>, std::vector<double>> get_motor_temps();
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
            void tank_control(pros::Controller* con = new pros::Controller(pros::E_CONTROLLER_MASTER), double curve_intensity = 0, int deadzone = 0);
    };
} // namespace meia