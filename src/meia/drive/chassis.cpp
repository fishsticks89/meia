#include "main.h"
namespace meia {
    //! Motor Functions
    void
    Chassis::set_voltage(std::pair<int, int> voltages) {
        for (int i : left_motors) {
            pros::c::motor_move_voltage(std::abs(i), util.sgn(i) * voltages.first * (12000.0 / 127.0));
        }
        for (int i : right_motors) {
            pros::c::motor_move_voltage(std::abs(i), util.sgn(i) * voltages.second * (12000.0 / 127.0));
        }
    }

    // Tare
    void
    Chassis::tare_motors() {
        for (int i : left_motors) {
            pros::c::motor_tare_position(abs(i));
        }
        for (int i : right_motors) {
            pros::c::motor_tare_position(abs(i));
        }
    }

    // get the ticks of each side
    std::pair<double, double>
    Chassis::get_motor_positions() {
        return {
        util.sgn(left_motors[0]) * pros::c::motor_get_position(abs(left_motors[0])), 
        util.sgn(right_motors[0]) * pros::c::motor_get_position(abs(right_motors[0]))
        };
    }

    // get the temps of each motor starting with left side
    std::pair<std::vector<double>, std::vector<double>>
    Chassis::get_motor_temps() {
        std::vector<double> l_temps;
        std::vector<double> r_temps;
        for (auto i : left_motors) {
            l_temps.push_back(pros::c::motor_get_temperature(abs(i)));
        }
        for (auto i : right_motors) {
            r_temps.push_back(pros::c::motor_get_temperature(abs(i)));
        }
        std::pair temps = {l_temps, r_temps};
        return temps;
    }

    // Brake modes
    void
    Chassis::set_drive_brake(pros::motor_brake_mode_e_t input) {
        for (int i : left_motors) {
            pros::c::motor_set_brake_mode(abs(i), input);
        }
        for (int i : right_motors) {
            pros::c::motor_set_brake_mode(abs(i), input);
        }
    }

    double curve_function(int x, double scale) {
        return (scale != 0 && x != 100) ? (powf(2.718, -(scale / 10)) + powf(2.718, (abs(x) - 127) / 10) * (1 - powf(2.718, -(scale / 10)))) * x : x; // a ternerary to improve performance
    }

    // Tank control
    void Chassis::tank_control(pros::Controller con, double curve_intensity, int deadzone) {
        // Threshold if joysticks don't come back to perfect 0
        if (std::abs(con.get_analog(ANALOG_LEFT_Y)) > deadzone || std::abs(con.get_analog(ANALOG_RIGHT_Y)) > deadzone) {
            set_voltage({curve_function(con.get_analog(ANALOG_LEFT_Y), curve_intensity), curve_function(con.get_analog(ANALOG_RIGHT_Y), curve_intensity)});
        }
        // When joys are released, do nothing
        else {
            set_voltage({0, 0});
        }
    }
} // namespace meia