#include "main.h"
namespace meia {
  std::vector<int> left_motors;
  std::vector<int> right_motors;
  //! Constructor
  Chassis::Chassis(std::vector<int> p_left_motors, std::vector<int> p_right_motors) {
    left_motors = p_left_motors;
    right_motors = p_right_motors;
  }

  //! Utility Functions
  bool is_reversed(int input) {
    if (input < 0)
      return true;
    return false;
  }

  int sgn (int input) {
    if (input > 0)
      return 1;
    else if (input < 0)
      return -1;
    return 0;
  }

  double clip_num(double input, double max, double min) {
    if (input > max)
      return max;
    else if (input < min)
      return min;
    return input;
  }

  //! Motor Functions
  void
  Chassis::set_voltage(int l, int r) {
    for (int i : left_motors) {
      pros::c::motor_move_voltage(std::abs(i), sgn(i) * l * (12000.0/127.0));
    }
    for (int i : right_motors) {
      pros::c::motor_move_voltage(std::abs(i), sgn(i) * r * (12000.0/127.0));
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
    return (scale != 0) ? (powf(2.718, -(scale/10)) + powf(2.718, (abs(x)-127)/10) * (1-powf(2.718, -(scale/10))))*x : x;
  }

  // Tank control
  void Chassis::tank_control(pros::Controller con, double curve_intensity, int deadzone) {
    printf(std::to_string(con.get_analog(ANALOG_LEFT_Y)).c_str());
    // Threshold if joysticks don't come back to perfect 0
    if (std::abs(con.get_analog(ANALOG_LEFT_Y))>deadzone || std::abs(con.get_analog(ANALOG_RIGHT_Y))>deadzone) {
      set_voltage(curve_function(con.get_analog(ANALOG_LEFT_Y), curve_intensity), curve_function(con.get_analog(ANALOG_RIGHT_Y), curve_intensity));
    }
    // When joys are released, do nothing
    else {
      set_voltage(0, 0);
    }
  }
}