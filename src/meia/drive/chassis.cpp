#include "main.h"
namespace meia
{

  //! Utility Functions
  bool is_reversed(int input)
  {
    return (input < 0) ? true : false;
  }

  int sgn(int input)
  {
    return (input > 0) ? 1 : -1;
  }

  double clip_num(double input, double max, double min)
  {
    if (min < input < max)
      return input;
    else
      return (min >= max) ? max : min;
  }

  //! Motor Functions
  void
  Chassis::set_voltage(int l, int r)
  {
    for (int i : left_motors)
    {
      pros::c::motor_move_voltage(std::abs(i), sgn(i) * l * (12000.0 / 127.0));
    }
    for (int i : right_motors)
    {
      pros::c::motor_move_voltage(std::abs(i), sgn(i) * r * (12000.0 / 127.0));
    }
  }

  // Tare
  void
  Chassis::tare_motors()
  {
    for (int i : left_motors)
    {
      pros::c::motor_tare_position(abs(i));
    }
    for (int i : right_motors)
    {
      pros::c::motor_tare_position(abs(i));
    }
  }

  // Tare
  std::pair<double, double> 
  Chassis::get_motor_positions()
  {
    return {pros::c::motor_get_position(abs(left_motors[0])), pros::c::motor_get_position(abs(right_motors[0]))};
  }

  // Brake modes
  void
  Chassis::set_drive_brake(pros::motor_brake_mode_e_t input)
  {
    for (int i : left_motors)
    {
      pros::c::motor_set_brake_mode(abs(i), input);
    }
    for (int i : right_motors)
    {
      pros::c::motor_set_brake_mode(abs(i), input);
    }
  }

  double curve_function(int x, double scale)
  {
    return (scale != 0 && x != 100) ? (powf(2.718, -(scale / 10)) + powf(2.718, (abs(x) - 127) / 10) * (1 - powf(2.718, -(scale / 10)))) * x : x * 1.27; // a ternerary to improve performance
  }

  // Tank control
  void Chassis::tank_control(pros::Controller con, double curve_intensity, int deadzone)
  {
    printf(std::to_string(con.get_analog(ANALOG_LEFT_Y)).c_str());
    // Threshold if joysticks don't come back to perfect 0
    if (std::abs(con.get_analog(ANALOG_LEFT_Y)) > deadzone || std::abs(con.get_analog(ANALOG_RIGHT_Y)) > deadzone)
    {
      set_voltage(curve_function(con.get_analog(ANALOG_LEFT_Y), curve_intensity), curve_function(con.get_analog(ANALOG_RIGHT_Y), curve_intensity));
    }
    // When joys are released, do nothing
    else
    {
      set_voltage(0, 0);
    }
  }
}