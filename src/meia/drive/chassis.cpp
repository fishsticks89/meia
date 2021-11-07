#pragma once
#include "main.h"
namespace meia {
  std::vector<int> LL_MOTOR_PORTS;
  std::vector<int> RR_MOTOR_PORTS;
  //! Constructor
  Chassis::Chassis(std::vector<int> pLL_MOTOR_PORTS, std::vector<int> pRR_MOTOR_PORTS) {
    // initializes motor reversing
    LL_MOTOR_PORTS.assign(pLL_MOTOR_PORTS.begin(), pLL_MOTOR_PORTS.end());
    RR_MOTOR_PORTS.assign(pRR_MOTOR_PORTS.begin(), pRR_MOTOR_PORTS.end());
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
    for (int i : LL_MOTOR_PORTS) {
      pros::c::motor_move_voltage(std::abs(i), sgn(i) * l * (12000.0/127.0));
    }
    for (int i : RR_MOTOR_PORTS) {
      pros::c::motor_move_voltage(std::abs(i), sgn(i) * r * (12000.0/127.0));
    }
  }

  // Brake modes
  void
  Chassis::set_drive_brake(pros::motor_brake_mode_e_t input) {
    for (int i : LL_MOTOR_PORTS) {
      pros::c::motor_set_brake_mode(abs(i), input);
    }
    for (int i : RR_MOTOR_PORTS) {
      pros::c::motor_set_brake_mode(abs(i), input);
    }
  }

  double curve_function(int x, double scale) {
      return (powf(2.718, -(scale/10)) + powf(2.718, (abs(x)-127)/10) * (1-powf(2.718, -(scale/10))))*x;
  }

  int deadzone;

  void Chassis::set_deadzone(int threshold) {
      deadzone = threshold;
  }

  // Tank control
  void Chassis::tank_control(pros::Controller con, int curve_intensity = 0) {
    // Threshold if joysticks don't come back to perfect 0
    if (std::abs(con.get_analog(ANALOG_LEFT_Y))>deadzone || std::abs(con.get_analog(ANALOG_RIGHT_Y))>deadzone) {
      set_voltage(curve_function(con.get_analog(ANALOG_LEFT_Y), curve_intensity), curve_function(con.get_analog(ANALOG_RIGHT_Y), curve_intensity));
    }
    // When joys are released, run active brake (P) on drive
    else {
      set_voltage(0, 0);
    }
  }
}