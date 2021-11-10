#include "main.h"
namespace meia {
    std::pair<double, double> ChassisController::set_voltage_normalized(double l_volt, double r_volt) {
        double l_dif = (std::abs(l_volt) > 127) ? std::abs(l_volt) - 127 : 0;
        double r_dif = (std::abs(r_volt) > 127) ? std::abs(l_volt) - 127 : 0;
        // step 1: scale down the voltages similarly if they are both over 127
        // step 2: return it idk
    }
    void ChassisController::pid_loop(void* p) {
        pros::delay(200);
        int delay_time = *(int*)p;
        std::string initmessage = "pid initiated";
        printf(initmessage.c_str());
        std::string printmessage = initmessage;
        int times = 0;
        while (true) {
            pros::lcd::set_text(5, printmessage.c_str());
            printmessage = initmessage + " " + std::to_string(times) + " " + std::to_string(delay_time);
            pros::delay(1000);
            times++;
            delay_time = *(int*)p;
        }
    }
}