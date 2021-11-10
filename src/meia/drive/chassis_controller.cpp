#include "main.h"
namespace meia {
    std::pair<double, double> ChassisController::set_voltage_normalized(double l_volt, double r_volt) {
        double l_dif = (std::abs(l_volt) > 127) ? std::abs(l_volt) - 127 : 0;
        double r_dif = (std::abs(r_volt) > 127) ? std::abs(l_volt) - 127 : 0;
        // step 1: scale down the voltages similarly if they are both over 127
        bool greater_dif = l_dif > r_dif;
        double dif = (greater_dif) ? l_dif : r_dif;
        double l_fac = (greater_dif) ? 1 : r_dif / l_dif;
        double r_fac = (greater_dif) ? l_dif / r_dif : 1;
        // step 2: return it idk
        return {1.0, 1.0}; // placeholder       
    }
    void ChassisController::pid_loop(void* p) {
        pros::delay(200);
        Pid_task_messenger_struct io = *(Pid_task_messenger_struct*)p;
        std::string initmessage = "pid initiated";
        printf(initmessage.c_str());
        std::string printmessage = initmessage;
        int times = 0;
        while (true) {
            pros::lcd::set_text(5, printmessage.c_str());
            printmessage = initmessage + " " + std::to_string(times) + " " + std::to_string(io.drive_task_delay_factor);
            pros::delay(1000);
            times++;
            io = *(Pid_task_messenger_struct*)p;
        }
    }
}