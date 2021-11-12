#include "main.h"
namespace meia {
    //! The Task
    void ChassisController::set_voltage_normalized(double l_volt, double r_volt) {
        double l_dif = (std::abs(l_volt) > 127) ? std::abs(l_volt) - 127 : 0;
        double r_dif = (std::abs(r_volt) > 127) ? std::abs(l_volt) - 127 : 0;
        // scale down the voltages similarly if one is over 127
        bool greater_dif = l_dif > r_dif;
        double dif = (greater_dif) ? l_dif : r_dif;
        double l_out = (127/(127 + dif)) * l_volt;
        double r_out = (127/(127 + dif)) * r_volt;
        set_voltage(l_out, r_out);  
    }
    void ChassisController::pid_loop(void* p) {
        pros::delay(200);
        Pid_task_messenger_struct input = *(Pid_task_messenger_struct*)p;
        if (!(3 >= input.drive_task_delay_factor > 0))
            throw "delay_time is measured in centiseconds and can only be 1 - 3";
        std::string initmessage = "pid initiated";
        printf(initmessage.c_str());
        std::string printmessage = initmessage;
        int times = 0;
        while (true) {
            if (times > 20)
                (*input.self_ptr).suspend();
            pros::lcd::set_text(5, printmessage.c_str());
            printmessage = initmessage + " " + std::to_string(times) + " " + std::to_string(input.drive_task_delay_factor);
            pros::delay(1000);
            times++;
            input = *(Pid_task_messenger_struct*)p;
        }
    }
}