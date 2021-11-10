#include "main.h"
namespace meia {
    void ChassisController::pid_loop(void* p) {
        int delay_time = ((Pid_task_messenger_struct*)p)->drive_task_delay_factor;
        std::string initmessage = "pid initiated";
        printf(initmessage.c_str());
        std::string printmessage = initmessage + " " + std::to_string(delay_time);
        while (true) {
            printf(printmessage.c_str());
            pros::delay(1000 * delay_time);
        }
    }
}