#include "main.h"
class Motor {
    public:
        pros::Motor motor;
        bool reversed;
        double p = 1;
        double target;
        static void setupTask();
        Motor(int_fast16_t port, bool reversed)
            : motor(port), reversed(reversed){};
        Motor(int_fast16_t port, double correct, bool reversed)
            : motor(port), reversed(reversed), p(correct){};
        void set_voltage(int_fast32_t voltage) {
            motor.move_voltage(voltage * ((reversed - 0.5) * 2));
        }
};