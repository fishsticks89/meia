#include "main.h"
class Motor {
    private:
        static void task_fn(void* params) {
            auto fn = static_cast<std::function<void()>*>(params);
            (*fn)();
        }
        pros::Task task;
        std::function<void()> motor_loop = [&]() {
            while (true) {
                if (runpid) {
                    motor.move_voltage((((!reversed) ? pidtarget : -pidtarget) * fac - motor.get_position()) * p);
                }
                pros::delay(5);
            }
        };
        pros::Motor motor;
    public:
        static double get_motor_fac(int motor_ratio, double gear_ratio) {
            return ((50.0 * motor_ratio) * gear_ratio) / 360.0;
        }
        bool reversed;
        double p = 1;
        double fac = 1;
        bool runpid = false;
        double pidtarget = 0;
        Motor(int_fast16_t port, double correct, bool reversed)
            : motor(port), reversed(reversed), p(correct), task(task_fn, &motor_loop, "balls") {
            };
        void set_fac(double pfac) {
            fac = pfac;
        }
        void set_voltage(int_fast32_t voltage) {
            runpid = false;
            motor.move_voltage(voltage * ((!reversed - 0.5) * 2));
        }
        void set_target(double target) {
            runpid = true;
            pidtarget = target;
        }
        void tare() {
            motor.tare_position();
        }
        void set_brake_mode(pros::motor_brake_mode_e_t mode) {
            motor.set_brake_mode(mode);
        }
        int get_temp() {
            return motor.get_temperature();
        }
};