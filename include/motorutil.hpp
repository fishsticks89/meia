#include "main.h"
class Motor {
    private:
        static void task_fn(void* params) {
            auto fn = static_cast<std::function<void()>*>(params);
            (*fn)();
        }
        pros::Task task;
        std::function<void()> motor_loop = [&]() {
            double prev_error = 0;
            while (true) {
                if (runpid) {
                    const double error = ((!reversed) ? pidtarget : -pidtarget) - (motor.get_position() / fac);
                    if (std::abs(error) > 2) {
                        motor.move_voltage((error * p.p + (prev_error - error) * p.d));
                    } else {
                        motor.move_voltage(0);
                    }
                    prev_error = error;
                }
                pros::delay(5);
            }
        };
    public:
        pros::Motor motor;
        static double get_motor_fac(int motor_ratio, double gear_ratio) {
            return ((50.0 * motor_ratio) * gear_ratio) / 360.0;
        }
        bool reversed;
        meia::Pid p;
        double fac = 1;
        bool runpid = false;
        double pidtarget = 0;
        Motor(int_fast16_t port, bool reversed, meia::Pid correct = meia::Pid (10000, 0, 0))
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
        void set_p(meia::Pid oink) {
            p = oink;
        }
};