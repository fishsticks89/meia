#include "main.h"
class Pneumatic {
    private:
        pros::ADIDigitalOut solenoid;
        bool reversed;

    public:
        Pneumatic(char port, bool reversed)
            : solenoid(port), reversed(reversed){};
        void set(bool activated) {
            solenoid.set_value(activated ^ reversed);
        }
        void activate() {
            set(true);
        }
        void deactivate() {
            set(false);
        }
};