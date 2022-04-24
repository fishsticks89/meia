#include "main.h"
using void_fn = void (*)(void);
using bool_fn = void (*)(bool);

class Pneumatic {
    private:
        pros::ADIDigitalOut solenoid;
        bool reversed;

    public:
        Pneumatic(char port, bool reversed)
            : solenoid(port), reversed(reversed){};
        void set(bool activated) {
            solenoid.set_value((!reversed) ? activated : !activated);
        }
        void activate() {
            set(true);
        }
        void deactivate() {
            set(false);
        }
};
class DualPneumatic {
    private:
        std::pair<Pneumatic, Pneumatic> pneumatics;

    public:
        DualPneumatic(Pneumatic one, Pneumatic two)
            : pneumatics({one, two}) {
        }
        void activate() {
            pneumatics.first.activate();
            pneumatics.second.activate();
        }
        void deactivate() {
            pneumatics.first.deactivate();
            pneumatics.second.deactivate();
        }
        void set(bool activated) {
            pneumatics.first.set(activated);
            pneumatics.second.set(activated);
        }
};