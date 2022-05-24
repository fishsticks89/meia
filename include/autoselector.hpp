#include "main.h"

struct Auton {
        std::string name;
        std::function<void(meia::ChassisController*)> func;

    public:
        Auton(std::string name, std::function<void(meia::ChassisController*)> func)
            : name(name), func(func) {
        }
};

class Autoselector {
    private:
        std::vector<Auton> autons;
        int active = 0;

    public:
        Autoselector(std::vector<Auton> autons)
            : autons(autons) {
        }
        Autoselector(std::vector<Auton> autons, int start_auton_index)
            : autons(autons), active(start_auton_index) {
        }
        Auton switch_left() {
            active -= 1;
            if (active < 0) {
                active = autons.size() - 1;
            }
            return autons[active];
        }
        Auton switch_right() {
            active += 1;
            if (active >= autons.size()) {
                active = 0;
            }
            return autons[active];
        }
        Auton get_active() {
            return autons[active];
        }
};

class ConsoleSelector {
    private:
        Autoselector autons;
        meia::Console* console;

    public:
        ConsoleSelector(Autoselector autons, meia::Console* console)
            : autons(autons), console(console) {
        }

        void log() {
            console->log(autons.get_active().name);
        }

        void switch_left() {
            autons.switch_left();
            log();
        }

        void switch_right() {
            autons.switch_right();
            log();
        }

        void run(meia::ChassisController* drive) {
            autons.get_active().func(drive);
        }
};