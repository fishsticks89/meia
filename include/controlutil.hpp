#include "main.h"
using void_fn = std::function<void()>;
using bool_fn = std::function<void(bool)>;
struct directional_fn {
    void_fn first_pressing;
    void_fn second_pressing;
    void_fn not_pressing;
    directional_fn(void_fn first_pressing, void_fn second_pressing, void_fn not_pressing): first_pressing(first_pressing), second_pressing(second_pressing), not_pressing(not_pressing) {}
};

class Button {
    private:
        pros::Controller* con;
        const pros::controller_digital_e_t button;
        bool wasPressed = false;

    public:
        Button(pros::Controller* con, pros::controller_digital_e_t button)
            : con(con), button(button) {
        }
        /**
         * If the button is being pressed and was not previously, returns true
         */
        bool operator()() {
            bool returnval = false;
            bool ispressing = con->get_digital(button);
            if (ispressing && !wasPressed) {
                returnval = true;
            }
            wasPressed = ispressing;
            return returnval;
        }
};
class Toggleable {
    private:
        Button pressing;
        bool toggle = false;

    public:
        Toggleable(pros::Controller* con, pros::controller_digital_e_t button)
            : pressing(con, button) {
        }
        /**
         * If the button is being pressed and was not previously, runs the lambda
         */
        bool operator()() {
            bool ispressing = pressing();
            if (ispressing) {
                toggle = !toggle;
            }
            return ispressing;
        }
        bool getState() {
            return toggle;
        }
};
class Directional {
    private:
        pros::Controller* con;
        const std::pair<pros::controller_digital_e_t, pros::controller_digital_e_t> buttons;
        bool whichwasrunning = true; // true if first
    public:
        Directional(pros::Controller* con, pros::controller_digital_e_t button1, pros::controller_digital_e_t button2)
            : con(con), buttons({button1, button2}) {
        }
        /**
         * If the button is being pressed and was not previously, returns true
         */
        std::pair<bool, bool> operator()() {
            bool first_pressing = con->get_digital(buttons.first);
            bool second_pressing = con->get_digital(buttons.second);
            if (first_pressing && second_pressing) {
                first_pressing = !whichwasrunning;
                second_pressing = whichwasrunning;
            } else {
                if (first_pressing && !second_pressing) {
                    whichwasrunning = true;
                }
                if (second_pressing && !first_pressing) {
                    whichwasrunning = false;
                }
            }
            return {first_pressing, second_pressing};
        }
};
class ControlScheme {
    private:
    pros::Controller* con;
    std::vector<std::pair<Button*, void_fn>> buttons = {};
    std::vector<std::pair<Toggleable*, bool_fn>> toggles = {};
    std::vector<std::pair<Directional*, directional_fn>> directionals = {};
    public:
    ControlScheme(pros::Controller* con)
        : con(con) {
    }
    ~ControlScheme() {
        for (auto button : buttons) {
            delete(button.first);
        }
        for (auto button : toggles) {
            delete(button.first);
        }
        for (auto button : directionals) {
            delete(button.first);
        }
    }
    void addButton(pros::controller_digital_e_t button, void_fn function) {
        buttons.push_back({new Button(con, button), function});
    }
    void addToggleable(pros::controller_digital_e_t button, bool_fn function) {
        toggles.push_back({new Toggleable(con, button), function});
    }
    void addDirectional(std::pair<pros::controller_digital_e_t, pros::controller_digital_e_t> buttons, void_fn first_pressing, void_fn second_pressing, void_fn not_pressing) {
        directionals.push_back({new Directional(con, buttons.first, buttons.second), directional_fn(first_pressing, second_pressing, not_pressing)});
    }
    void operator()() {
        for (auto button : buttons) {
            if ((*button.first)()) {
                button.second();
            }
        }
        for (auto toggle : toggles) {
            if ((*toggle.first)()) {
                toggle.second(toggle.first->getState());
            }
        }
        for (auto directional : directionals) {
            auto pressing = (*directional.first)();

            if (pressing.first) {
                directional.second.first_pressing();
            }
            if (pressing.second) {
                directional.second.second_pressing();
            }
            if (!pressing.first && !pressing.second) {
                directional.second.not_pressing();
            }
        }
    }
};