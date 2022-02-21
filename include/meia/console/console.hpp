#include "main.h"

namespace meia {
    enum x_y_button_e {
        left, right, center, body
    };
    class Console {
        private:
            std::vector<std::pair<std::string, bool>> logs = {};
            static std::vector<std::pair<std::pair<int_fast16_t, int_fast16_t>, bool>> white();
            static std::vector<std::pair<std::pair<int_fast16_t, int_fast16_t>, bool>> blue();
            void logo();
            void render();
            void clear();
        public:
            Console() {};
            int init();
            int log(std::string);
            // void register_callback(pros::touch_event_cb_fn_t);
            static x_y_button_e to_button(int x, int y);
    };
}