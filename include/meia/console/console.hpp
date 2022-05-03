#include "main.h"

namespace meia {
    enum button_e {
        left, right, center, body
    };
    class Console {
        private:
            struct logs_struct {
                pros::Mutex* mutex;
                std::vector<std::pair<std::string, bool>>* logs;
                static void render(std::vector<std::pair<std::string, bool>>);
                static void clear();
                logs_struct(pros::Mutex* mutex, std::vector<std::pair<std::string, bool>>* logs) : mutex(mutex), logs(logs) {};
            };
            std::vector<std::pair<std::string, bool>> logs = {};
            static std::vector<std::pair<std::pair<int_fast16_t, int_fast16_t>, bool>> white();
            static std::vector<std::pair<std::pair<int_fast16_t, int_fast16_t>, bool>> red();
            void logo();
            static void task_fn(void*);
            pros::Mutex mutex;
            pros::Task task;
            logs_struct io;
        public:
            Console() : io(&mutex, &logs), task(task_fn, &io, 'c') {};
            int init();
            
            /**
             * Does not work for some reason
             */
            void clear();
            int log(std::string);
            // void register_callback(pros::touch_event_cb_fn_t);
            static button_e to_button(int x, int y);
    };
}