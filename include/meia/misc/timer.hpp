#include "main.h"
namespace meia {
    class Timer {
    private:
        int start_time = pros::millis();
        int held_time = pros::millis();
        bool paused = pros::millis();
        pros::Mutex mutex;
    public:
        // resets and begins the timer 
        void start();
        // allows the timer to resume going with it's value when it was last paused
        void resume();
        // pauses the timer but maintains it's value
        void pause();
        // resets the value of the timer, so when it is next resumed it starts at zero
        void reset();
        // gets the current value of the timer
        int get_time();
    };
}