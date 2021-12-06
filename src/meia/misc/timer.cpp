#include "main.h"
namespace meia {
    void Timer::start() {
        mutex.take(3000);
        start_time = pros::millis();
        held_time = 0;
        paused = false;
        mutex.give();
    }
    void Timer::resume() {
        mutex.take(3000);
        start_time = pros::millis() - held_time;
        held_time = 0;
        paused = false;
        mutex.give();
    }
    void Timer::pause() {
        mutex.take(3000);
        held_time = pros::millis() - start_time;
        paused = true;
        mutex.give();
    }
    void Timer::reset() {
        mutex.take(3000);
        start_time = pros::millis();
        held_time = 0;
        mutex.give();
    }
    int Timer::get_time() {
        mutex.take(3000);
        int returnval;
        if (paused)
            returnval = held_time;
        else
            returnval = pros::millis() - start_time;
        mutex.give();
        return returnval;
    }
}