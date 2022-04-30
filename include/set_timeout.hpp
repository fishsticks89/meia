#include "main.h"

struct AsyncJob {
    std::function<void()> function;
    int time;
    AsyncJob(int time, std::function<void()> function) : time(time), function(function) {}
};

class Async {
    private:
        pros::Task task;
        std::vector<AsyncJob> jobs = {};
        static void task_fn(void* p) {
            auto jobs = static_cast<std::vector<AsyncJob>*>(p);
            while (true) {
                if (jobs->size() > 0) {
                    if (jobs->at(0).time <= pros::millis()) {
                        // std::cout << "lol3 " << jobs->at(0).time << " realtime: " << pros::millis() << std::endl;
                        jobs->at(0).function();
                        jobs->erase(jobs->begin());
                    }
                }
                pros::delay(50);
            }
        }
    public: 
        Async() : task(task_fn, &jobs,"bob") {}
        void add_job(AsyncJob job) {
            jobs.push_back(job);
        }
        void timeout(std::function<void()> function, int time) {
            add_job(AsyncJob(time + pros::millis(), function));
        }
};