#include "main.h"
namespace meia {
    class Pid {
        public:
            double p;
            double i;
            double d;
            Pid(double p, double i, double d)
                : p(p), i(i), d(d){};
    };
    /**
     * A Curve for an end of a motion profiled motion
     *  \param acceleration
     *      The acceleration of the curve measured in (deg or units)/second^2.
     *      Play around with this to find your robot's max acceleleration (make sure drive.getBehind() is never over 0)
     *  \param endpoint_speed
     *      The speed at the endpoint of the curve
     *      If you do not understand this, leave at 0
     *  \param antijerk_percent
     *      On this graph, the x axis is time and the y axis is velocity: https://www.desmos.com/calculator/kqs7xawjnq
     *      Antijerk percent, or the 'f' slider, is the % the 'corners' of the motion profile are rounded off to reduce jerk
     *      If you do not understand this, leave at 0
     */
    class Curve {
        public:
            Curve(double acceleration = 0, double endpoint_speed = 0, double antijerk_percent = 0)
                : endpoint_speed(endpoint_speed), acceleration(acceleration), antijerk_percent(antijerk_percent){};
            double endpoint_speed;
            double acceleration;
            double antijerk_percent;
    };
    class MovementTelemetry {
        private:
            double total_move;
            double* amount_completed;
            pros::Mutex* mutex;
            int id;
            int* current_id;

        public:
            MovementTelemetry(double total_move = 0, double* amount_completed = nullptr, int* current_id = nullptr, pros::Mutex* completed = nullptr, int id = -1)
                : total_move(total_move), amount_completed(amount_completed), mutex(mutex), id(id), current_id(current_id) {};
            void wait_until(double val);
            
    };
} // namespace meia