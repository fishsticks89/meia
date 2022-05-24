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
    class Acceleration {
        const double start_vel;
        const double acc;
        Acceleration(double start_vel, double acc) : start_vel(start_vel), acc(acc) {}; 
    };
    /**
     * A class to represent a turn acceleration
     * \param start_vel
     *      The rotational velocity to accelerate from (deg/sec)
     * \param acc
     *      The acceleration to reach the peak velocity with (deg/sec^2)
     */
    class RotationalAcceleration : Acceleration {};
    /**
     * A class to represent a directional acceleration
     * \param start_vel
     *      The linear velocity to accelerate from (in/sec)
     * \param acc
     *      The acceleration to reach the peak velocity with (in/sec^2)
     */
    class LinearAcceleration : Acceleration {};
    // class MovementTelemetry {
    //     private:
    //         double total_move = 1;
    //         double* amount_completed;
    //         pros::Mutex* mutex;
    //         int id = -1;
    //         int* current_id;

    //     public:
    //         MovementTelemetry(double total_move, double* amount_completed, int* current_id, pros::Mutex* mutex, int id)
    //             : total_move(total_move), amount_completed(amount_completed), mutex(mutex), id(id), current_id(current_id) {};
    //         MovementTelemetry() {};
    //         void wait_until(double val);
    // };
} // namespace meia