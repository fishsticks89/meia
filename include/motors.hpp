#include "main.h"
extern meia::Pid drive_pid;
extern pros::Motor take;
extern Motor lift;
extern Imu imu;

void setupMotors();
void awaitCalibrate();

extern Pneumatic clamp;
extern Pneumatic mogo;
extern Pneumatic shtick;
extern Pneumatic hpost;

extern meia::Console console;