#include "main.h"

double drive_width = 14.1;

void turn(meia::ChassisController* drive, double amount, double speed = 360, double acc = 720) {
    bool ispos = amount > 0;
    amount = std::abs(amount);
    if ((speed / acc) * speed > amount) {
        speed = (std::sqrt(std::abs(amount * acc)) - 0.05);
    }
    meia::p::turn(drive, amount * (((ispos > 0) - 0.5) * 2), speed, acc, drive_width);
}

void arcturn(meia::ChassisController* drive, bool left, double amount, double speed = 180, double acc = 360) {
    bool ispos = amount > 0;
    amount = std::abs(amount);
    if ((speed / acc) * speed > amount) {
        speed = (std::sqrt(amount * acc) - 0.05);
    }
    meia::p::arcturn(drive, left, amount * (((ispos > 0) - 0.5) * 2), speed, acc, drive_width);
}

void go(meia::ChassisController* drive, double amount, double speed = 45, double acc = 130) {
    std::cout << std::sqrt(amount * acc * 2) - 0.05 << std::endl;
    if ((speed / acc) * (speed / 2) > amount) {
        speed = std::sqrt(std::abs(amount * acc * 2)) - 0.05;
    }
    meia::p::debting_go(drive, amount, speed, acc);
}

void igo(meia::ChassisController* drive, double heading_target, double amount, double speed = 45, double acc = 130, meia::Pid heading_pd = meia::Pid(0.2, 0, 0)) {
    heading_target = imu.wrap(heading_target);
    std::cout << std::sqrt(amount * acc * 2) - 0.05 << std::endl;
    if ((speed / acc) * (speed / 2) > amount) {
        speed = std::sqrt(std::abs(amount * acc * 2)) - 0.05;
    }
    meia::p::debting_go_heading_corr(drive, &imu, amount, speed, acc, heading_pd, heading_target, drive_width);
}

const auto t_pd = meia::Pid(8, 0, 0);

void iturnb(meia::ChassisController* drive, bool dir, double target, double speed = 360, double acc = 720, meia::Pid drive_pd = t_pd) {
    target = imu.wrap(target);
    double amount = imu.get_dist(dir, target);
    if ((speed / acc) * speed > std::abs(amount)) {
        speed = (std::sqrt(std::abs(amount * acc)) - 0.05);
    }
    std::cout  << "iturnbamount: " << amount << std::endl;
    std::cout  << "lolololo: " << (speed / acc) * speed << std::endl;
    meia::p::imuturn(drive, dir, &imu, target, speed, acc, drive_width, drive_pd);
}
void iturn(meia::ChassisController* drive, double target, double speed = 360, double acc = 720, meia::Pid drive_pd = t_pd) {
    bool dir = imu.get_dir(target);
    iturnb(drive, dir, target, speed, acc, drive_pd);
}

void goalrush(meia::ChassisController* drive) {
    shtick.activate();
    std::cout << meia::p::debting_go(drive, 34, 70, 240) << std::endl;
    std::cout << meia::p::debting_go(drive, -34, 70, 240) << std::endl;
}

void rightrushrings(meia::ChassisController* drive) {
    mogo.deactivate();
    shtick.activate();
    // shtick nmogo
    std::cout << meia::p::debting_go(drive, 34, 60, 150) << std::endl;
    std::cout << meia::p::debting_go(drive, -34, 60, 90) << std::endl;
    drive->set_pid_constants(drive_pid);
    lift.set_voltage(-4000);
    clamp.deactivate();
    shtick.deactivate();
    pros::delay(200);
    // get nmogo in clamp
    meia::p::turn(drive, 10, 30, 100, drive_width);
    meia::p::debting_go(drive, 19, 50, 360);
    clamp.activate();
    pros::delay(100);
    lift.set_target(20);
    pros::delay(400);
    // get rmogo in
    turn(drive, imu.get_dist(true, -90), 140, 300);
    lift.set_voltage(-4000);
    meia::p::debting_go(drive, -17.7, 20, 900);
    mogo.activate();
    pros::delay(300);
    lift.set_target(40);
    meia::p::debting_go(drive, 5, 30, 6000);
    meia::p::turn(drive, 91, 90, 360, drive_width);
    lift.set_target(50);
    take.move_voltage(12000);
    meia::p::debting_go(drive, 45, 13, 50);
    mogo.deactivate();
    meia::p::debting_go(drive, -60, 70, 180);
    drive->change_target({-4, 4});
}

void lrushrings(meia::ChassisController* drive) {
    mogo.deactivate();
    shtick.activate();
    go(drive, 34);
    drive->set_pid_constants(350, 0, 150);
    lift.set_voltage(-4000);
    go(drive, -22);
    drive->set_pid_constants(drive_pid);
    shtick.deactivate();
    clamp.deactivate();
    pros::delay(200);
    turn(drive, 12);
    go(drive, 10);
    clamp.activate();
    lift.set_target(10);
    pros::delay(200);
    turn(drive, 20);
    lift.set_target(120);
    go(drive, -28, 30, 40);
    turn(drive, imu.get_dist(true, -92), 90, 220);
    lift.set_voltage(-12000);
    pros::delay(230);
    // get mogo
    go(drive, -12.4, 7, 30);
    pros::delay(200);
    mogo.activate();
    pros::delay(100);
    lift.set_target(120);
    go(drive, 12);
    turn(drive, 20);
    // go get rings
    take.move_voltage(12000);
    go(drive, 7);
    turn(drive, 120);
    lift.set_target(40);
    go(drive, 35);
    drive->set_pid_constants(350, 90, 150);
    go(drive, 30, 10);
    drive->set_pid_constants(drive_pid);
    mogo.deactivate();
    go(drive, -35);
}

void rushclamp(meia::ChassisController* drive) {
    clamp.deactivate();
    std::cout << meia::p::debting_go(drive, 41, 70, 240) << std::endl;
    clamp.activate();
    std::cout << meia::p::debting_go(drive, -41, 70, 240) << std::endl;
}

void trushclamp(meia::ChassisController* drive) {
    lift.set_voltage(-2000);
    clamp.deactivate();
    std::cout << meia::p::debting_go(drive, 53, 60, 240) << std::endl;
    clamp.activate();
    shtick.activate();
    std::cout << meia::p::debting_go(drive, -48.5, 60, 240) << std::endl;
    pros::delay(50);
    turn(drive, 25, 360, 1000);
    std::cout << meia::p::debting_go(drive, 34, 70, 240) << std::endl;
    std::cout << meia::p::debting_go(drive, -34, 70, 240) << std::endl;
}

void soloawp(meia::ChassisController* drive) {
    mogo.deactivate();
    drive_width = 14.1;
    lift.set_target(35);
    pros::delay(600);
    go(drive, 4);
    clamp.deactivate();
    pros::delay(500);
    go(drive, -5, 45, 30);
    lift.set_voltage(-9000);
    turn(drive, 90, 200);
    // go out
    go(drive, -15.5, 20, 30);
    drive->set_pid_constants(300, 0, 90);
    pros::delay(100);
    drive->set_pid_constants(drive_pid);
    // turn before length
    turn(drive, imu.get_dist(true, -5), 220, 500);
    std::cout << "imudist: " << imu.get_dist(true, 0) << std::endl;
    go(drive, 70);
    // get amogo
    turn(drive, imu.get_dist(false, 180), 220, 500);
    go(drive, -32.5, 20, 60);
    pros::delay(100);
    mogo.activate();
    pros::delay(200);
    take.move_voltage(12000);
    go(drive, 22, 45, 30);
    // get big nmogo
    turn(drive, imu.get_dist(false, -180 + 50));
    clamp.deactivate();
    std::cout << meia::p::debting_go(drive, 34, 60, 160) << std::endl;
    clamp.activate();
    pros::delay(200);
    mogo.deactivate();
    std::cout << meia::p::debting_go(drive, -45, 60, 60) << std::endl;
}

void sKILLS(meia::ChassisController* drive) {
    drive_width = 14.41;
    { // get first mogo
        mogo.deactivate();
        pros::delay(700);
        pros::delay(200);
        go(drive, -3, 20, 8);
        mogo.activate();
    }
    { // get neutral goal
        const double turnd = 101;
        arcturn(drive, true, turnd, 100);
        clamp.deactivate();
        lift.set_voltage(-4000);

        igo(drive, turnd, 38.5);
        clamp.activate();
        lift.set_target(20);
        igo(drive, turnd, 40, 45, 80);
    }
    { // get rings
        const double turnd = 180;
        iturn(drive, turnd, 90, 400);
        lift.set_target(95);
        igo(drive, turnd, 20, 30);
        take.move_voltage(12000);
        igo(drive, turnd, 15, 10);
        igo(drive, turnd, -8);
    }
    { // platform nmogo
        double turnd = 90;
        iturn(drive, turnd, 90, 400);
        pros::delay(200);
        mogo.deactivate(); // leave mogo to be picked up later
        pros::delay(500);
        take.move_voltage(0);
        pros::delay(200);
        igo(drive, turnd, 4, 30, 50); // try to get off rear mogo
        igo(drive, turnd, 8, 30, 50);
        lift.set_target(50);
        pros::delay(700);
        clamp.deactivate();
        lift.set_target(95);
        pros::delay(800);
        igo(drive, turnd, -2, 30);
        mogo.activate();
    }
    { // fclamp rmogo
        const double turnd = -90;
        iturnb(drive, true, turnd, 360, 300);
        lift.set_voltage(-12000);
        pros::delay(900);
        igo(drive, turnd, 10, 30);
        clamp.activate();
        lift.set_target(20);
    }
    { // platform rmogo & push back bnmogo
        double turnd = 90;
        iturn(drive, turnd, 360, 100);
        lift.set_target(80);
        take.move_voltage(0);
        pros::delay(200);
        lift.set_target(70);
        pros::delay(700);
        igo(drive, turnd, 15, 30, 30);
        clamp.deactivate();
        lift.set_target(95);
        pros::delay(500);
        lift.set_target(4);
    
        // push back bnmogo
        igo(drive, turnd, -66);
        igo(drive, turnd, 2);
    }
    { // go get the ramogo
        double turnd = -150;
        iturn(drive, turnd);
        igo(drive, turnd, -66);
    }   
}

void auton(meia::ChassisController* drive) {
    sKILLS(drive);
    clamp.deactivate();
    mogo.deactivate();
}