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

void arcturn(meia::ChassisController* drive, bool left, double amount, double speed = 180, double acc = 720) {
    bool ispos = amount > 0;
    amount = std::abs(amount);
    if ((speed / acc) * speed > amount) {
        speed = (std::sqrt(amount * acc) - 0.05);
    }
    meia::p::arcturn(drive, left, amount * (((ispos > 0) - 0.5) * 2), speed, acc, drive_width);
}

double go(meia::ChassisController* drive, double amount, double speed = 45, double acc = 130) {
    std::cout << "max_speed(in/sec_<from go>: " << std::sqrt(std::abs(amount * acc * 2)) - 0.05 << std::endl;
    if ((speed / acc) * (speed / 2) > amount) {
        speed = std::sqrt(std::abs(amount * acc * 2)) - 0.05;
    }
    return meia::p::debting_go(drive, amount, speed, acc);
}

double igo(meia::ChassisController* drive, double heading_target, double amount, double speed = 45, double acc = 130, meia::Pid heading_pd = meia::Pid(4, 0, 0)) {
    heading_target = imu.wrap(heading_target);
    std::cout << std::sqrt(amount * acc * 2) - 0.05 << std::endl;
    if ((speed / acc) * (speed / 2) > std::abs(amount)) {
        speed = std::sqrt(std::abs(amount * acc * 2)) - 0.05;
    }
    std::cout << "speed: " << speed << std::endl;
    return meia::p::debting_go_heading_corr(drive, &imu, amount, speed, acc, heading_pd, heading_target, drive_width);
}

const auto t_pd = meia::Pid(8, 0, 0);

void iturnb(meia::ChassisController* drive, bool dir, double target, double speed = 360, double acc = 720, meia::Pid drive_pd = t_pd) {
    target = imu.wrap(target);
    double amount = imu.get_dist(dir, target);
    if ((speed / acc) * speed > std::abs(amount)) {
        speed = (std::sqrt(std::abs(amount * acc)) - 0.05);
    }
    std::cout << "iturnbamount: " << amount << std::endl;
    std::cout << "lolololo: " << (speed / acc) * speed << std::endl;
    meia::p::imuturn(drive, dir, &imu, target, speed, acc, drive_width, drive_pd);
}

void iturn(meia::ChassisController* drive, double target, double speed = 360, double acc = 720, meia::Pid drive_pd = t_pd) {
    bool dir = imu.get_dir(target);
    iturnb(drive, dir, target, speed, acc, drive_pd);
}

void goalrush(meia::ChassisController* drive) {
    int x = pros::millis();
    shtick.activate();
    std::cout << meia::p::debting_go(drive, 35, 70, 240) << std::endl;
    std::cout << meia::p::debting_go(drive, -35, 70, 240) << std::endl;
    std::cout << "time: " << pros::millis() - x << std::endl;
    if (pros::millis() - x > 2700) {
        while (true) {
            std::cout << meia::p::debting_go(drive, -35, 70, 240) << std::endl;
        }
    }
}

void rightrushrings(meia::ChassisController* drive) {
    double turnd = 0;
    lift.set_voltage(-7000);
    clamp.deactivate();
    mogo.deactivate();
    // shtick nmogo
    int start_time = pros::millis();
    std::cout << igo(drive, turnd, 43, 45, 150) << std::endl;
    clamp.activate();
    pros::delay(150);
    std::cout << igo(drive, turnd, -26, 45, 90) << std::endl;
    std::cout << "time: " << pros::millis() - start_time << std::endl;
    if (pros::millis() - start_time > 3000) {
        while (true) {
            std::cout << meia::p::debting_go(drive, -35, 70, 240) << std::endl;
        }
    }
    lift.set_target(30);
    // get rmogo in
    iturn(drive, -90, 140, 300);
    lift.set_voltage(-4000);
    meia::p::debting_go(drive, -14, 20, 20);
    pros::delay(150);
    mogo.activate();
    pros::delay(300);
    lift.set_target(40);

    // meia::p::debting_go(drive, 2, 30, 6000);
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
    go(drive, 35);
    drive->set_pid_constants(350, 0, 150);
    lift.set_voltage(-4000);
    go(drive, -23);
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
    lift.set_target(80);
    go(drive, -28, 30, 40);
    turn(drive, imu.get_dist(true, -92), 90, 220);
    lift.set_voltage(-12000);
    pros::delay(600);
    // get mogo
    go(drive, -13, 7, 30);
    pros::delay(200);
    mogo.activate();
    pros::delay(100);
    lift.set_target(95);
    // go get rings
    arcturn(drive, true, 125);
    lift.set_p(meia::Pid(500, 0, 0));
    lift.set_target(60);
    take.move_voltage(12000);
    go(drive, 24);
    arcturn(drive, true, imu.get_dist(90));
    go(drive, 27, 12);
    mogo.deactivate();
    go(drive, -10);
    go(drive, 5);
}

void leftrushclamp(meia::ChassisController* drive) {
    double turnd = 0;
    lift.set_voltage(-8000);
    clamp.deactivate();
    std::cout << igo(drive, turnd, 50, 45, 150, meia::Pid(2, 0, 0)) << std::endl;
    clamp.activate();
    lift.set_voltage(0);
    pros::delay(100);
    std::cout << igo(drive, turnd, -50, 45, 240, meia::Pid(2, 0, 0)) << std::endl;
}

void trushclamp(meia::ChassisController* drive) {
    lift.set_voltage(-2000);
    clamp.deactivate();
    int x = pros::millis();
    std::cout << igo(drive, 0, 54.5, 60, 240) << std::endl;
    std::cout << "time: " << pros::millis() - x << std::endl;
    if (pros::millis() - x > 3200) {
        while (true) {
            std::cout << meia::p::debting_go(drive, -35, 70, 240) << std::endl;
        }
    }
    clamp.activate();
    shtick.activate();
    pros::delay(100);
    std::cout << igo(drive, 0, -48.5, 60, 240) << std::endl;
    pros::delay(50);
    double turnd = 28;
    iturn(drive, turnd, 360, 1000);
    std::cout << igo(drive, turnd, 34, 70, 240) << std::endl;
    std::cout << igo(drive, turnd, -34, 70, 240) << std::endl;
}

void rthenleft(meia::ChassisController* drive) {
    double turnd = 0;
    // shtick nmogo
    shtick.activate();
    int start_time = pros::millis();
    std::cout << igo(drive, turnd, 34, 60, 150) << std::endl;
    std::cout << igo(drive, turnd, -34, 60, 90) << std::endl;
    std::cout << "time: " << pros::millis() - start_time << std::endl;
    if (pros::millis() - start_time > 2700) {
        while (true) {
            std::cout << meia::p::debting_go(drive, -35, 70, 240) << std::endl;
        }
    }
    shtick.deactivate();
    pros::delay(300);
    turnd = -26;
    iturn(drive, turnd);
    clamp.deactivate();
    lift.set_voltage(-8000);
    igo(drive, turnd, 50);
    clamp.activate();
    pros::delay(200);
    lift.set_voltage(0);
    igo(drive, turnd, -43);
}

void soloawp(meia::ChassisController* drive) {
    double turnd = 0;
    mogo.deactivate();
    drive_width = 14.1;
    lift.set_p(meia::Pid(500.0, 0, 0));
    lift.set_target(27.5);
    pros::delay(400);
    go(drive, 2);
    pros::delay(200);
    clamp.deactivate();
    lift.set_p(lift_pid);
    pros::delay(500);
    go(drive, -5, 45, 30);
    lift.set_voltage(-9000);
    turn(drive, 90, 200);
    // go out
    go(drive, -15.5);
    // turn before length
    turnd = -4;
    iturn(drive, turnd, 220, 500);
    std::cout << "imudist: " << imu.get_dist(true, 0) << std::endl;
    igo(drive, turnd, 70);
    // get amogo
    turn(drive, imu.get_dist(false, 180), 220, 300);
    go(drive, -26, 30, 60);
    pros::delay(200);
    go(drive, -6, 5, 14);
    pros::delay(100);
    mogo.activate();
    pros::delay(200);
    take.move_voltage(12000);
    go(drive, 22, 46, 37);
    // get big nmogo
    turn(drive, imu.get_dist(false, -180 + 50));
    clamp.deactivate();
    std::cout << meia::p::debting_go(drive, 33.7, 60, 160) << std::endl;
    clamp.activate();
    pros::delay(200);
    mogo.deactivate();
    std::cout << meia::p::debting_go(drive, -45, 60, 60) << std::endl;
}

void sKILLS(meia::ChassisController* drive) {
    drive_width = 14.41;
    { // get first mogo
        mogo.deactivate();
        pros::delay(2000);
        go(drive, -3, 20, 8);
        mogo.activate();
        pros::delay(100);
    }
    { // get neutral goal and get nmogo and go to platform
        double initialturnd = 20;
        arcturn(drive, true, initialturnd, 160, 400);
        pros::delay(100);
        igo(drive, initialturnd, 4);
        double turnd = 108;
        arcturn(drive, true, turnd - initialturnd, 160, 400);
        pros::delay(200);
        clamp.deactivate();
        lift.set_voltage(-4000);

        igo(drive, turnd, 39);
        clamp.activate();
        lift.set_target(20);

        turnd = 135;
        iturn(drive, turnd, 360, 500);
        take.move_voltage(12000);
        lift.set_target(95);
        igo(drive, turnd, 48, 45, 80);
        lift.set_target(95); // for some reason if first doesn't work
        take.move_voltage(0);
    }
    { // platform nmogo
        double turnd = 90;
        iturn(drive, turnd, 90, 400);
        pros::delay(200);
        mogo.deactivate(); // leave mogo to be picked up later
        pros::delay(400);
        igo(drive, turnd, 4, 100, 170); // try to get off rear mogo
        pros::delay(200);
        igo(drive, turnd, 15, 30, 70);
        lift.set_target(50);
        pros::delay(700);
        clamp.deactivate();
        pros::delay(100);
        lift.set_target(95);
        pros::delay(450);
        igo(drive, turnd, -5, 30);
    }
    { // fclamp rmogo
        const double turnd = -90;
        async.timeout([]() { lift.set_voltage(-12000); }, 500);
        iturnb(drive, true, turnd, 360, 200);
        igo(drive, turnd, 10, 30);
        clamp.activate();
    }
    { // platform rmogo & push back bnmogo
        double turnd = 105;
        lift.set_target(85);
        iturn(drive, turnd, 360, 200);
        take.move_voltage(0);
        igo(drive, turnd, 17, 30, 30);

        lift.set_target(70);
        pros::delay(200);
        clamp.deactivate();
        pros::delay(500);
        // igo(drive, turnd, -2, 30, 60); // center with nmogo

        turnd = -90;
        lift.set_target(85);
        async.timeout([]() { lift.set_target(0); }, 350);
        iturn(drive, turnd, 360, 500);
        lift.set_target(-3);

        // push back bnmogo
        drive->set_pid_constants(300, 0, 180);
        take.move_voltage(12000);
        igo(drive, turnd, 59, 45, 100);
        drive->set_pid_constants(drive_pid);
        take.move_voltage(-12000);
        clamp.deactivate();
        async.timeout([]() { lift.set_voltage(2000); }, 200);
        igo(drive, turnd, -6);
    }
    { // get ramogo & center with rnmogo
        double turnd = 19;
        iturnb(drive, false, turnd, 360, 400);
        lift.set_target(0); // get ready to release mogo
        pros::delay(200);
        lift.set_voltage(-3000); // lower lift
        igo(drive, turnd, -49, 30, 40);
        pros::delay(100);
        igo(drive, turnd, -6, 60, 12);
        pros::delay(70);
        mogo.activate();
        pros::delay(200);

        turnd = 0;

        // center with nmogo
        iturn(drive, turnd);
        igo(drive, turnd, 15.5);
    }
    { // get rnmogo
        double turnd = 90;
        iturn(drive, turnd, 360, 400);
        igo(drive, turnd, 23);
        clamp.activate();
        lift.set_target(35);
        igo(drive, turnd, 34, 45, 40);
        turnd = 0;
        take.move_voltage(12000);
        iturn(drive, turnd, 360, 500);
        lift.set_target(70);

        // center with plat
        lift.set_target(95);
        take.move_voltage(-12000);
        igo(drive, turnd, 50, 45, 50, meia::Pid(6, 0, 0));
        pros::delay(200);
        igo(drive, turnd, -10, 45, 50);
        pros::delay(200);
    }
    { // platform rnmogo
        double turnd = 90;
        iturn(drive, turnd, 360, 500);
        mogo.deactivate();
        take.move_voltage(0);
        lift.set_target(80);
        pros::delay(500);
        igo(drive, turnd, 5, 45, 130);
        pros::delay(130);
        igo(drive, turnd, 12, 30, 30);
        clamp.deactivate();
        lift.set_target(70);
        pros::delay(500);
        igo(drive, turnd, -5, 30, 30);
    }
    { // get rear mogo
        const double turnd = -90;
        async.timeout([]() { lift.set_target(0); }, 500);
        iturnb(drive, true, turnd, 360, 300);
        pros::delay(200);
        igo(drive, turnd, 10, 30);
        clamp.activate();
        lift.set_target(80);
    }
    { // platform rmogo
        double turnd = 80;
        iturn(drive, turnd, 360, 300);
        lift.set_target(80);
        igo(drive, turnd, 19, 30, 30);
        clamp.deactivate();
        lift.set_target(75);
        pros::delay(500);
        igo(drive, turnd, -8, 30, 60); // center with nmogo
    }
    { // get right far alliance mogo
        double turnd = 0;
        iturn(drive, turnd, 360, 500);
        lift.set_target(0);
        igo(drive, turnd, -51);
        // turn to blue mogo
        turnd = 53;
        lift.set_voltage(-6000);
        iturn(drive, turnd);
        igo(drive, turnd, 19, 45, 90);
        clamp.activate();
        pros::delay(200);
        lift.set_target(10);
        igo(drive, turnd, -19);
    }
    { // get left far alliance mogo
        double turnd = 0;
        mogo.deactivate();
        iturn(drive, turnd, 360, 500);
        igo(drive, turnd, 70);

        // turn and get mogo
        turnd = 180;
        iturnb(drive, true, turnd);
        igo(drive, turnd, -21, 45, 100);
        pros::delay(200);
        igo(drive, turnd, -8, 45, 20);
        mogo.activate();
        pros::delay(200);
    }
    { // stuff
        double turnd = 45;
        iturn(drive, -40);
    }
}

void stupod(meia::ChassisController* drive) {
    mogo.deactivate();
    pros::delay(1000);
    double turnd = 0;
    go(drive, -15, 1, 10);
    pros::delay(1000);
    mogo.activate();
    pros::delay(1000);
    take.move_voltage(12000);

    pros::delay(1000);
    go(drive, 15, 3);
    pros::delay(6000);
    mogo.deactivate();
    go(drive, 10);
    go(drive, -7);
    go(drive, 10);
}

ConsoleSelector autons(
    Autoselector(
        {
            Auton("SOLOAWP SHKOSH", soloawp),                       // 0
            Auton("shtick rush both sides", goalrush),              // 1
            Auton("left shtick rush w/rings", lrushrings),          // 2
            Auton("left clamp", leftrushclamp),                     // 3
            Auton("right clamp rush w/rings", rightrushrings),      // 4
            Auton("right shtick rush nmogo and right", trushclamp), // 5
            Auton("get right neutral then big neutral", rthenleft), // 6
            Auton("skills", sKILLS),                                // 7
            Auton("stupod", stupod),                                // 8
        },
        3), // defaulton
    &console);