#include "main.h"

void goalrush(meia::ChassisController* drive) {
    // std::cout << "autotime" << std::endl;
    // clamp.deactivate();
    // shtick.deactivate();
    // std::cout << meia::p::debting_go(drive, 21.5, 40.0, 120.0) << std::endl;
    // std::cout << meia::p::debting_go(drive, -21.5, 60.0, 90.0) << std::endl;
    // if (std::abs(drive->get_error().first) > 4) {
    //     meia::p::profile(drive, [](int t, int tt) -> std::pair<double, double> {
    //         return {-0.1, -0.1};
    //     }, 15000);
    // }
    // std::cout << "debt: " << meia::p::debting_go(drive, -24, 47, 300) << std::endl;
    // meia::p::turn(drive, 90.0, 30.0, 80.0, 14.5);
    // std::cout << "debt2: " << meia::p::debting_go(drive, 15, 47, 300) << std::endl;
    // std::cout << meia::p::turn(drive, 360, 20, 120, 14.1) << std::endl;
    meia::p::turn(drive, -360 * 3, 120, 240, 14.1);
}



