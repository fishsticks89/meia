#include "main.h"
namespace meia {
    void logger() {
        std::string d = "glubr";
        printf(d.c_str());
    }
    pros::Task goobel(logger);
}