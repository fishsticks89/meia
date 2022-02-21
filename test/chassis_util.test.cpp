#include "../src/meia/drive/chassis_util.cpp"
#include <iostream>
#include <string>
void ASSERT (double a, double b) {
  if (a != b) {
    std::cout << std::endl << "f***, " + std::to_string(a) + " != " + std::to_string(b) << std::endl;
    throw "\nf***, " + std::to_string(a) + " != " + std::to_string(b) + "\n";
  }
}

int main() {
  ASSERT(meia::get_curve_distance(1, 4, 0), 6.0);
  ASSERT(meia::get_curve_iterations(1, 4, 0), 3);
  std::cout << "all good!" << std::flush;
}