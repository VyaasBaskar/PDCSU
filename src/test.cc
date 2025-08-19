#include <chrono>
#include <iostream>

#include "control/icnor.h"

using namespace pdcsu::control;
using namespace pdcsu::units;
using namespace pdcsu::util;

int main() {
  DefBLDC def_bldc(105_u_A, 1.8_u_A, 2.5_u_Nm, 5676_u_rpm);
  DefLinearSys def_sys(def_bldc, 1, 214.85_u_rot / 262.5_u_in, 0.0_u_mps2,
      1_u_kg, 5_u_N, 0.5_u_N / 5676_u_rpm, 10_u_ms);

  std::cout << def_sys.inertia.value() << std::endl;

  ICNOR icnor(def_bldc, def_sys, 3000_u_rpm);

  std::cout << "v_max: " << icnor.getZ() << " rad/s\n";

  radian_t x = 45_u_in * def_sys.gear_ratio;

  std::cout << x.value() << " rad\n";

  icnor.setTarget(x, 0_u_radps);
  icnor.setState(0_u_rad, 0_u_radps);

  auto start = std::chrono::high_resolution_clock::now();
  std::tuple<double, double, double, double, double> result = icnor.optimize();
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed = end - start;
  std::cout << "Optimization took " << elapsed.count() << " ms\n";

  std::cout << std::get<0>(result) << ", " << std::get<1>(result) << ", "
            << std::get<2>(result) << ", " << std::get<3>(result) << ", "
            << std::get<4>(result) << std::endl;
  return 0;
}