#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include "control/icnor.h"
#include "simulation/simbldc.h"

using namespace pdcsu::control;
using namespace pdcsu::units;
using namespace pdcsu::util;
using namespace pdcsu::simulation;

int main() {
  DefBLDC def_bldc(105_u_A, 1.8_u_A, 2.5_u_Nm, 5676_u_rpm);
  DefLinearSys def_sys(def_bldc, 1, 214.85_u_rot / 262.5_u_in, 0.0_u_mps2,
      5_u_kg, 5_u_N, 0.5_u_N / 5676_u_rpm, 10_u_ms);

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

  SimBLDC simBldc = SimBLDC(def_sys);

  std::vector<double> projected_output{};

  std::ofstream data_file("sim_data.csv");
  data_file << "step,pos,vel,output\n";

  for (int i = 0; i < 160; i++) {
    if (i % 3 == 0) {
      icnor.setTarget(45_u_in * def_sys.gear_ratio, 0_u_radps);
      icnor.setState(simBldc.getPosition(), simBldc.getVelocity());
      result = icnor.optimize();
      std::cout << std::get<0>(result) << ", " << std::get<1>(result) << ", "
                << std::get<2>(result) << ", " << std::get<3>(result) << ", "
                << std::get<4>(result) << std::endl;
      projected_output = icnor.getProjectedOutput(3);
    }
    double output = (projected_output[i % 3]) /
                    radps_t(def_sys.def_bldc.free_speed).value();
    simBldc.SetCurrentLimit(105_u_A);
    simBldc.setControlTarget(output);
    simBldc.Tick(10_u_ms);
    double pos = simBldc.getPosition().value();
    double vel = simBldc.getVelocity().value();
    data_file << i << "," << pos << "," << vel << "," << output << "\n";
    std::cout << "Step " << i << ": pos=" << pos << " rad, vel=" << vel
              << " rad/s, output=" << output << std::endl;
  }
  data_file.close();
  std::cout << "Final position: " << simBldc.getPosition().value() << " rad\n";
  std::cout << "Simulation data written to sim_data.csv\n";
  return 0;
}