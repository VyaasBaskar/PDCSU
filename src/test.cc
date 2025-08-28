#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include "control/icnor.h"
#include "control/util.h"
#include "simulation/simbldc.h"

using namespace pdcsu::control;
using namespace pdcsu::units;
using namespace pdcsu::util;
using namespace pdcsu::simulation;

int main() {
  DefBLDC def_bldc(105_u_A, 1.8_u_A, 2.5_u_Nm, 5676_u_rpm);
  DefLinearSys def_sys(def_bldc, 1, 214.85_u_rot / 262.5_u_in, 0.0_u_mps2,
      5_u_kg, 40_u_N, 3_u_N / 5676_u_rpm, 20_u_ms);

  DefBLDC bldc2 = def_bldc;
  bldc2.stall_current = 40_u_A;
  bldc2.stall_torque = bldc2.stall_torque * 40_u_A / 105_u_A;

  std::cout << def_sys.inertia.value() << std::endl;

  ICNOR icnor(bldc2, def_sys, 5000_u_rpm);
  FFModel ffmodel(def_sys);
  SymmetricHysteresis hys(0.12_u_rad, 0.3_u_rad);

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

  for (int i = 0; i < 95; i++) {
    if (i % 1 == 0) {
      icnor.setTarget(45_u_in * def_sys.gear_ratio, 0_u_radps);
      icnor.setState(simBldc.getPosition(), simBldc.getVelocity());
      icnor.setState(simBldc.getPosition(), simBldc.getVelocity());
      auto opt_start = std::chrono::high_resolution_clock::now();
      result = icnor.optimize();
      auto opt_end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> opt_elapsed =
          opt_end - opt_start;
      std::cout << "Optimization took " << opt_elapsed.count() << " ms\n";
      std::cout << std::get<0>(result) << ", " << std::get<1>(result) << ", "
                << std::get<2>(result) << ", " << std::get<3>(result) << ", "
                << std::get<4>(result) << std::endl;
      projected_output = icnor.getProjectedOutput(5);
    }
    bool cut = hys.cut(45_u_in * def_sys.gear_ratio, simBldc.getPosition());
    double orig_output = ((projected_output[i % 1]) /
                          radps_t(def_sys.def_bldc.free_speed).value());
    orig_output =
        (((def_bldc.free_speed * orig_output - simBldc.getVelocity()) /
                 (105_u_A / 40_u_A) +
             simBldc.getVelocity()) /
            def_bldc.free_speed)
            .value();
    double output = (cut ? 0.0 : orig_output) +
                    ffmodel.FF(simBldc.getPosition(), simBldc.getVelocity(),
                        orig_output, cut);
    simBldc.SetCurrentLimit(40_u_A);
    simBldc.setControlTarget(output);
    simBldc.Tick(20_u_ms);
    double pos = simBldc.getPosition().value();
    double vel = simBldc.getVelocity().value();
    data_file << i << "," << pos << "," << vel << "," << output << "\n";
    std::cout << "Step " << i << ": pos=" << pos << " rad, vel=" << vel
              << " rad/s, output=" << output << ", cut=" << cut
              << ", x=" << x.value() << std::endl;
  }
  data_file.close();
  std::cout << "Final position: " << simBldc.getPosition().value() << " rad\n";
  std::cout << "Simulation data written to sim_data.csv\n";
  return 0;
}