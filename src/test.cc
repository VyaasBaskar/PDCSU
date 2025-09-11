#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include "control/icnor.h"
#include "control/util.h"
#include "gclass.h"
#include "simulation/simbldc.h"

using namespace pdcsu::control;
using namespace pdcsu::units;
using namespace pdcsu::util;
using namespace pdcsu::simulation;

int main() {
  DefBLDC def_bldc(105_u_A, 1.8_u_A, 2.5_u_Nm, 5676_u_rpm);
  DefLinearSys def_sys(def_bldc, 1, 214.85_u_rot / 262.5_u_in, 0.0_u_mps2,
      3_u_kg, 40_u_N, 3_u_N / 5676_u_rpm, 20_u_ms);

  amp_t clim = 40_u_A;

  ICNORPositionControl icnor(def_sys);
  icnor.setProjectionHorizon(1);

  auto x = 45_u_in;

  SimBLDC simBldc = SimBLDC(def_sys);
  simBldc.SetCurrentLimit(clim);

  icnor.setConstraints(5000_u_rpm, clim);

  std::ofstream data_file("sim_data.csv");
  data_file << "step,pos,vel,output\n";

  std::vector<double> v_pos;
  std::vector<double> v_vel;
  std::vector<double> v_time;

  for (int i = 0; i < 100; i++) {
    simBldc.setControlTarget(icnor.getOutput(def_sys.toNative(45_u_in),
        0_u_radps, simBldc.getPosition(), simBldc.getVelocity()));
    simBldc.Tick(def_sys.control_period);
    double pos = inch_t(def_sys.toReal(simBldc.getPosition())).value();
    double vel = fps_t(def_sys.toReal(simBldc.getVelocity())).value();
    data_file << i << "," << pos << "," << vel << "\n";
    std::cout << "Step " << i << ": pos=" << pos << " in, vel=" << vel
              << " ft/s, x=" << x.value() << std::endl;

    v_time.push_back(i * def_sys.control_period.value() / 1000.0);
    v_pos.push_back(pos);
    v_vel.push_back(vel);
  }
  data_file.close();
  std::cout << "Final position: " << simBldc.getPosition().value() << " rad\n";
  std::cout << "Simulation data written to sim_data.csv\n";

  auto [quality, behavior] = gclass::classify_trace(v_time, v_pos, v_vel);
  std::cout << "Quality: " << quality << ", Behavior: " << behavior << "\n";
  return 0;
}