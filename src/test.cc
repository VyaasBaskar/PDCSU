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
  DefBLDC def_bldc(105_A_, 1.8_A_, 2.5_Nm_, 5676_rpm_);
  DefLinearSys def_sys(def_bldc, 1, 214.85_rot_ / 262.5_in_, 0.0_mps2_, 3_kg_,
      22_N_, 0.5_N_ / 5676_rpm_, 20_ms_, 0_ohm_);
  amp_t clim = 40_A_;

  ICNORPositionControl icnor(def_sys);
  icnor.setProjectionHorizon(3);

  SimBLDC simBldc = SimBLDC(def_sys);
  simBldc.SetCurrentLimit(clim);

  icnor.setConstraints(5000_rpm_, clim);
  icnor.setTolerance(def_sys.toNative(0.25_in_), def_sys.toNative(0.5_in_));

  std::ofstream data_file("sim_data.csv");
  data_file << "step,pos,vel,output\n";

  std::vector<double> v_pos;
  std::vector<double> v_vel;
  std::vector<double> v_time;

  for (int i = 0; i < 100; i++) {
    simBldc.setControlTarget(icnor.getOutput(def_sys.toNative(45_in_), 0_rad_,
        simBldc.getPosition(), simBldc.getVelocity()));
    simBldc.Tick(def_sys.control_period);
    double pos = inch_t(def_sys.toReal(simBldc.getPosition())).value();
    double vel = fps_t(def_sys.toReal(simBldc.getVelocity())).value();
    data_file << i << "," << pos << "," << vel << "\n";
    std::cout << "Step " << i << ": pos=" << pos << " in, vel=" << vel
              << " ft/s" << std::endl;

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