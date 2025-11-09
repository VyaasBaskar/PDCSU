#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "control/icnor.h"
#include "control/util.h"
#include "gclass.h"
#include "simulation/simbldc.h"

using namespace pdcsu::control;
using namespace pdcsu::units;
using namespace pdcsu::util;
using namespace pdcsu::simulation;

namespace {

struct Scenario {
  std::string name;
  DefLinearSys sys;
  meter_t target_real;
  meter_t tol_inner_real;
  meter_t tol_outer_real;
  rpm_t speed_limit;
  amp_t current_limit;
  nm_t extra_load;
  int max_steps;
  meter_t settle_pos_tol;
  mps_t settle_vel_tol;
  int settle_window;
};

Scenario make_baseline() {
  DefBLDC def_bldc(105_u_A, 1.8_u_A, 2.5_u_Nm, 5676_u_rpm);
  DefLinearSys sys(def_bldc, 1, 214.85_u_rot / 262.5_u_in, 0.0_u_mps2, 3_u_kg,
      22_u_N, 0.5_u_N / 5676_u_rpm, 20_u_ms, 0.05_u_ohm);
  return {"baseline_linear", std::move(sys), 0.55_u_m, 0.003_u_m, 0.008_u_m,
      4200_u_rpm, 32_u_A, 0.0_u_Nm, 600, 0.004_u_m, 0.01_u_mps, 50};
}

Scenario make_heavy_payload() {
  DefBLDC def_bldc(140_u_A, 2.5_u_A, 3.8_u_Nm, 5200_u_rpm, 24_u_V);
  DefLinearSys sys(def_bldc, 1, 120_u_rot / 0.75_u_m, 4.0_u_mps2, 6.5_u_kg,
      30_u_N, 0.6_u_N / 4000_u_rpm, 25_u_ms, 0.08_u_ohm);
  return {"heavy_payload", std::move(sys), 0.38_u_m, 0.004_u_m, 0.010_u_m,
      3200_u_rpm, 22_u_A, 0.0_u_Nm, 700, 0.006_u_m, 0.02_u_mps, 70};
}

Scenario make_fast_loop() {
  DefBLDC def_bldc(90_u_A, 1.2_u_A, 1.9_u_Nm, 6100_u_rpm, 12_u_V);
  DefLinearSys sys(def_bldc, 1, 90_u_rot / 0.5_u_m, 0.0_u_mps2, 2.2_u_kg,
      10_u_N, 0.25_u_N / 5000_u_rpm, 10_u_ms, 0.03_u_ohm);
  return {"fast_loop", std::move(sys), 0.28_u_m, 0.002_u_m, 0.006_u_m,
      3000_u_rpm, 18_u_A, 0.0_u_Nm, 520, 0.003_u_m, 0.015_u_mps, 45};
}

Scenario make_dual_motor() {
  DefBLDC def_bldc(110_u_A, 1.9_u_A, 2.2_u_Nm, 5600_u_rpm, 18_u_V);
  DefLinearSys sys(def_bldc, 2, 150_u_rot / 1.0_u_m, 2.0_u_mps2, 5.0_u_kg,
      24_u_N, 0.5_u_N / 4500_u_rpm, 18_u_ms, 0.05_u_ohm);
  return {"dual_motor", std::move(sys), 0.65_u_m, 0.004_u_m, 0.010_u_m,
      3200_u_rpm, 24_u_A, 0.0_u_Nm, 680, 0.005_u_m, 0.018_u_mps, 60};
}

Scenario make_high_damping() {
  DefBLDC def_bldc(125_u_A, 2.2_u_A, 3.0_u_Nm, 5400_u_rpm, 18_u_V);
  DefLinearSys sys(def_bldc, 1, 180_u_rot / 0.9_u_m, 4.9_u_mps2, 4.0_u_kg,
      24_u_N, 0.9_u_N / 4500_u_rpm, 22_u_ms, 0.06_u_ohm);
  return {"high_damping", std::move(sys), 0.45_u_m, 0.003_u_m, 0.009_u_m,
      4200_u_rpm, 36_u_A, 0.02_u_Nm, 600, 0.004_u_m, 0.010_u_mps, 50};
}

}  // namespace

int main() {
  // Ensure output is not buffered
  std::cout.setf(std::ios::unitbuf);
  
  std::cout << "Initializing ICNOR benchmark scenarios..." << std::endl;
  
  std::vector<Scenario> scenarios;
  scenarios.reserve(5);
  scenarios.push_back(make_baseline());
  scenarios.push_back(make_heavy_payload());
  scenarios.push_back(make_fast_loop());
  scenarios.push_back(make_dual_motor());
  scenarios.push_back(make_high_damping());

  std::cout << "Running " << scenarios.size() << " scenarios..." << std::endl;
  
  bool all_quality_good = true;

  std::cout << std::fixed << std::setprecision(3);

  for (auto &scenario : scenarios) {
    std::cout << "\n=== " << scenario.name << " ===" << std::endl;
    
    try {
      ICNORPositionControl icnor(scenario.sys);
      icnor.setProjectionHorizon(4);
      icnor.setConstraints(scenario.speed_limit, scenario.current_limit);
      icnor.setTolerance(scenario.sys.toNative(scenario.tol_inner_real),
          scenario.sys.toNative(scenario.tol_outer_real));
      
      auto learner =
          std::make_shared<ICNORLearner>("icnor_history_" + scenario.name);
      learner->setAutoSaveStride(5);
      icnor.attachLearner(learner);

      SimBLDC sim(scenario.sys);
      sim.SetCurrentLimit(scenario.current_limit);
      sim.SetLoad(scenario.extra_load);
      auto target_native = scenario.sys.toNative(scenario.target_real);
      radps_t zero_velocity = 0_u_radps;

      auto control_period = scenario.sys.control_period;
      int max_steps = scenario.max_steps;
      double target_real_value = scenario.target_real.value();
      double settle_pos_tol = scenario.settle_pos_tol.value();
      double settle_vel_tol = scenario.settle_vel_tol.value();
      std::vector<double> time_samples;
      std::vector<double> pos_samples;
      std::vector<double> vel_samples;
      time_samples.reserve(max_steps);
      pos_samples.reserve(max_steps);
      vel_samples.reserve(max_steps);

      double accumulated_ns = 0.0;
      int calls = 0;
      int settle_counter = 0;

      for (int step = 0;
           step < max_steps && settle_counter < scenario.settle_window; ++step) {
        auto t_start = std::chrono::steady_clock::now();
        double duty_cycle = icnor.getOutput(
            target_native, zero_velocity, sim.getPosition(), sim.getVelocity());
        auto t_end = std::chrono::steady_clock::now();

        accumulated_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(
                              t_end - t_start)
                              .count();
        ++calls;

        sim.setControlTarget(duty_cycle);
        sim.Tick(control_period);

        double time_sec = step * control_period.value() / 1000.0;
        double pos_real = scenario.sys.toReal(sim.getPosition()).value();
        double vel_real = scenario.sys.toReal(sim.getVelocity()).value();
        double vel_for_eval = std::max(vel_real, 0.0);

        time_samples.push_back(time_sec);
        pos_samples.push_back(pos_real);
        vel_samples.push_back(vel_for_eval);

        double pos_error = std::abs(pos_real - target_real_value);
        double vel_mag = std::abs(vel_real);
        if (pos_error <= settle_pos_tol && vel_mag <= settle_vel_tol) {
          ++settle_counter;
        } else {
          settle_counter = 0;
        }
      }

      if (!time_samples.empty()) {
        const int pad_samples = 40;
        double time_increment = control_period.value() / 1000.0;
        double last_time = time_samples.back();
        double last_pos = pos_samples.back();
        for (int i = 1; i <= pad_samples; ++i) {
          time_samples.push_back(last_time + i * time_increment);
          pos_samples.push_back(last_pos);
          vel_samples.push_back(0.0);
        }
      }

      auto [quality, behavior] =
          gclass::classify_trace(time_samples, pos_samples, vel_samples);
      bool pass_quality = (quality == "great" || quality == "good");
      if (!pass_quality) all_quality_good = false;

      double avg_ns = (calls > 0) ? (accumulated_ns / calls) : 0.0;

      double final_pos = pos_samples.empty() ? 0.0 : pos_samples.back();
      double final_vel = vel_samples.empty() ? 0.0 : vel_samples.back();

      learner->saveIfDirty();

      std::cout << "[" << scenario.name << "] "
                << "avg getOutput: " << avg_ns << " ns, "
                << "final pos: " << final_pos << " m, "
                << "final vel: " << final_vel << " m/s, "
                << "quality: " << quality << ", behavior: " << behavior << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "ERROR in scenario " << scenario.name << ": " << e.what() << std::endl;
      all_quality_good = false;
    } catch (...) {
      std::cerr << "UNKNOWN ERROR in scenario " << scenario.name << std::endl;
      all_quality_good = false;
    }
  }

  if (!all_quality_good) {
    std::cerr << "ICNOR quality check failed for one or more scenarios." << std::endl;
    return 1;
  }

  std::cout << "All scenarios passed quality requirements." << std::endl;
  return 0;
}

