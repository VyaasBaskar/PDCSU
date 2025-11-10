#ifdef _WIN32
#include <crtdbg.h>
#endif
#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
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
      11_u_N, 0.55_u_N / 5676_u_rpm, 20_u_ms, 0.05_u_ohm);
  return {"baseline_linear", std::move(sys), 0.55_u_m, 0.015_u_m, 0.03_u_m,
      4200_u_rpm, 32_u_A, 0.0_u_Nm, 600, 0.004_u_m, 0.01_u_mps, 50};
}

Scenario make_heavy_payload() {
  DefBLDC def_bldc(140_u_A, 2.5_u_A, 3.8_u_Nm, 5200_u_rpm, 24_u_V);
  DefLinearSys sys(def_bldc, 1, 120_u_rot / 0.75_u_m, 0.0_u_mps2, 6.5_u_kg,
      22_u_N, 1.1_u_N / 4000_u_rpm, 25_u_ms, 0.08_u_ohm);
  return {"heavy_payload", std::move(sys), 0.38_u_m, 0.015_u_m, 0.03_u_m,
      3200_u_rpm, 22_u_A, 0.0_u_Nm, 700, 0.006_u_m, 0.02_u_mps, 70};
}

Scenario make_fast_loop() {
  DefBLDC def_bldc(90_u_A, 1.2_u_A, 1.9_u_Nm, 6100_u_rpm, 12_u_V);
  DefLinearSys sys(def_bldc, 1, 90_u_rot / 0.5_u_m, 0.0_u_mps2, 2.2_u_kg,
      0_u_N, 0_u_N / 5000_u_rpm, 10_u_ms, 0.03_u_ohm);
  return {"fast_loop", std::move(sys), 0.28_u_m, 0.015_u_m, 0.03_u_m,
      3000_u_rpm, 18_u_A, 0.0_u_Nm, 520, 0.003_u_m, 0.015_u_mps, 45};
}

Scenario make_dual_motor() {
  DefBLDC def_bldc(110_u_A, 1.9_u_A, 2.2_u_Nm, 5600_u_rpm, 18_u_V);
  DefLinearSys sys(def_bldc, 2, 150_u_rot / 1.0_u_m, 2.0_u_mps2, 5.0_u_kg,
      10_u_N, 0.5_u_N / 4500_u_rpm, 18_u_ms, 0.05_u_ohm);
  return {"dual_motor", std::move(sys), 0.65_u_m, 0.015_u_m, 0.03_u_m,
      3200_u_rpm, 24_u_A, 0.0_u_Nm, 680, 0.005_u_m, 0.018_u_mps, 60};
}

Scenario make_high_damping() {
  DefBLDC def_bldc(125_u_A, 2.2_u_A, 3.0_u_Nm, 5400_u_rpm, 18_u_V);
  DefLinearSys sys(def_bldc, 1, 180_u_rot / 0.9_u_m, 4.9_u_mps2, 4.0_u_kg,
      25_u_N, 1_u_N / 4500_u_rpm, 22_u_ms, 0.06_u_ohm);
  return {"high_damping", std::move(sys), 0.45_u_m, 0.015_u_m, 0.03_u_m,
      4200_u_rpm, 36_u_A, 0.02_u_Nm, 600, 0.004_u_m, 0.010_u_mps, 50};
}

Scenario make_model_mismatch() {
  DefBLDC estimator_bldc(90_u_A, 1.8_u_A, 4.0_u_Nm, 5000_u_rpm, 12_u_V);
  DefLinearSys estimator_sys(estimator_bldc, 1, 180_u_rot / 0.75_u_m,
      0.0_u_mps2, 2.8_u_kg, 15_u_N, 0.3_u_N / 3800_u_rpm, 20_u_ms, 0.04_u_ohm);

  DefBLDC actual_bldc(180_u_A, 3.5_u_A, 5.5_u_Nm, 6200_u_rpm, 28_u_V);
  DefLinearSys actual_sys(actual_bldc, 2, 240_u_rot / 0.7_u_m,
      7.5_u_mps2, 8.8_u_kg, 28_u_N, 1.2_u_N / 5200_u_rpm, 12_u_ms, 0.12_u_ohm);

  Scenario scenario{"model_mismatch", estimator_sys, 0.52_u_m,
      0.03_u_m, 0.06_u_m, 3600_u_rpm, 30_u_A, 0.00_u_Nm, 850, 0.008_u_m,
      0.020_u_mps, 70};
  scenario.sys = actual_sys;
  return scenario;
}

}  // namespace

int main() {
#ifdef _DEBUG
#ifdef _WIN32
  _CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_FILE);
  _CrtSetReportFile(_CRT_WARN, _CRTDBG_FILE_STDERR);
  _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_CHECK_ALWAYS_DF | _CRTDBG_LEAK_CHECK_DF);
#endif
#endif

  // Ensure output is not buffered
  std::cout.setf(std::ios::unitbuf);
  
  std::cout << "Initializing ICNOR benchmark scenarios..." << std::endl;
  std::filesystem::path results_dir = std::filesystem::current_path() / "results";
  std::error_code mkdir_ec;
  std::filesystem::create_directories(results_dir, mkdir_ec);
  if (mkdir_ec) {
    std::cerr << "[warning] unable to create results directory at "
              << results_dir << ": " << mkdir_ec.message() << std::endl;
  }
  
  std::vector<Scenario> scenarios;
  scenarios.reserve(5);
  scenarios.push_back(make_baseline());
  scenarios.push_back(make_heavy_payload());
  scenarios.push_back(make_fast_loop());
  scenarios.push_back(make_dual_motor());
  scenarios.push_back(make_high_damping());
  scenarios.push_back(make_model_mismatch());

  std::cout << "Running " << scenarios.size() << " scenarios..." << std::endl;
  
  std::cout << std::fixed << std::setprecision(3);

  for (auto &scenario : scenarios) {
    std::cout << "\n=== " << scenario.name << " ===" << std::endl;
    
    try {
      ICNORPositionControl icnor(scenario.sys);
      icnor.setProjectionHorizon(4);
      icnor.setConstraints(scenario.speed_limit, scenario.current_limit);
      auto tol_inner = scenario.sys.toNative(scenario.tol_inner_real);
      auto tol_outer = scenario.sys.toNative(scenario.tol_outer_real);
      icnor.setTolerance(tol_inner, tol_outer);
      auto learner = std::make_shared<ICNORLearner>(
          (results_dir / ("icnor_history_" + scenario.name)).string());
      learner->setAutoSaveStride(60);
      icnor.attachLearner(learner);

      SimBLDC sim(scenario.sys);
      sim.SetCurrentLimit(scenario.current_limit);
      sim.SetLoad(scenario.extra_load);
      radps_t zero_velocity = 0_u_radps;

      auto control_period = scenario.sys.control_period;
      int max_steps = scenario.max_steps;
      double settle_pos_tol = scenario.settle_pos_tol.value();
      double settle_vel_tol = scenario.settle_vel_tol.value();
      std::vector<double> time_samples;
      std::vector<double> pos_samples;
      std::vector<double> vel_samples;
      std::vector<double> output_samples;
      std::vector<meter_t> waypoints = {
          0.35 * scenario.target_real,
          scenario.target_real,
          0.6 * scenario.target_real,
          0.0_u_m};

      time_samples.reserve(max_steps * waypoints.size());
      pos_samples.reserve(max_steps * waypoints.size());
      vel_samples.reserve(max_steps * waypoints.size());
      output_samples.reserve(max_steps * waypoints.size());

      double accumulated_ns = 0.0;
      int calls = 0;
      int global_step = 0;
      double dt_seconds = control_period.value() / 1000.0;

      struct MotionReport {
        std::string label;
        std::vector<double> time;
        std::vector<double> pos;
        std::vector<double> vel;
        std::vector<double> output;
        std::string quality;
        std::string behavior;
        double final_pos = 0.0;
        double final_vel = 0.0;
        bool history_saved = false;
      };

      std::vector<MotionReport> motion_reports;
      motion_reports.reserve(waypoints.size());

      for (size_t waypoint_idx = 0; waypoint_idx < waypoints.size();
           ++waypoint_idx) {
        const auto &target_real = waypoints[waypoint_idx];
        auto target_native = scenario.sys.toNative(target_real);
        double target_real_value = target_real.value();
        int settle_counter = 0;
        int steps = 0;

        MotionReport report;
        report.label = scenario.name + "_motion_" + std::to_string(waypoint_idx + 1);

        while (steps < max_steps && settle_counter < scenario.settle_window) {
          auto t_start = std::chrono::steady_clock::now();
          double duty_cycle = icnor.getOutput(
              target_native, zero_velocity, sim.getPosition(), sim.getVelocity());
          auto t_end = std::chrono::steady_clock::now();

          accumulated_ns +=
              std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start)
                  .count();
          ++calls;

          sim.setControlTarget(duty_cycle);
          sim.Tick(control_period);

          double time_sec = global_step * dt_seconds;
          double pos_real = scenario.sys.toReal(sim.getPosition()).value();
          double vel_real = scenario.sys.toReal(sim.getVelocity()).value();
          double vel_for_eval = std::max(vel_real, 0.0);

          report.time.push_back(time_sec);
          report.pos.push_back(pos_real);
          report.vel.push_back(vel_for_eval);
          report.output.push_back(duty_cycle);

          time_samples.push_back(time_sec);
          pos_samples.push_back(pos_real);
          vel_samples.push_back(vel_for_eval);
          output_samples.push_back(duty_cycle);

          double pos_error = std::abs(pos_real - target_real_value);
          double vel_mag = std::abs(vel_real);
          if (pos_error <= settle_pos_tol && vel_mag <= settle_vel_tol) {
            ++settle_counter;
          } else {
            settle_counter = 0;
          }

          ++steps;
          ++global_step;
        }

        if (!report.time.empty()) {
          const int pad_samples = 40;
          double last_time = report.time.back();
          double last_pos = report.pos.back();
          for (int i = 1; i <= pad_samples; ++i) {
            report.time.push_back(last_time + i * dt_seconds);
            report.pos.push_back(last_pos);
            report.vel.push_back(0.0);
            report.output.push_back(0.0);
          }
        }

        auto [quality, behavior] =
            gclass::classify_trace(report.time, report.pos, report.vel);
        report.quality = quality;
        report.behavior = behavior;
        report.final_pos = report.pos.empty() ? 0.0 : report.pos.back();
        report.final_vel = report.vel.empty() ? 0.0 : report.vel.back();

        report.history_saved = learner->saveIfDirty();
        if (report.history_saved) {
          std::cout << "  [history] saved to " << learner->storagePath() << std::endl;
        } else {
          std::cout << "  [history] no save after motion "
                    << (waypoint_idx + 1) << " (dirty flag not set yet)" << std::endl;
        }

        std::cout << "  Motion " << (waypoint_idx + 1)
                  << " target " << target_real_value << " m => quality: " << quality
                  << ", behavior: " << behavior << ", final pos: " << report.final_pos
                  << " m, final vel: " << report.final_vel << " m/s" << std::endl;

        motion_reports.push_back(std::move(report));
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
          output_samples.push_back(0.0);
        }
      }

      double avg_ns = (calls > 0) ? (accumulated_ns / calls) : 0.0;

      double final_pos = pos_samples.empty() ? 0.0 : pos_samples.back();
      double final_vel = vel_samples.empty() ? 0.0 : vel_samples.back();

      bool final_save = learner->saveIfDirty();
      if (final_save) {
        std::cout << "  [history] saved to " << learner->storagePath() << std::endl;
      }

      std::filesystem::path history_path(learner->storagePath());
      if (!learner->storagePath().empty()) {
        bool exists = std::filesystem::exists(history_path);
        std::cout << "  History file "
                  << (exists ? "present: " : "missing: ")
                  << history_path.string() << std::endl;
      }

      std::ofstream csv(results_dir / (scenario.name + ".csv"), std::ios::trunc);
      if (csv.is_open()) {
        csv << "step,pos,vel,output\n";
        for (size_t i = 0; i < pos_samples.size(); ++i) {
          double pos = pos_samples[i];
          double vel = i < vel_samples.size() ? vel_samples[i] : 0.0;
          double out = i < output_samples.size() ? output_samples[i] : 0.0;
          csv << i << ',' << pos << ',' << vel << ',' << out << '\n';
        }
      } else {
        std::cerr << "[warning] unable to open results CSV for " << scenario.name << std::endl;
      }

      std::cout << "[" << scenario.name << "] "
                << "avg getOutput: " << avg_ns << " ns, "
                << "final pos: " << final_pos << " m, "
                << "final vel: " << final_vel << " m/s" << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "ERROR in scenario " << scenario.name << ": " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "UNKNOWN ERROR in scenario " << scenario.name << std::endl;
    }
  }

  return 0;
}

