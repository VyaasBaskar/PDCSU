#ifdef _WIN32
#include <crtdbg.h>
#endif
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "util/sysdef.h"
#include "util/units.h"
#include "simulation/simbldc.h"

using namespace pdcsu::util;
using namespace pdcsu::units;
using namespace pdcsu::simulation;

struct Scenario {
  std::string name;
  DefLinearSys sys;
  amp_t current_limit;
  nm_t external_load;
  int steps;
};

Scenario make_baseline() {
  DefBLDC def_bldc(105_u_A, 1.8_u_A, 2.5_u_Nm, 5676_u_rpm);
  DefLinearSys sys(def_bldc, 1, 214.85_u_rot / 262.5_u_in, 0.0_u_mps2, 3_u_kg,
      11_u_N, 0.55_u_N / 5676_u_rpm, 20_u_ms, 0.05_u_ohm);
  return {"baseline_linear", std::move(sys), 32_u_A, 0.0_u_Nm, 800};
}

Scenario make_fast_loop() {
  DefBLDC def_bldc(90_u_A, 1.2_u_A, 1.9_u_Nm, 6100_u_rpm, 12_u_V);
  DefLinearSys sys(def_bldc, 1, 90_u_rot / 0.5_u_m, 0.0_u_mps2, 2.2_u_kg, 0_u_N,
      0_u_N / 5000_u_rpm, 10_u_ms, 0.03_u_ohm);
  return {"fast_loop", std::move(sys), 18_u_A, 0.0_u_Nm, 600};
}

int main() {
#ifdef _DEBUG
#ifdef _WIN32
  _CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_FILE);
  _CrtSetReportFile(_CRT_WARN, _CRTDBG_FILE_STDERR);
  _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_CHECK_ALWAYS_DF |
                 _CRTDBG_LEAK_CHECK_DF);
#endif
#endif

  std::cout.setf(std::ios::unitbuf);

  std::filesystem::path results_dir = std::filesystem::current_path() / "results";
  std::error_code mkdir_ec;
  std::filesystem::create_directories(results_dir, mkdir_ec);
  if (mkdir_ec) {
    std::cerr << "[warning] unable to create results directory at " << results_dir
              << ": " << mkdir_ec.message() << std::endl;
  }

  std::vector<Scenario> scenarios;
  scenarios.push_back(make_baseline());
  scenarios.push_back(make_fast_loop());

  std::cout << "Running " << scenarios.size() << " SimBLDC scenarios..." << std::endl;

  for (auto &scenario : scenarios) {
    std::cout << "\n=== " << scenario.name << " ===" << std::endl;

    try {
      SimBLDC sim(scenario.sys);
      sim.SetCurrentLimit(scenario.current_limit);
      sim.SetLoad(scenario.external_load);

      const int hold_steps = 50;
      const int return_steps = 800;
      const double pos_tolerance = 0.0005;
      const double vel_tolerance = 0.005;
      std::vector<double> pos_samples, vel_samples, output_samples;
      pos_samples.reserve(scenario.steps + return_steps + hold_steps);
      vel_samples.reserve(scenario.steps + return_steps + hold_steps);
      output_samples.reserve(scenario.steps + return_steps + hold_steps);

      for (int step = 0; step < scenario.steps; ++step) {
        double phase = (2.0 * step) / scenario.steps;
        double DC = (phase < 1.0) ? (-0.8 + 1.6 * phase) : (0.8 - 1.6 * (phase - 1.0));

        sim.setControlTarget(DC);
        sim.Tick(scenario.sys.control_period);

        pos_samples.push_back(scenario.sys.toReal(sim.getPosition()).value());
        vel_samples.push_back(scenario.sys.toReal(sim.getVelocity()).value());
        output_samples.push_back(DC);
      }

      int settle_count = 0;
      const int settle_window = 20;
      for (int step = 0; step < return_steps; ++step) {
        double pos_real = scenario.sys.toReal(sim.getPosition()).value();
        double vel_real = scenario.sys.toReal(sim.getVelocity()).value();
        
        if (std::abs(pos_real) < pos_tolerance && std::abs(vel_real) < vel_tolerance) {
          ++settle_count;
          if (settle_count >= settle_window) {
            break;
          }
        } else {
          settle_count = 0;
        }

        double kp = 1.0;
        double max_dc = 0.3;
        double DC = std::clamp(-kp * vel_real, -max_dc, max_dc);

        sim.setControlTarget(DC);
        sim.Tick(scenario.sys.control_period);

        pos_samples.push_back(scenario.sys.toReal(sim.getPosition()).value());
        vel_samples.push_back(scenario.sys.toReal(sim.getVelocity()).value());
        output_samples.push_back(DC);
      }

      for (int step = 0; step < hold_steps; ++step) {
        sim.setControlTarget(0.0);
        sim.Tick(scenario.sys.control_period);

        pos_samples.push_back(scenario.sys.toReal(sim.getPosition()).value());
        vel_samples.push_back(scenario.sys.toReal(sim.getVelocity()).value());
        output_samples.push_back(0.0);
      }

      std::filesystem::path csvpath = results_dir / ("simbldc_" + scenario.name + ".csv");
      std::ofstream csv(csvpath, std::ios::trunc);
      if (csv.is_open()) {
        csv << std::fixed << std::setprecision(6) << "step,pos,vel,output\n";
        for (size_t i = 0; i < pos_samples.size(); ++i) {
          csv << i << ',' << pos_samples[i] << ',' << vel_samples[i]
              << ',' << output_samples[i] << '\n';
        }
        csv.close();
        std::cout << "  wrote " << csvpath << std::endl;

        std::filesystem::path graphpath = results_dir / ("simbldc_" + scenario.name + ".png");
        std::filesystem::path script_path = std::filesystem::current_path() / "graph_simbldc.py";
        if (std::filesystem::exists(script_path)) {
          std::string csv_str = csvpath.string();
          std::string graph_str = graphpath.string();
          std::string script_str = script_path.string();
#ifdef _WIN32
          std::string cmd = "python \"" + script_str + "\" \"" + csv_str + "\" \"" + graph_str + "\"";
#else
          std::string cmd = "python3 \"" + script_str + "\" \"" + csv_str + "\" \"" + graph_str + "\"";
#endif
          int result = std::system(cmd.c_str());
          if (result == 0) {
            std::cout << "  generated graph: " << graphpath << std::endl;
          } else {
            std::cerr << "  [warning] graph generation failed for " << scenario.name << std::endl;
          }
        } else {
          std::cerr << "  [warning] graph script not found: " << script_path << std::endl;
        }
      } else {
        std::cerr << "[warning] unable to open results CSV for " << scenario.name << std::endl;
      }

      if (!pos_samples.empty()) {
        std::cout << "  final pos: " << pos_samples.back() << " m, final vel: "
                  << vel_samples.back() << " m/s" << std::endl;
      }
    } catch (const std::exception &e) {
      std::cerr << "ERROR in scenario " << scenario.name << ": " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "UNKNOWN ERROR in scenario " << scenario.name << std::endl;
    }
  }

  return 0;
}
