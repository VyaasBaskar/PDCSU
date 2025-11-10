#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

struct gclass {
  static constexpr double OVERSHOOT_THRESH = 0.01;
  static constexpr double SETTLING_TOL = 0.02;
  static constexpr double PAUSE_FRAC = 0.10;
  static constexpr double VEL_PAUSE_THRESH = 0.01;
  static constexpr double OSC_AMPLITUDE_FRAC = 0.005;

  static double compute_target(
      const std::vector<double>& pos, double last_frac = 0.10) {
    int n = pos.size();
    int m = std::max(1, (int)std::ceil(n * last_frac));
    double sum = std::accumulate(pos.end() - m, pos.end(), 0.0);
    return sum / m;
  }

  static std::pair<double, int> compute_overshoot(
      const std::vector<double>& pos, double target) {
    auto it = std::max_element(pos.begin(), pos.end());
    int idx = std::distance(pos.begin(), it);
    double peak_val = *it;
    if (target == 0.0) return {peak_val - target, idx};
    return {(peak_val - target) / std::abs(target), idx};
  }

  static double compute_settling_time(const std::vector<double>& time,
      const std::vector<double>& error, double tol, double target) {
    double abs_tol = tol * (target != 0 ? std::abs(target) : 1.0);
    for (int i = 0; i < (int)error.size(); i++) {
      bool all_within = true;
      for (int j = i; j < (int)error.size(); j++) {
        if (std::abs(error[j]) > abs_tol) {
          all_within = false;
          break;
        }
      }
      if (all_within) return time[i];
    }
    return NAN;
  }

  static int count_significant_oscillations(
      const std::vector<double>& error, double target, double amp_frac) {
    double amp_thresh = amp_frac * (target != 0 ? std::abs(target) : 1.0);
    int count = 0;
    for (int i = 1; i < (int)error.size() - 1; i++) {
      double d1 = error[i] - error[i - 1];
      double d2 = error[i + 1] - error[i];
      if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) {
        if (std::abs(error[i]) >= amp_thresh) count++;
      }
    }
    return count;
  }

  static std::tuple<bool, int, int> detect_start_and_stop(
      const std::vector<double>& time, const std::vector<double>& velocity,
      const std::vector<double>& pos, double target, double error_frac = 0.10,
      int min_crossings = 2, int min_samples_between = 2,
      double close_radius_mult = 3.0) {
    (void)time;
    int n = (int)velocity.size();
    if (n < 3 || (int)pos.size() != n) return {false, -1, -1};

    double err_thresh = error_frac * (target != 0.0 ? std::abs(target) : 1.0);

    std::vector<double> err(n);
    for (int i = 0; i < n; ++i)
      err[i] = pos[i] - target;

    int cross_count = 0;
    int first_idx = -1, last_idx = -1;
    int last_cross_idx = -1000000;

    for (int i = 1; i < n; ++i) {
      if (err[i] * err[i - 1] < 0.0) {
        double amp = std::max(std::abs(err[i]), std::abs(err[i - 1]));
        if (amp >= err_thresh) {
          if (first_idx == -1) first_idx = i - 1;
          if (i - last_cross_idx >= min_samples_between) {
            ++cross_count;
            last_idx = i;
            last_cross_idx = i;
          }
        }
      }
    }

    if (cross_count < min_crossings) {
      double close_radius = close_radius_mult * err_thresh;
      for (int i = 1; i < n; ++i) {
        if (velocity[i] * velocity[i - 1] < 0.0) {
          if (std::abs(err[i]) <= close_radius ||
              std::abs(err[i - 1]) <= close_radius) {
            if (first_idx == -1) first_idx = i - 1;
            if (i - last_cross_idx >= min_samples_between) {
              ++cross_count;
              last_idx = i;
              last_cross_idx = i;
            }
          }
        }
      }
    }

    bool valid = (cross_count >= min_crossings);
    if (!valid) return {false, -1, -1};

    first_idx = std::max(0, first_idx);
    last_idx = std::min(n - 1, last_idx == -1 ? first_idx : last_idx);
    return {true, first_idx, last_idx};
  }

  static std::pair<std::string, std::string> classify_trace(
      const std::vector<double>& time, const std::vector<double>& pos,
      const std::vector<double>& vel) {
    int n = pos.size();
    if (n < 3) return {"poor", "none"};

    double target = compute_target(pos);
    std::vector<double> error(n);
    for (int i = 0; i < n; i++)
      error[i] = pos[i] - target;

    auto [overshoot_val, peak_idx] = compute_overshoot(pos, target);
    double settling_time =
        compute_settling_time(time, error, SETTLING_TOL, target);
    double total_time =
        (time.back() - time.front() != 0) ? (time.back() - time.front()) : 1.0;

    int osc_count =
        count_significant_oscillations(error, target, OSC_AMPLITUDE_FRAC);

    auto [starting_and_stopping, stop_start_idx, stop_end_idx] =
        detect_start_and_stop(time, vel, pos, target);

    bool peak_before_end =
        peak_idx < (n - std::max(1, (int)std::ceil(n * 0.1)));
    bool overshooting = (overshoot_val > OVERSHOOT_THRESH) && peak_before_end;
    bool chattering = osc_count >= 4;

    std::string behavior = "none";
    if (overshooting)
      behavior = "overshooting";
    else if (chattering)
      behavior = "chattering";
    else if (starting_and_stopping)
      behavior = "starting and stopping";

    int score = 0;
    if (overshoot_val > OVERSHOOT_THRESH) {
      score +=
          1 + (int)(0.4 * std::min(1.0, (overshoot_val - OVERSHOOT_THRESH) /
                                            std::max(OVERSHOOT_THRESH, 1e-6)));
    }
    if (std::isnan(settling_time))
      score += 3;
    else if ((settling_time - time.front()) > 0.3 * total_time)
      score += 1;

    if (osc_count >= 3)
      score += 2;
    else if (osc_count >= 1)
      score += 1;

    double transient_min = *std::min_element(
        pos.begin(), pos.begin() + std::max(1, (int)(0.5 * n)));
    if (target != 0 && (target - transient_min) / std::abs(target) > 0.2)
      score += 2;

    if (starting_and_stopping) score += 2;

    std::string quality;
    if (score <= 2)
      quality = "great";
    else if (score <= 3)
      quality = "good";
    else if (score <= 7)
      quality = "ok";
    else
      quality = "poor";

    return {quality, behavior};
  }
};