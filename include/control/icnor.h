#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "control/util.h"
#include "util/sysdef.h"

#ifdef _WIN32
#include "corecrt_math_defines.h"
#endif

/* Inexpensive Constrained Nonlinear Optimal Regulator */

using namespace pdcsu::util;
using namespace pdcsu::units;

namespace pdcsu::control {

namespace icnor_internal {
struct ICNORTuningParameters {
  double time_scale_factor = 1.0;
  double time_offset = 0.0;
  double z_fudge = 1.0;
  double load_scale = 1.0;
  double load_bias = 0.0;
  double friction_scale = 1.0;
};

class ICNOR;
}

struct ICNORLearningSample {
  int64_t timestamp_ms = 0;
  double target_position = 0.0;
  double target_velocity = 0.0;
  double state_position = 0.0;
  double state_velocity = 0.0;
  double tstar = 0.0;
  double zeta = 0.0;
  double alpha = 0.0;
  double beta = 0.0;
  double gamma = 0.0;
  double v_max = 0.0;
  double sysvmax = 0.0;
  double control_period = 0.0;
  double max_control_target = 0.0;
  double position_error = 0.0;
  double velocity_error = 0.0;
  double beta_abs = 0.0;
  double gamma_abs = 0.0;
  double t_hi_init = 0.0;
  double t_lo_init = 0.0;
  double distance_to_target = 0.0;
  bool saturated = false;
  bool solved = false;
};

class ICNORLearner : public std::enable_shared_from_this<ICNORLearner> {
public:
  struct UpdateResult {
    std::optional<icnor_internal::ICNORTuningParameters> new_tuning;
    bool should_save = false;
  };

  ICNORLearner();
  explicit ICNORLearner(std::string storage_path);
  ~ICNORLearner();

  void setStoragePath(const std::string &path);
  std::string storagePath() const;

  void enableAutoSave(bool enabled);
  void setAutoSaveStride(size_t stride);

  void loadAsync();
  void saveAsync();
  bool saveIfDirty();

  UpdateResult notifyOptimizationResult(
      const ICNORLearningSample &sample,
      const icnor_internal::ICNORTuningParameters &current_params);

  std::optional<icnor_internal::ICNORTuningParameters> suggestFromHistory(
      const icnor_internal::ICNORTuningParameters &current_params);
  
  icnor_internal::ICNORTuningParameters getCurrentTuning() const;

private:
  static std::string ensureExtension(std::string path);
  static bool endsWithInsensitive(
      const std::string &value, const std::string &ending);

  struct MotionMetrics {
    size_t samples = 0;
    size_t hi_samples = 0;
    double avg_hi_ratio = 0.65;
    double saturation_ratio = 0.0;
    double failure_ratio = 0.0;
    double avg_peak_error = 0.0;
    double avg_control_ratio = 0.0;
    double final_position_error = 0.0;
    double final_velocity_error = 0.0;
    double distance_travelled = 0.0;
  double overshoot_ratio = 0.0;
  double settled_fraction = 0.0;
  double settling_time_s = 0.0;
  int oscillation_events = 0;
  double max_velocity = 0.0;
  double initial_velocity_abs = 0.0;
  double initial_position_offset = 0.0;
    bool completed = false;
  };

  struct MotionAccumulator {
    bool active = false;
    double target_position = 0.0;
    double target_velocity = 0.0;
    double start_distance = 0.0;
    double last_distance = 0.0;
    size_t samples = 0;
    size_t hi_samples = 0;
    size_t saturated_samples = 0;
    size_t failure_samples = 0;
    double sum_hi_ratio = 0.0;
    double sum_peak_error = 0.0;
    double sum_control_ratio = 0.0;
    double final_position_error = 0.0;
    double final_velocity_error = 0.0;
    double norm_denominator = 1.0;
    double max_norm_error = 0.0;
    double last_norm_error = 0.0;
    bool last_error_valid = false;
    int oscillation_events = 0;
    int last_violation_index = -1;
    double control_period_s = 0.0;
    double max_velocity_mag = 0.0;
    double initial_velocity_sum = 0.0;
    double initial_position_sum = 0.0;
    int initial_sample_count = 0;

    void reset() {
      active = false;
      target_position = 0.0;
      target_velocity = 0.0;
      start_distance = 0.0;
      last_distance = 0.0;
      samples = 0;
      hi_samples = 0;
      saturated_samples = 0;
      failure_samples = 0;
      sum_hi_ratio = 0.0;
      sum_peak_error = 0.0;
      sum_control_ratio = 0.0;
      final_position_error = 0.0;
      final_velocity_error = 0.0;
      norm_denominator = 1.0;
      max_norm_error = 0.0;
      last_norm_error = 0.0;
      last_error_valid = false;
      oscillation_events = 0;
      last_violation_index = -1;
      control_period_s = 0.0;
      max_velocity_mag = 0.0;
      initial_velocity_sum = 0.0;
      initial_position_sum = 0.0;
      initial_sample_count = 0;
    }

    void begin(const ICNORLearningSample &sample) {
      active = true;
      target_position = sample.target_position;
      target_velocity = sample.target_velocity;
      start_distance = std::fabs(sample.distance_to_target);
      last_distance = start_distance;
      samples = 0;
      hi_samples = 0;
      saturated_samples = 0;
      failure_samples = 0;
      sum_hi_ratio = 0.0;
      sum_peak_error = 0.0;
      sum_control_ratio = 0.0;
      final_position_error = sample.position_error;
      final_velocity_error = sample.velocity_error;
      norm_denominator = std::max({std::fabs(target_position), start_distance,
          std::fabs(sample.position_error), 1e-3});
      max_norm_error = 0.0;
      last_norm_error = 0.0;
      last_error_valid = false;
      oscillation_events = 0;
      last_violation_index = -1;
      control_period_s = std::max(sample.control_period, 1e-6);
      max_velocity_mag = std::fabs(sample.state_velocity);
      initial_velocity_sum = 0.0;
      initial_position_sum = 0.0;
      initial_sample_count = 0;
    }

    bool targetChanged(const ICNORLearningSample &sample) const {
      constexpr double kTargetPosTol = 1e-3;
      constexpr double kTargetVelTol = 1e-3;
      return !active ||
             std::fabs(sample.target_position - target_position) > kTargetPosTol ||
             std::fabs(sample.target_velocity - target_velocity) > kTargetVelTol;
    }

    void accumulate(const ICNORLearningSample &sample) {
      if (!active) {
        begin(sample);
      }
      ++samples;
      final_position_error = sample.position_error;
      final_velocity_error = sample.velocity_error;
      last_distance = std::fabs(sample.distance_to_target);
      max_velocity_mag = std::max(max_velocity_mag, std::fabs(sample.state_velocity));

      double norm_error =
          sample.position_error / std::max(norm_denominator, 1e-3);
      double abs_norm_error = std::fabs(norm_error);
      max_norm_error = std::max(max_norm_error, abs_norm_error);

      constexpr double kOscAmp = 0.005;
      if (last_error_valid) {
        if (norm_error * last_norm_error < 0.0 &&
            std::max(std::fabs(norm_error), std::fabs(last_norm_error)) >= kOscAmp) {
          ++oscillation_events;
        }
      }
      last_norm_error = norm_error;
      last_error_valid = true;

      constexpr double kSettleTol = 0.02;
      if (abs_norm_error > kSettleTol) {
        last_violation_index = static_cast<int>(samples) - 1;
      }

      constexpr int kInitialWindow = 6;
      constexpr double kInitialDistThresh = 0.12;
      if (samples <= kInitialWindow &&
          std::fabs(sample.distance_to_target) <= kInitialDistThresh) {
        initial_velocity_sum += std::fabs(sample.state_velocity);
        initial_position_sum += sample.position_error;
        ++initial_sample_count;
      }

      if (sample.t_hi_init > 1e-6) {
        double ratio = sample.tstar / sample.t_hi_init;
        sum_hi_ratio += ratio;
        ++hi_samples;
      }
      saturated_samples += sample.saturated ? 1 : 0;
      failure_samples += sample.solved ? 0 : 1;

      double peak_error_norm =
          (sample.max_control_target - sample.v_max) /
          std::max(sample.v_max, 1.0);
      sum_peak_error += peak_error_norm;

      double control_ratio =
          sample.max_control_target / std::max(sample.sysvmax, 1.0);
      sum_control_ratio += control_ratio;
    }

    MotionMetrics finalize(bool completed) {
      MotionMetrics metrics;
      metrics.samples = samples;
      metrics.hi_samples = hi_samples;
      metrics.avg_hi_ratio =
          (hi_samples > 0) ? (sum_hi_ratio / static_cast<double>(hi_samples))
                           : 0.65;
      metrics.saturation_ratio =
          (samples > 0) ? static_cast<double>(saturated_samples) /
                              static_cast<double>(samples)
                        : 0.0;
      metrics.failure_ratio =
          (samples > 0) ? static_cast<double>(failure_samples) /
                              static_cast<double>(samples)
                        : 0.0;
      metrics.avg_peak_error =
          (samples > 0) ? (sum_peak_error / static_cast<double>(samples)) : 0.0;
      metrics.avg_control_ratio =
          (samples > 0) ? (sum_control_ratio / static_cast<double>(samples))
                        : 0.0;
      metrics.final_position_error = final_position_error;
      metrics.final_velocity_error = final_velocity_error;
      double travelled = std::max(0.0, start_distance - last_distance);
      metrics.distance_travelled = travelled;
      metrics.overshoot_ratio = max_norm_error;
      int settle_steps =
          (samples > 0) ? static_cast<int>(samples) - last_violation_index - 1 : 0;
      settle_steps = std::clamp(settle_steps, 0, static_cast<int>(samples));
      metrics.settled_fraction =
          (samples > 0) ? settle_steps / static_cast<double>(samples) : 0.0;
      metrics.settling_time_s =
          (last_violation_index < 0)
              ? 0.0
              : (last_violation_index + 1) * control_period_s;
      metrics.oscillation_events = oscillation_events;
      metrics.max_velocity = max_velocity_mag;
      metrics.initial_velocity_abs =
          (initial_sample_count > 0)
              ? (initial_velocity_sum / static_cast<double>(initial_sample_count))
              : std::fabs(final_velocity_error);
      metrics.initial_position_offset =
          (initial_sample_count > 0)
              ? (initial_position_sum / static_cast<double>(initial_sample_count))
              : final_position_error;
      metrics.completed = completed && samples > 0;
      reset();
      return metrics;
    }
  };

  void updateEMAs(const MotionMetrics &metrics);
  std::optional<icnor_internal::ICNORTuningParameters> deriveTuningLocked(
      const icnor_internal::ICNORTuningParameters &current_params);
  void waitForIO();
  bool applyNewtonAdjustment(double metric, double derivative_hint,
      double metric_scale, double &param, double min_val, double max_val,
      double max_step, double stability_factor,
      double confidence_floor) const;

  mutable std::mutex mutex_;
  std::string storage_path_;
  bool auto_save_ = true;
  size_t autosave_stride_ = 10;
  size_t updates_since_save_ = 0;
  bool dirty_ = false;
  mutable std::mutex io_mutex_;
  std::future<void> io_future_;

  size_t motion_count_ = 0;
  double ema_hi_ratio_ = 0.65;
  double ema_saturated_ratio_ = 0.0;
  double ema_failure_ratio_ = 0.0;
  double ema_peak_error_ = 0.0;
  double ema_pos_error_ = 0.0;
  double ema_vel_error_ = 0.0;
  double ema_control_ratio_ = 0.0;
  double ema_distance_travelled_ = 0.0;
  double ema_overshoot_ratio_ = 0.0;
  double ema_settled_fraction_ = 0.0;
  double ema_settling_time_s_ = 0.0;
  double ema_oscillation_events_ = 0.0;
  double ema_max_velocity_ = 0.0;
  double ema_initial_velocity_abs_ = 0.0;
  double ema_initial_position_offset_ = 0.0;

  double ema_time_scale_ = 1.0;
  double ema_time_offset_ = 0.0;
  double tuned_z_fudge_ = 1.0;
  double tuned_load_scale_ = 1.0;
  double tuned_load_bias_ = 0.0;
  double tuned_friction_scale_ = 1.0;

  MotionAccumulator motion_acc_;
};

inline ICNORLearner::ICNORLearner() : storage_path_("") {}

inline ICNORLearner::ICNORLearner(std::string storage_path)
    : storage_path_(ensureExtension(std::move(storage_path))) {
  if (!storage_path_.empty()) { loadAsync(); }
}

inline ICNORLearner::~ICNORLearner() {
  waitForIO();
}

inline std::string ICNORLearner::ensureExtension(std::string path) {
  constexpr const char *kExt = ".iclearn";
  if (path.empty()) return path;
  if (!endsWithInsensitive(path, kExt)) { path += kExt; }
  return path;
}

inline bool ICNORLearner::endsWithInsensitive(
    const std::string &value, const std::string &ending) {
  if (ending.size() > value.size()) return false;
  auto start = value.size() - ending.size();
  for (size_t i = 0; i < ending.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(value[start + i])) !=
        std::tolower(static_cast<unsigned char>(ending[i]))) {
      return false;
    }
  }
  return true;
}

inline void ICNORLearner::setStoragePath(const std::string &path) {
  std::lock_guard<std::mutex> lock(mutex_);
  storage_path_ = ensureExtension(path);
}

inline std::string ICNORLearner::storagePath() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return storage_path_;
}

inline void ICNORLearner::enableAutoSave(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto_save_ = enabled;
}

inline void ICNORLearner::setAutoSaveStride(size_t stride) {
  std::lock_guard<std::mutex> lock(mutex_);
  autosave_stride_ = std::max<size_t>(1, stride);
}

inline void ICNORLearner::updateEMAs(const MotionMetrics &metrics) {
  constexpr double alpha = 0.15;
  constexpr double one_minus_alpha = 1.0 - alpha;

  ema_hi_ratio_ =
      one_minus_alpha * ema_hi_ratio_ + alpha * metrics.avg_hi_ratio;
  ema_saturated_ratio_ =
      one_minus_alpha * ema_saturated_ratio_ + alpha * metrics.saturation_ratio;
  ema_failure_ratio_ =
      one_minus_alpha * ema_failure_ratio_ + alpha * metrics.failure_ratio;
  ema_peak_error_ =
      one_minus_alpha * ema_peak_error_ + alpha * metrics.avg_peak_error;
  ema_pos_error_ =
      one_minus_alpha * ema_pos_error_ + alpha * metrics.final_position_error;
  ema_vel_error_ =
      one_minus_alpha * ema_vel_error_ + alpha * metrics.final_velocity_error;
  ema_control_ratio_ =
      one_minus_alpha * ema_control_ratio_ + alpha * metrics.avg_control_ratio;
  ema_distance_travelled_ = one_minus_alpha * ema_distance_travelled_ +
                            alpha * metrics.distance_travelled;
  ema_overshoot_ratio_ =
      one_minus_alpha * ema_overshoot_ratio_ + alpha * metrics.overshoot_ratio;
  ema_settled_fraction_ =
      one_minus_alpha * ema_settled_fraction_ + alpha * metrics.settled_fraction;
  ema_settling_time_s_ =
      one_minus_alpha * ema_settling_time_s_ + alpha * metrics.settling_time_s;
  ema_oscillation_events_ =
      one_minus_alpha * ema_oscillation_events_ +
      alpha * static_cast<double>(metrics.oscillation_events);
  ema_max_velocity_ =
      one_minus_alpha * ema_max_velocity_ + alpha * metrics.max_velocity;
  ema_initial_velocity_abs_ =
      one_minus_alpha * ema_initial_velocity_abs_ + alpha * metrics.initial_velocity_abs;
  ema_initial_position_offset_ =
      one_minus_alpha * ema_initial_position_offset_ + alpha * metrics.initial_position_offset;
  ++motion_count_;
}

inline std::optional<icnor_internal::ICNORTuningParameters>
ICNORLearner::deriveTuningLocked(
    const icnor_internal::ICNORTuningParameters &current_params) {
  constexpr size_t kMinMotionsForTuning = 3;
  if (motion_count_ < kMinMotionsForTuning) { return std::nullopt; }

  auto params = current_params;
  bool changed = false;

  double desired_ratio = 0.65;
  double overshoot_target = 0.02;
  double overshoot_metric = ema_overshoot_ratio_ - overshoot_target;
  double oscillation_metric =
      std::clamp(ema_oscillation_events_ - 1.0, -1.5, 3.0);
  double settle_gap =
      std::clamp(0.85 - ema_settled_fraction_, -0.8, 0.8);
  double settling_time_metric =
      std::clamp(ema_settling_time_s_ - 0.15, -0.3, 0.6);

  double time_scale_metric =
      (ema_hi_ratio_ - desired_ratio) +
      0.4 * overshoot_metric +
      0.2 * oscillation_metric;
  double time_scale_stability =
      std::clamp(1.0 - 0.5 * ema_saturated_ratio_, 0.1, 1.0) *
      std::clamp(1.0 - ema_failure_ratio_, 0.0, 1.0) *
      std::clamp(ema_settled_fraction_ + 0.1, 0.0, 1.0);
  if (applyNewtonAdjustment(time_scale_metric, 0.8, 0.04,
          params.time_scale_factor, 0.4, 2.6, 0.25, time_scale_stability, 0.05)) {
    changed = true;
  }

  double desired_failure = 0.05;
  double offset_metric =
      (ema_failure_ratio_ - desired_failure) +
      0.3 * settle_gap +
      0.2 * settling_time_metric;
  double offset_stability =
      std::clamp(1.0 - 0.3 * ema_saturated_ratio_, 0.0, 1.0) *
      std::clamp(1.0 - ema_failure_ratio_, 0.0, 1.0) *
      std::clamp(ema_settled_fraction_ + 0.05, 0.0, 1.0);
  if (applyNewtonAdjustment(offset_metric, -0.6, 0.04,
          params.time_offset, -0.5, 1.5, 0.25, offset_stability, 0.05)) {
    changed = true;
  }

  double z_confidence =
      std::clamp(ema_distance_travelled_ / 0.05, 0.0, 1.0) *
      std::clamp(1.0 - ema_failure_ratio_, 0.0, 1.0) *
      std::clamp(ema_settled_fraction_ + 0.1, 0.0, 1.0);
  double z_metric =
      0.5 * ema_peak_error_ +
      0.4 * overshoot_metric +
      0.2 * oscillation_metric;
  if (applyNewtonAdjustment(z_metric,
          std::max(0.2, ema_control_ratio_ + 0.2), 0.15, params.z_fudge, 0.3,
          3.0, 0.3, z_confidence, 0.05)) {
    changed = true;
  }

  double desired_control_ratio = 0.6;
  double velocity_term =
      std::clamp(ema_vel_error_ / std::max(0.02, ema_distance_travelled_ + 1e-3),
          -1.5, 1.5);
  double load_scale_confidence =
      std::clamp(ema_distance_travelled_ / 0.03, 0.0, 1.0) *
      std::clamp(1.0 - ema_failure_ratio_, 0.0, 1.0) *
      std::clamp(ema_settled_fraction_ + 0.03, 0.0, 1.0);
  double still_velocity_term =
      std::clamp((ema_initial_velocity_abs_ - 0.012) / 0.025, -1.5, 1.5);
  double load_scale_metric =
      (ema_control_ratio_ - desired_control_ratio) +
      0.4 * velocity_term +
      0.3 * overshoot_metric +
      0.35 * std::max(still_velocity_term, 0.0);
  if (applyNewtonAdjustment(load_scale_metric, -0.9, 0.035, params.load_scale,
          0.5, 2.0, 0.35, load_scale_confidence, 0.035)) {
    changed = true;
  }

  double load_bias_confidence =
      std::clamp(ema_distance_travelled_ / 0.02, 0.0, 1.0) *
      std::clamp(1.0 - ema_failure_ratio_, 0.0, 1.0) *
      std::clamp(ema_settled_fraction_ + 0.03, 0.0, 1.0);
  double distance_scale =
      std::max(ema_distance_travelled_, 0.01);
  double still_offset_term =
      std::clamp(ema_initial_position_offset_ / 0.01, -2.0, 2.0);
  double load_bias_metric =
      (ema_pos_error_ / distance_scale) +
      0.25 * settling_time_metric +
      0.35 * still_offset_term;
  if (applyNewtonAdjustment(load_bias_metric, -1.0, 0.01, params.load_bias,
          -5.0, 5.0, 0.35, load_bias_confidence, 0.035)) {
    changed = true;
  }

  double friction_confidence =
      std::clamp(ema_distance_travelled_ / 0.02, 0.0, 1.0) *
      std::clamp(1.0 - ema_failure_ratio_, 0.0, 1.0) *
      std::clamp(ema_settled_fraction_ + 0.15, 0.0, 1.0);
  double friction_metric =
      std::clamp(ema_initial_velocity_abs_ - 0.02, -0.5, 0.5);
  if (applyNewtonAdjustment(friction_metric, -0.6, 0.01, params.friction_scale,
          0.5, 2.0, 0.15, friction_confidence, 0.02)) {
    changed = true;
  }

  params.time_scale_factor =
      std::clamp(params.time_scale_factor, 0.4, 2.6);
  params.time_offset = std::clamp(params.time_offset, -0.5, 1.5);
  params.z_fudge = std::clamp(params.z_fudge, 0.3, 3.0);
  params.load_scale = std::clamp(params.load_scale, 0.5, 2.0);
  params.load_bias = std::clamp(params.load_bias, -5.0, 5.0);
  params.friction_scale = std::clamp(params.friction_scale, 0.5, 2.0);

  ema_time_scale_ = 0.9 * ema_time_scale_ + 0.1 * params.time_scale_factor;
  ema_time_offset_ = 0.9 * ema_time_offset_ + 0.1 * params.time_offset;
  tuned_z_fudge_ = params.z_fudge;
  tuned_load_scale_ = params.load_scale;
  tuned_load_bias_ = params.load_bias;
  tuned_friction_scale_ = params.friction_scale;

  if (!changed) return std::nullopt;
  return params;
}

inline bool ICNORLearner::applyNewtonAdjustment(double metric,
    double derivative_hint, double metric_scale, double &param,
    double min_val, double max_val, double max_step, double stability_factor,
    double confidence_floor) const {
  if (motion_count_ < 3) return false;
  if (!std::isfinite(metric) || !std::isfinite(derivative_hint)) return false;
  double bounded_metric_scale = std::max(metric_scale, 1e-6);
  double motion_factor =
      1.0 - std::exp(-static_cast<double>(motion_count_) / 20.0);
  double magnitude_factor =
      1.0 - std::exp(-std::fabs(metric) / bounded_metric_scale);
  double stability = std::clamp(stability_factor, 0.0, 1.0);
  double confidence =
      std::clamp(motion_factor * magnitude_factor * stability, 0.0, 1.0);
  if (confidence < confidence_floor) return false;

  double derivative =
      (std::fabs(derivative_hint) < 1e-6)
          ? (derivative_hint >= 0.0 ? 1e-6 : -1e-6)
          : derivative_hint;
  double raw_delta = -metric / derivative;
  if (!std::isfinite(raw_delta)) return false;
  raw_delta = std::clamp(raw_delta, -max_step, max_step);
  double adjusted_delta = raw_delta * confidence;
  double new_value = std::clamp(param + adjusted_delta, min_val, max_val);
  if (!std::isfinite(new_value)) return false;
  if (std::fabs(new_value - param) < 1e-6) return false;
  param = new_value;
  return true;
}

inline void ICNORLearner::waitForIO() {
  std::future<void> worker;
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (io_future_.valid()) {
      worker = std::move(io_future_);
    }
  }
  if (worker.valid()) {
    worker.get();
  }
}

inline void ICNORLearner::loadAsync() {
  std::string path;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (storage_path_.empty()) return;
    path = storage_path_;
  }

  waitForIO();

  auto task = [this, path]() {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
      return;
    }

    std::string contents((std::istreambuf_iterator<char>(in)),
        std::istreambuf_iterator<char>());

    size_t motion_count = 0;
    double ema_hi_ratio = 0.65, ema_saturated_ratio = 0.0, ema_failure_ratio = 0.0;
    double ema_time_scale = 1.0, ema_time_offset = 0.0;
    double ema_peak_error = 0.0;
    double ema_pos_error = 0.0;
    double ema_vel_error = 0.0;
    double ema_control_ratio = 0.0;
    double ema_distance_travelled = 0.0;
    double ema_overshoot = 0.0;
    double ema_settled_fraction = 0.0;
    double ema_settling_time = 0.0;
    double ema_osc_events = 0.0;
    double ema_max_velocity = 0.0;
    double ema_initial_velocity_abs = 0.0;
    double ema_initial_position_offset = 0.0;
    double tuned_z = 1.0;
    double tuned_load_scale = 1.0;
    double tuned_load_bias = 0.0;
    double tuned_friction_scale = 1.0;

    if (!contents.empty()) {
      std::istringstream numeric(contents);
      std::vector<double> values;
      double value = 0.0;
      while (numeric >> value) {
        values.push_back(value);
      }

      if (values.size() >= 6 && values.size() < 14) {
        ema_time_scale = values[0];
        ema_time_offset = values[1];
        tuned_z = values[2];
        tuned_load_scale = values[3];
        tuned_load_bias = values[4];
        tuned_friction_scale = values[5];
      } else if (values.size() == 5) {
        ema_time_scale = values[0];
        ema_time_offset = values[1];
        tuned_z = values[2];
        tuned_load_scale = values[3];
        tuned_load_bias = values[4];
      } else if (values.size() >= 19) {
        // Legacy dense numeric format.
        motion_count = static_cast<size_t>(std::max(0.0, values[0]));
        ema_hi_ratio = values[1];
        ema_saturated_ratio = values[2];
        ema_failure_ratio = values[3];
        ema_time_scale = values[4];
        ema_time_offset = values[5];
        ema_peak_error = values[6];
        ema_pos_error = values[7];
        ema_vel_error = values[8];
        ema_control_ratio = values[9];
        ema_distance_travelled = values[10];
        ema_overshoot = values[11];
        ema_settled_fraction = values[12];
        ema_settling_time = values[13];
        ema_osc_events = values[14];
        ema_max_velocity = values[15];
        tuned_z = values[16];
        tuned_load_scale = values[17];
        tuned_load_bias = values[18];
      } else if (values.size() >= 14 && values.size() < 19) {
        motion_count = static_cast<size_t>(std::max(0.0, values[0]));
        ema_hi_ratio = values[1];
        ema_saturated_ratio = values[2];
        ema_failure_ratio = values[3];
        ema_time_scale = values[4];
        ema_time_offset = values[5];
        ema_peak_error = values[6];
        ema_pos_error = values[7];
        ema_vel_error = values[8];
        ema_control_ratio = values[9];
        ema_distance_travelled = values[10];
        tuned_z = values[11];
        tuned_load_scale = values[12];
        tuned_load_bias = values[13];
      } else {
        std::istringstream legacy(contents);
        std::string line;
        while (std::getline(legacy, line)) {
          if (line.empty() || line[0] == '#') continue;
          size_t eq = line.find('=');
          if (eq == std::string::npos) continue;
          std::string key = line.substr(0, eq);
          std::string val = line.substr(eq + 1);
          try {
            if (key == "sample_count") motion_count = std::stoull(val);
            else if (key == "ema_hi_ratio") ema_hi_ratio = std::stod(val);
            else if (key == "ema_saturated_ratio")
              ema_saturated_ratio = std::stod(val);
            else if (key == "ema_failure_ratio")
              ema_failure_ratio = std::stod(val);
            else if (key == "ema_time_scale") ema_time_scale = std::stod(val);
            else if (key == "ema_time_offset") ema_time_offset = std::stod(val);
            else if (key == "ema_peak_error")
              ema_peak_error = std::stod(val);
            else if (key == "ema_pos_error")
              ema_pos_error = std::stod(val);
            else if (key == "ema_vel_error")
              ema_vel_error = std::stod(val);
            else if (key == "ema_ff_ratio")
              ema_control_ratio = std::stod(val);
            else if (key == "ema_overshoot_ratio")
              ema_overshoot = std::stod(val);
            else if (key == "ema_settled_fraction")
              ema_settled_fraction = std::stod(val);
            else if (key == "ema_settling_time_s")
              ema_settling_time = std::stod(val);
            else if (key == "ema_oscillation_events")
              ema_osc_events = std::stod(val);
            else if (key == "ema_max_velocity")
              ema_max_velocity = std::stod(val);
            else if (key == "tuned_z_fudge")
              tuned_z = std::stod(val);
            else if (key == "tuned_load_scale")
              tuned_load_scale = std::stod(val);
            else if (key == "tuned_load_bias")
              tuned_load_bias = std::stod(val);
          } catch (...) {
            continue;
          }
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      motion_count_ = motion_count;
      ema_hi_ratio_ = ema_hi_ratio;
      ema_saturated_ratio_ = ema_saturated_ratio;
      ema_failure_ratio_ = ema_failure_ratio;
      ema_time_scale_ = ema_time_scale;
      ema_time_offset_ = ema_time_offset;
      ema_peak_error_ = ema_peak_error;
      ema_pos_error_ = ema_pos_error;
      ema_vel_error_ = ema_vel_error;
      ema_control_ratio_ = ema_control_ratio;
      ema_distance_travelled_ = ema_distance_travelled;
      ema_overshoot_ratio_ = ema_overshoot;
      ema_settled_fraction_ = ema_settled_fraction;
      ema_settling_time_s_ = ema_settling_time;
      ema_oscillation_events_ = ema_osc_events;
      ema_max_velocity_ = ema_max_velocity;
      ema_initial_velocity_abs_ = ema_initial_velocity_abs;
      ema_initial_position_offset_ = ema_initial_position_offset;
      tuned_z_fudge_ = std::clamp(tuned_z, 0.3, 3.0);
      tuned_load_scale_ = std::clamp(tuned_load_scale, 0.5, 2.0);
      tuned_load_bias_ = std::clamp(tuned_load_bias, -5.0, 5.0);
      tuned_friction_scale_ = std::clamp(tuned_friction_scale, 0.5, 2.0);
      dirty_ = false;
      updates_since_save_ = 0;
    }
  };

  {
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    io_future_ = std::async(std::launch::async, std::move(task));
  }
}

inline void ICNORLearner::saveAsync() {
  waitForIO();

  std::string path;
  std::string data;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (storage_path_.empty()) return;
    path = storage_path_;
    std::ostringstream oss;
    oss << std::setprecision(17)
        << ema_time_scale_ << ' '
        << ema_time_offset_ << ' '
        << tuned_z_fudge_ << ' '
        << tuned_load_scale_ << ' '
        << tuned_load_bias_ << ' '
        << tuned_friction_scale_ << '\n';
    data = oss.str();
  }

  auto task = [this, path, data]() {
    std::ofstream out(path, std::ios::trunc);
    if (out.is_open()) {
      out << data;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      dirty_ = false;
      updates_since_save_ = 0;
    }
  };

  {
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    io_future_ = std::async(std::launch::async, std::move(task));
  }
}

inline bool ICNORLearner::saveIfDirty() {
  bool should_save = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    should_save = dirty_;
  }
  if (should_save) {
    saveAsync();
  }
  waitForIO();
  return should_save;
}

inline ICNORLearner::UpdateResult ICNORLearner::notifyOptimizationResult(
    const ICNORLearningSample &sample,
    const icnor_internal::ICNORTuningParameters &current_params) {
  UpdateResult result;
  bool should_save_now = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    constexpr size_t kMinSamplesPerMotion = 2;
    constexpr double kMinTravelDistance = 0.005;

    if (motion_acc_.targetChanged(sample)) {
      if (motion_acc_.active && motion_acc_.samples > 0) {
        motion_acc_.finalize(false);
      }
      motion_acc_.begin(sample);
    }

    motion_acc_.accumulate(sample);

    bool ready_to_finalize =
        sample.solved && !sample.saturated &&
        motion_acc_.samples >= kMinSamplesPerMotion;

    if (ready_to_finalize) {
      MotionMetrics metrics = motion_acc_.finalize(true);
      bool sufficient_history =
          metrics.samples >= kMinSamplesPerMotion &&
          metrics.distance_travelled >= kMinTravelDistance;
      bool qualitative_ok = metrics.completed && sufficient_history;

      if (qualitative_ok) {
        updateEMAs(metrics);
        dirty_ = true;
        ++updates_since_save_;
        result.new_tuning = deriveTuningLocked(current_params);
        if (auto_save_ && autosave_stride_ > 0 &&
            updates_since_save_ >= autosave_stride_) {
          should_save_now = true;
        }
      }
    }
  }
  if (should_save_now) {
    saveAsync();
  }
  result.should_save = should_save_now;
  return result;
}

inline std::optional<icnor_internal::ICNORTuningParameters>
ICNORLearner::suggestFromHistory(
    const icnor_internal::ICNORTuningParameters &current_params) {
  std::lock_guard<std::mutex> lock(mutex_);
  return deriveTuningLocked(current_params);
}

inline icnor_internal::ICNORTuningParameters 
ICNORLearner::getCurrentTuning() const {
  std::lock_guard<std::mutex> lock(mutex_);
  icnor_internal::ICNORTuningParameters params;
  params.time_scale_factor = ema_time_scale_;
  params.time_offset = ema_time_offset_;
  params.z_fudge = tuned_z_fudge_;
  params.load_scale = tuned_load_scale_;
  params.load_bias = tuned_load_bias_;
  params.friction_scale = tuned_friction_scale_;
  return params;
}

}  // namespace pdcsu::control

namespace pdcsu::control::icnor_internal {

using ::pdcsu::control::ICNORLearner;
using ::pdcsu::control::ICNORLearningSample;

class ICNOR {
private:
  double Z;  // Z = τ_max / (J * w_f)

  double x0, v0;
  double T, P;

  double v_max;
  double sysvmax;

  double control_period;

  double tau_max_nominal_;
  double inertia_nominal_;
  double free_speed_nominal_;

  double z_fudge_ = 1.0;
  double load_scale_ = 1.0;
  double load_bias_ = 0.0;
  double friction_scale_ = 1.0;
  int coarse_steps_ = 8;
  int fine_steps_ = 10;
  double beta_range_ = 80000.0;
  double max_allow_margin_ = 2.0;

  // Solver constants
  double t_lo_init = 1e-6, t_hi_init = 4.0;
  const int time_bisect_iters = 7;
  const double tolerance = 1e-7;

  // Precomputed constants
  double invZ, invZ2, invZ3, invZ4;

  // Control parameters
  double tstar = 0.0, zeta = 0.0, alpha = 0.0, beta = 0.0, gamma = 0.0;
  double alphaS = 0.0, betaS = 0.0, gammaS = 0.0;

  ICNORTuningParameters tuning_params_;
  std::weak_ptr<ICNORLearner> learner_;
  bool learner_tuning_applied_ = false;

  double last_t_hi_init_ = 0.0;
  double last_t_lo_init_ = 0.0;
  double last_distance_to_target_ = 0.0;
  double last_control_peak_ = 0.0;
  double last_beta_abs_ = 0.0;
  double last_gamma_abs_ = 0.0;
  bool last_saturated_ = false;
  bool last_feasible_ = true;

public:
private:
  inline void findZ_and_inverses(double tau_max, double J, double w_f) {
    if (tau_max <= 0.0 || J <= 0.0 || w_f <= 0.0) {
      std::ostringstream oss;
      oss << "Invalid parameters for findZ: tau_max=" << tau_max
          << " J=" << J << " w_f=" << w_f;
      throw std::runtime_error(oss.str());
    }
    Z = tau_max / (J * w_f);
    invZ = 1.0 / Z;
    invZ2 = invZ * invZ;
    invZ3 = invZ2 * invZ;
    invZ4 = invZ3 * invZ;
  }

  inline void recomputeZ() {
    findZ_and_inverses(tau_max_nominal_ * z_fudge_, inertia_nominal_,
        free_speed_nominal_);
  }

  struct ControlEvalContext {
    double t;
    double t2;
    double t3;
    double tmid;
    double tmid2;
    double tmid3;
    double max_allow;
  };

  inline ControlEvalContext make_control_eval_context(double t,
      double max_allow) const {
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double tmid = 0.5 * t;
    const double tmid2 = tmid * tmid;
    return ControlEvalContext{
        t, t2, t3, tmid, tmid2, tmid2 * tmid, max_allow};
  }

  inline void updateTimingBounds() {
    last_distance_to_target_ = std::fabs(T - x0);
    const double denom = std::max(v_max, 1e-6);
    double base_hi =
        std::min(3.0, 1.35 * (last_distance_to_target_ / denom) + 0.35);
    double tuned_hi =
        base_hi * tuning_params_.time_scale_factor + tuning_params_.time_offset;
    tuned_hi = std::clamp(tuned_hi, 0.05, 4.0);
    t_hi_init = tuned_hi;
    t_lo_init = std::max(-0.5, 0.5 * t_hi_init - 0.5);
    last_t_hi_init_ = t_hi_init;
    last_t_lo_init_ = t_lo_init;
  }

  inline ICNORLearningSample buildLearningSample(bool solved) const {
    ICNORLearningSample sample;
    const auto now = std::chrono::system_clock::now();
    sample.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              now.time_since_epoch())
                              .count();
    sample.target_position = T;
    sample.target_velocity = P;
    sample.state_position = x0;
    sample.state_velocity = v0;
    sample.tstar = tstar;
    sample.zeta = zeta;
    sample.alpha = alphaS;
    sample.beta = betaS;
    sample.gamma = gammaS;
    sample.v_max = v_max;
    sample.sysvmax = sysvmax;
    sample.control_period = control_period;
    sample.max_control_target = last_control_peak_;
    sample.position_error = T - x0;
    sample.velocity_error = P - v0;
    sample.beta_abs = last_beta_abs_;
    sample.gamma_abs = last_gamma_abs_;
    sample.t_hi_init = last_t_hi_init_;
    sample.t_lo_init = last_t_lo_init_;
    sample.distance_to_target =
        (last_distance_to_target_ > 0.0) ? last_distance_to_target_
                                         : std::fabs(T - x0);
    sample.saturated = last_saturated_;
    sample.solved = solved && last_feasible_;
    return sample;
  }

  inline void notifyLearner(bool solved) {
    auto learner = learner_.lock();
    if (!learner) return;
    ICNORLearningSample sample = buildLearningSample(solved);
    auto result = learner->notifyOptimizationResult(sample, tuning_params_);
  if (result.new_tuning) {
    applyTuningParameters(*result.new_tuning);
  }
    if (result.should_save) {
      learner->saveAsync();
    }
  }

  // Computes the maximum control target ICNOR attempts to apply
  inline double max_control_target(const ControlEvalContext &ctx,
      double zeta_val, double alpha_val, double beta_val,
      double gamma_val) const {
    double vmax_ = fabs(zeta_val);
    if (vmax_ > ctx.max_allow) return vmax_;

    double end_val =
        fabs(zeta_val + alpha_val * ctx.t + beta_val * ctx.t2 +
            gamma_val * ctx.t3);
    if (end_val > ctx.max_allow) return end_val;
    vmax_ = std::max(vmax_, end_val);

    double mid_val = fabs(zeta_val + alpha_val * ctx.tmid +
                          beta_val * ctx.tmid2 + gamma_val * ctx.tmid3);
    if (mid_val > ctx.max_allow) return mid_val;
    vmax_ = std::max(vmax_, mid_val);

    return vmax_;
  }

  struct SolveContext {
    double M00;
    double M01;
    double M10;
    double M11;
    double det;
    double rhs0_const;
    double rhs0_beta;
    double rhs0_gamma;
    double rhs1_const;
    double rhs1_beta;
    double rhs1_gamma;
  };

  struct SolverOutput {
    double zeta;
    double alpha;
    double beta;
    double gamma;
    double peak;
    double beta_abs;
    double gamma_abs;
    bool saturated;
  };

  inline SolveContext make_solve_context(double t) const {
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double K = exp(-Z * t);
    const double one_minus_K = 1.0 - K;

    SolveContext ctx{};
    ctx.M00 = Z * t - 1.0 + K;
    ctx.M01 = 0.5 * Z * t2 - t + one_minus_K * invZ;
    ctx.M10 = one_minus_K;
    ctx.M11 = t + invZ * (K - 1.0);
    ctx.det = ctx.M00 * ctx.M11 - ctx.M01 * ctx.M10;

    ctx.rhs0_const = Z * (T - x0) - v0 * one_minus_K;
    ctx.rhs0_beta =
        (t3 * Z / 3.0) + 2.0 * t2 + 2.0 * t * invZ + 2.0 * invZ2 * one_minus_K;
    ctx.rhs0_gamma = (-Z * t4 / 4.0) - t3 + 3.0 * t2 * invZ +
                     6.0 * t * invZ2 + 6.0 * invZ3 * one_minus_K;

    ctx.rhs1_const = P - v0 * K;
    ctx.rhs1_beta =
        -t2 - 2.0 * t * invZ - 2.0 * invZ2 + 2.0 * K * invZ2;
    ctx.rhs1_gamma = -t3 - 3.0 * t2 * invZ - 6.0 * t * invZ2 +
                     6.0 * invZ3 - 6.0 * K * invZ3;
    return ctx;
  }

  // Solves for zeta and alpha, given beta and gamma
  inline std::optional<std::pair<double, double>> solve_zeta_alpha(
      double beta, double gamma, const SolveContext &ctx) {
    if (fabs(ctx.det) < tolerance) return std::nullopt;

    double rhs0 =
        ctx.rhs0_const + beta * ctx.rhs0_beta + gamma * ctx.rhs0_gamma;
    double rhs1 =
        ctx.rhs1_const + beta * ctx.rhs1_beta + gamma * ctx.rhs1_gamma;

    zeta = (rhs0 * ctx.M11 - ctx.M01 * rhs1) / ctx.det;
    alpha = (ctx.M00 * rhs1 - rhs0 * ctx.M10) / ctx.det;
    return std::make_pair(zeta, alpha);
  }

  // Solves the system, if possible in the specified time
  inline std::optional<SolverOutput> solve_if_feasible(double t) {
    std::optional<SolverOutput> best_sol;

    const int coarse_steps = std::max(4, coarse_steps_);
    const int fine_steps = std::max(4, fine_steps_);
    const double b_range = std::max(1000.0, beta_range_);
    double best_b_real = 0.0;
    double min_b_abs = std::numeric_limits<double>::max();
    double best_peak = std::numeric_limits<double>::infinity();

    const SolveContext ctx = make_solve_context(t);
    if (fabs(ctx.det) < tolerance) return std::nullopt;

    const double denom = std::max(v_max, 1e-6);
    const double coarse_sign =
        -std::copysign(1.0, T - x0 - 10.0 * v0 / denom);
    const double max_allow = v_max + max_allow_margin_;
    const ControlEvalContext control_ctx =
        make_control_eval_context(t, max_allow);

    constexpr double kBestAbsGoal = 1e-6;

    auto try_candidate = [&](double b_real, double g_real, double z,
                             double a) {
      double candidate_peak =
          max_control_target(control_ctx, z, a, b_real, g_real);
      if (candidate_peak > max_allow) return false;
      double b_abs = std::fabs(b_real);
      bool better =
          (b_abs < min_b_abs - 1e-15) ||
          (std::fabs(b_abs - min_b_abs) <= 1e-12 &&
              candidate_peak < best_peak - 1e-12);
      if (better) {
        min_b_abs = b_abs;
        best_peak = candidate_peak;
        best_b_real = b_real;
        best_sol = SolverOutput{z, a, b_real, g_real, candidate_peak, b_abs,
            std::fabs(g_real), candidate_peak >= v_max};
        return true;
      }
      return false;
    };

    bool coarse_done = false;
    for (int i = 0; i <= coarse_steps && !coarse_done; ++i) {
      double b = b_range * i / coarse_steps;
      double g = 0.5 * b;
      double b_real = b * coarse_sign;
      double g_real = g * coarse_sign;
      auto sol = solve_zeta_alpha(b_real, g_real, ctx);
      if (!sol) continue;
      auto [z, a] = *sol;
      if (try_candidate(b_real, g_real, z, a)) {
        if (min_b_abs <= kBestAbsGoal && best_sol) { return best_sol; }
        if (std::fabs(std::fabs(b_real) - b) <= 1e-12) {
          coarse_done = true;
        }
      }
    }

    if (min_b_abs < std::numeric_limits<double>::max()) {
      double fine_b_range = b_range / std::max(1, coarse_steps);
      const double fine_sign = -std::copysign(1.0, T - x0);
      const double base_b = (fine_sign == 0.0) ? 0.0 : best_b_real / fine_sign;
      const double fine_step =
          fine_steps > 0 ? fine_b_range / fine_steps : fine_b_range;
      int max_offset = fine_steps;
      double base_abs = std::fabs(base_b);
      double search_limit = base_abs + min_b_abs;
      if (fine_step > 0.0) {
        max_offset = std::min(fine_steps,
            static_cast<int>(std::ceil(search_limit / fine_step)));
      }
      bool finished = false;
      for (int i = -max_offset; i <= max_offset && !finished; ++i) {
        if (i != 0 && fine_step > 0.0) {
          double delta = std::fabs(i * fine_step);
          double min_possible =
              std::fabs(std::fabs(base_b) - delta);
          if (min_possible >= min_b_abs - 1e-15) {
            continue;
          }
        }
        const int sign_count = (i == 0) ? 1 : 2;
        for (int idx = 0; idx < sign_count && !finished; ++idx) {
          int sign = (idx == 0) ? 1 : -1;
          double b = base_b + sign * i * fine_step;
          double g = 0.5 * b;
          double b_real = b * fine_sign;
          double g_real = g * fine_sign;
          auto sol = solve_zeta_alpha(b_real, g_real, ctx);
          if (!sol) continue;
          auto [z, a] = *sol;
          if (try_candidate(b_real, g_real, z, a) &&
              min_b_abs <= kBestAbsGoal && best_sol) {
            finished = true;
            break;
          }
        }
      }
    }

    return best_sol;
  }

public:
  std::tuple<double, double, double, double, double> optimize() {
    if (!learner_tuning_applied_) {
      auto learner = learner_.lock();
      if (learner && !learner->storagePath().empty()) {
        auto tuning = learner->getCurrentTuning();
        applyTuningParameters(tuning);
      }
      learner_tuning_applied_ = true;
    }
    
    double t_lo = t_lo_init, t_hi = t_hi_init;

    auto feas_hi = solve_if_feasible(t_hi);
    if (!feas_hi) {
      if (T > x0) {
        this->zeta = v_max;
      } else {
        this->zeta = -v_max;
      }
      this->alphaS = this->betaS = this->gammaS = 0.0;
      this->tstar = 1000.0;
      last_feasible_ = false;
      last_control_peak_ = std::fabs(this->zeta);
      last_beta_abs_ = 0.0;
      last_gamma_abs_ = 0.0;
      last_saturated_ = true;
      notifyLearner(false);
      return std::make_tuple(
          1000.0, this->zeta, this->alphaS, this->betaS, this->gammaS);
    }

    SolverOutput best_output = *feas_hi;
    double best_time = t_hi;
    last_feasible_ = true;

    for (int i = 0; i < time_bisect_iters; i++) {
      double t_mid = 0.5 * (t_lo + t_hi);
      auto feas_mid = solve_if_feasible(t_mid);
      if (feas_mid) {
        t_hi = t_mid;
        best_output = *feas_mid;
        best_time = t_mid;
      } else {
        t_lo = t_mid;
      }
    }
    this->tstar = best_time;
    this->zeta = best_output.zeta;
    this->alpha = best_output.alpha;
    this->beta = best_output.beta;
    this->gamma = best_output.gamma;
    this->alphaS = best_output.alpha;
    this->betaS = best_output.beta;
    this->gammaS = best_output.gamma;
    last_control_peak_ = best_output.peak;
    last_beta_abs_ = best_output.beta_abs;
    last_gamma_abs_ = best_output.gamma_abs;
    last_saturated_ = best_output.saturated;
    notifyLearner(true);
    return std::make_tuple(
        this->tstar, this->zeta, this->alphaS, this->betaS, this->gammaS);
  }

  void attachLearner(const std::shared_ptr<ICNORLearner> &learner) {
    learner_ = learner;
  }

  void applyTuningParameters(const ICNORTuningParameters &params) {
    ICNORTuningParameters tuned = params;
    tuned.time_scale_factor =
        std::clamp(tuned.time_scale_factor, 0.25, 3.0);
    tuned.time_offset = std::clamp(tuned.time_offset, -0.5, 1.5);
    tuned.z_fudge = std::clamp(tuned.z_fudge, 0.25, 3.0);
    tuned.load_scale = std::clamp(tuned.load_scale, 0.5, 2.0);
    tuned.load_bias = std::clamp(tuned.load_bias, -5.0, 5.0);
    tuned.friction_scale = std::clamp(tuned.friction_scale, 0.5, 2.0);
    tuning_params_ = tuned;
    z_fudge_ = tuning_params_.z_fudge;
    load_scale_ = tuning_params_.load_scale;
    load_bias_ = tuning_params_.load_bias;
    friction_scale_ = tuning_params_.friction_scale;
    recomputeZ();
    updateTimingBounds();
  }

  const ICNORTuningParameters &getTuningParameters() const {
    return tuning_params_;
  }

  ICNOR(BasePlant def_sys, radps_t v_max)
      : x0(0.0),
        v0(0.0),
        T(0.0),
        P(0.0),
        v_max(v_max.value()),
        sysvmax(radps_t(def_sys.def_bldc.free_speed).value()),
        control_period(second_t(def_sys.control_period).value()),
        tau_max_nominal_(def_sys.def_bldc.stall_torque.value()),
        inertia_nominal_(def_sys.inertia.value()),
        free_speed_nominal_(radps_t(def_sys.def_bldc.free_speed).value()) {
    recomputeZ();
    zeta = alphaS = betaS = gammaS = 0.0;
    updateTimingBounds();
  }

  void setTarget(radian_t T, radps_t P) {
    this->T = T.value();
    this->P = P.value();
    updateTimingBounds();
  }

  void setState(radian_t x0, radps_t v0) {
    this->x0 = x0.value();
    this->v0 = v0.value();
    updateTimingBounds();
  }

  void setTargetAndState(radian_t T, radps_t P, radian_t x0, radps_t v0) {
    setTarget(T, P);
    setState(x0, v0);
  }

  double getImmediateOutput() { return getProjectedOutput(1)[0]; }

  double computeAverageUkOverInterval(double t, double dt) {
    auto integral_poly = [&](double a, double b) {
      double b2 = b * b, a2 = a * a;
      double b3 = b2 * b, a3 = a2 * a;
      double b4 = b3 * b, a4 = a3 * a;
      return zeta * (b - a) + 0.5 * alphaS * (b2 - a2) +
             (1.0 / 3.0) * betaS * (b3 - a3) + 0.25 * gammaS * (b4 - a4);
    };

    double t0 = t;
    double t1 = t + dt;

    if (t1 <= tstar) {
      double integral = integral_poly(t0, t1);
      double avg = integral / dt;
      return avg / sysvmax;
    }

    if (t0 >= tstar) { return (P / sysvmax); }

    double integral_before = integral_poly(t0, tstar);
    double integral_after = P * (t1 - tstar);
    double integral_total = integral_before + integral_after;
    double avg = integral_total / dt;
    return avg / sysvmax;
  }

  std::vector<double> getProjectedOutput(int steps) {
    std::vector<double> output(steps);

    double x = x0;
    double v = v0;
    double dt = control_period;

    double K = std::exp(-Z * dt);

    double t = steps <= 1 ? 0.0 : control_period * 0.5;
    for (int i = 0; i < steps; ++i) {
      double uk = computeAverageUkOverInterval(t, dt) * sysvmax;
      output[i] = uk / sysvmax;

      double v_next = v * K + uk * (1.0 - K);
      double x_next = x + (1.0 - K) / Z * v + uk * (dt - (1.0 - K) / Z);

      v = v_next;
      x = x_next;
      t += dt;
    }

    return output;
  }

  std::vector<double> getProjectedOutput(ms_t duration) {
    int steps = static_cast<int>(duration.value() / control_period);
    return getProjectedOutput(steps);
  }
};

}

using namespace pdcsu::control::icnor_internal;

namespace pdcsu::control {

class ICNORPositionControl {
private:
  BasePlant plant;
  FFModel ffModel;
  SymmetricHysteresis hys;
  
  amp_t current_limit;
  double scaling_factor = 1.0;
  
  std::shared_ptr<ICNORLearner> learner_;
  std::unique_ptr<ICNOR> icnor;

  unsigned int projection_horizon = 3U;

  radian_t T_ = 0_u_rad;
  radps_t P_ = 0_u_radps;

  std::vector<double> projected_output_;

  size_t projection = 0U;

public:
  ICNORPositionControl(BasePlant plant)
      : plant(plant),
        ffModel(plant),
        hys(0.02_u_rad, 0.05_u_rad),
        current_limit(plant.def_bldc.stall_current),
        learner_(nullptr),
        icnor(constructICNOR(plant.def_bldc.free_speed * 0.85)) {}

  void attachLearner(std::shared_ptr<ICNORLearner> learner) {
    learner_ = std::move(learner);
    if (icnor) { 
      icnor->attachLearner(learner_);
    }
  }

  void setConstraints(radps_t v_max, amp_t current_limit) {
    this->current_limit = current_limit;
    icnor = constructICNOR(v_max);

    ohm_t ir =
        (plant.def_bldc.operating_voltage / plant.def_bldc.stall_current);

    nm_t sys_tau_max =
        plant.def_bldc.stall_torque * ir / (plant.circuit_res + ir);
    scaling_factor = (current_limit / plant.def_bldc.stall_current).value() *
                     (plant.def_bldc.stall_torque / sys_tau_max).value();
    scaling_factor = std::min(scaling_factor, 1.0);
  }

  void setProjectionHorizon(unsigned int horizon) {
    this->projection_horizon = horizon;
  }

  void setTolerance(radian_t inner, radian_t outer) {
    hys.setTolerance(inner, outer);
  }

  double getOutput(radian_t T, radps_t P, radian_t x0, radps_t v0) {
    if (projected_output_.size() == 0U || u_abs(T - T_) > 0.01_u_rad ||
        u_abs(P - P_) > 0.01_u_radps ||
        projection >= projected_output_.size() - 1) {
      icnor->setTargetAndState(T, P, x0, v0);
      icnor->optimize();
      const auto &tuning = icnor->getTuningParameters();
      ffModel.setLoadAdjustments(tuning.load_scale, tuning.load_bias, tuning.friction_scale); // TODO: fix
      projected_output_ = icnor->getProjectedOutput(projection_horizon);
      projection = 0U;
      T_ = T;
      P_ = P;
    }

    double orig_output = projected_output_[projection++];
    orig_output =
        (((plant.def_bldc.free_speed * orig_output - v0) * scaling_factor +
             v0) /
            plant.def_bldc.free_speed)
            .value();

    bool cut = hys.cut(T, x0);

    return (cut ? 0.0 : orig_output) + ffModel.FF(x0, v0, cut);
  }

private:
  std::unique_ptr<ICNOR> constructICNOR(radps_t v_max) {
    DefBLDC bldc2 = plant.def_bldc;
    BasePlant defPlant2 = plant;
    bldc2.stall_current = current_limit;
    double ir =
        (plant.def_bldc.operating_voltage / plant.def_bldc.stall_current)
            .value();

    double adjusted_tau = (bldc2.stall_torque * current_limit /
        u_min(plant.def_bldc.stall_current,
            plant.def_bldc.stall_current *
                (plant.circuit_res.value() + ir) / ir) *
        0.95)
                              .value();
    constexpr double kMinTorque = 1e-3;
    if (adjusted_tau < kMinTorque) adjusted_tau = kMinTorque;
    bldc2.stall_torque = nm_t(adjusted_tau);
    defPlant2.def_bldc = bldc2;
    auto instance = std::make_unique<ICNOR>(defPlant2, v_max);
    if (learner_) { instance->attachLearner(learner_); }
    return instance;
  }
};

}