#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iterator>
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
  int coarse_steps = 8;
  int fine_steps = 10;
  double beta_range = 80000.0;
  double max_allow_margin = 2.0;
  double time_scale_factor = 1.0;
  double time_offset = 0.0;
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

  void updateEMAs(const ICNORLearningSample &sample);
  std::optional<icnor_internal::ICNORTuningParameters> deriveTuningLocked(
      const icnor_internal::ICNORTuningParameters &current_params);
  void waitForIO();

  mutable std::mutex mutex_;
  std::string storage_path_;
  bool auto_save_ = true;
  size_t autosave_stride_ = 10;
  size_t updates_since_save_ = 0;
  bool dirty_ = false;
  mutable std::mutex io_mutex_;
  std::thread io_thread_;

  size_t sample_count_ = 0;
  double ema_beta_abs_ = 0.0;
  double ema_gamma_abs_ = 0.0;
  double ema_max_beta_abs_ = 0.0;
  double ema_hi_ratio_ = 0.65;
  double ema_saturated_ratio_ = 0.0;
  double ema_failure_ratio_ = 0.0;
  
  double ema_beta_range_ = 80000.0;
  double ema_margin_ = 2.0;
  double ema_time_scale_ = 1.0;
  double ema_time_offset_ = 0.0;
  int tuned_coarse_steps_ = 8;
  int tuned_fine_steps_ = 10;
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

inline void ICNORLearner::updateEMAs(const ICNORLearningSample &sample) {
  ++sample_count_;
  
  constexpr double alpha = 0.15;
  constexpr double one_minus_alpha = 0.85;
  
  ema_beta_abs_ = one_minus_alpha * ema_beta_abs_ + alpha * sample.beta_abs;
  ema_gamma_abs_ = one_minus_alpha * ema_gamma_abs_ + alpha * sample.gamma_abs;
  ema_max_beta_abs_ = std::max(ema_max_beta_abs_ * 0.98, sample.beta_abs);
  
  if (sample.t_hi_init > 1e-6) {
    double ratio = sample.tstar / sample.t_hi_init;
    ema_hi_ratio_ = one_minus_alpha * ema_hi_ratio_ + alpha * ratio;
  }
  
  ema_saturated_ratio_ = one_minus_alpha * ema_saturated_ratio_ + 
                         alpha * (sample.saturated ? 1.0 : 0.0);
  ema_failure_ratio_ = one_minus_alpha * ema_failure_ratio_ + 
                       alpha * (sample.solved ? 0.0 : 1.0);
}

inline std::optional<icnor_internal::ICNORTuningParameters>
ICNORLearner::deriveTuningLocked(
    const icnor_internal::ICNORTuningParameters &current_params) {
  constexpr size_t kMinSamplesForTuning = 5;
  if (sample_count_ < kMinSamplesForTuning) { return std::nullopt; }

  auto params = current_params;
  bool changed = false;

  const double kMinBetaRange = 2000.0;
  const double kMaxBetaRange = 200000.0;
  double target_beta_range =
      std::clamp(ema_max_beta_abs_ * 1.25, kMinBetaRange, kMaxBetaRange);
  ema_beta_range_ = 0.85 * ema_beta_range_ + 0.15 * target_beta_range;

  if (std::fabs(params.beta_range - ema_beta_range_) >
      std::max(0.05 * params.beta_range, 500.0)) {
    params.beta_range = ema_beta_range_;
    changed = true;
  }

  double target_margin = std::clamp(1.0 + ema_saturated_ratio_ * 5.0, 0.5, 6.0);
  ema_margin_ = 0.9 * ema_margin_ + 0.1 * target_margin;
  if (std::fabs(params.max_allow_margin - ema_margin_) > 0.05) {
    params.max_allow_margin = ema_margin_;
    changed = true;
  }

  double desired_ratio = 0.65;
  double scale_target = std::clamp(
      params.time_scale_factor * (1.0 + 0.6 * (ema_hi_ratio_ - desired_ratio)),
      0.5, 2.5);
  ema_time_scale_ = 0.9 * ema_time_scale_ + 0.1 * scale_target;
  if (std::fabs(params.time_scale_factor - ema_time_scale_) > 0.03) {
    params.time_scale_factor = ema_time_scale_;
    changed = true;
  }

  double offset_target = std::clamp(ema_failure_ratio_ * 0.5, -0.2, 0.8);
  ema_time_offset_ = 0.9 * ema_time_offset_ + 0.1 * offset_target;
  if (std::fabs(params.time_offset - ema_time_offset_) > 0.02) {
    params.time_offset = ema_time_offset_;
    changed = true;
  }

  int coarse_target = static_cast<int>(std::round(std::clamp(
      6.0 + ema_saturated_ratio_ * 6.0 + ema_failure_ratio_ * 4.0, 6.0, 14.0)));
  int fine_target = static_cast<int>(std::round(std::clamp(
      9.0 + (ema_gamma_abs_ / std::max(1.0, ema_beta_abs_ + 1e-6)) * 6.0,
      8.0, 24.0)));

  if (params.coarse_steps != coarse_target) {
    params.coarse_steps = coarse_target;
    tuned_coarse_steps_ = coarse_target;
    changed = true;
  }
  if (params.fine_steps != fine_target) {
    params.fine_steps = fine_target;
    tuned_fine_steps_ = fine_target;
    changed = true;
  }

  if (!changed) return std::nullopt;
  return params;
}

inline void ICNORLearner::waitForIO() {
  std::thread worker;
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (io_thread_.joinable()) {
      worker = std::move(io_thread_);
    }
  }
  if (worker.joinable()) {
    worker.join();
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

  std::lock_guard<std::mutex> io_lock(io_mutex_);
  io_thread_ = std::thread([this, path]() {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
      return;
    }

    std::string contents((std::istreambuf_iterator<char>(in)),
        std::istreambuf_iterator<char>());

    size_t sample_count = 0;
    double ema_beta_abs = 0.0, ema_gamma_abs = 0.0, ema_max_beta_abs = 0.0;
    double ema_hi_ratio = 0.65, ema_saturated_ratio = 0.0, ema_failure_ratio = 0.0;
    double ema_beta_range = 80000.0, ema_margin = 2.0;
    double ema_time_scale = 1.0, ema_time_offset = 0.0;
    int tuned_coarse_steps = 8, tuned_fine_steps = 10;

    if (!contents.empty()) {
      std::istringstream numeric(contents);
      double sample_count_d = 0.0;
      double coarse_d = 8.0, fine_d = 10.0;
      if (numeric >> sample_count_d >> ema_beta_abs >> ema_gamma_abs >>
          ema_max_beta_abs >> ema_hi_ratio >> ema_saturated_ratio >>
          ema_failure_ratio >> ema_beta_range >> ema_margin >>
          ema_time_scale >> ema_time_offset >> coarse_d >> fine_d) {
        sample_count = static_cast<size_t>(std::max<double>(0.0, sample_count_d));
        tuned_coarse_steps = static_cast<int>(std::lround(coarse_d));
        tuned_fine_steps = static_cast<int>(std::lround(fine_d));
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
            if (key == "sample_count") sample_count = std::stoull(val);
            else if (key == "ema_beta_abs") ema_beta_abs = std::stod(val);
            else if (key == "ema_gamma_abs") ema_gamma_abs = std::stod(val);
            else if (key == "ema_max_beta_abs") ema_max_beta_abs = std::stod(val);
            else if (key == "ema_hi_ratio") ema_hi_ratio = std::stod(val);
            else if (key == "ema_saturated_ratio")
              ema_saturated_ratio = std::stod(val);
            else if (key == "ema_failure_ratio")
              ema_failure_ratio = std::stod(val);
            else if (key == "ema_beta_range") ema_beta_range = std::stod(val);
            else if (key == "ema_margin") ema_margin = std::stod(val);
            else if (key == "ema_time_scale") ema_time_scale = std::stod(val);
            else if (key == "ema_time_offset") ema_time_offset = std::stod(val);
            else if (key == "tuned_coarse_steps")
              tuned_coarse_steps = std::stoi(val);
            else if (key == "tuned_fine_steps")
              tuned_fine_steps = std::stoi(val);
          } catch (...) {
            continue;
          }
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      sample_count_ = sample_count;
      ema_beta_abs_ = ema_beta_abs;
      ema_gamma_abs_ = ema_gamma_abs;
      ema_max_beta_abs_ = ema_max_beta_abs;
      ema_hi_ratio_ = ema_hi_ratio;
      ema_saturated_ratio_ = ema_saturated_ratio;
      ema_failure_ratio_ = ema_failure_ratio;
      ema_beta_range_ = ema_beta_range;
      ema_margin_ = ema_margin;
      ema_time_scale_ = ema_time_scale;
      ema_time_offset_ = ema_time_offset;
      tuned_coarse_steps_ = tuned_coarse_steps;
      tuned_fine_steps_ = tuned_fine_steps;
      dirty_ = false;
      updates_since_save_ = 0;
    }
  });
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
        << static_cast<unsigned long long>(sample_count_) << ' '
        << ema_beta_abs_ << ' '
        << ema_gamma_abs_ << ' '
        << ema_max_beta_abs_ << ' '
        << ema_hi_ratio_ << ' '
        << ema_saturated_ratio_ << ' '
        << ema_failure_ratio_ << ' '
        << ema_beta_range_ << ' '
        << ema_margin_ << ' '
        << ema_time_scale_ << ' '
        << ema_time_offset_ << ' '
        << static_cast<int>(tuned_coarse_steps_) << ' '
        << static_cast<int>(tuned_fine_steps_) << '\n';
    data = oss.str();
  }

  std::lock_guard<std::mutex> io_lock(io_mutex_);
  io_thread_ = std::thread([this, path, data]() {
    std::ofstream out(path, std::ios::trunc);
    if (out.is_open()) {
      out << data;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      dirty_ = false;
      updates_since_save_ = 0;
    }
  });
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
    updateEMAs(sample);
    dirty_ = true;
    ++updates_since_save_;
    result.new_tuning = deriveTuningLocked(current_params);
    if (auto_save_ && autosave_stride_ > 0 &&
        updates_since_save_ >= autosave_stride_) {
      should_save_now = true;
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
  params.coarse_steps = tuned_coarse_steps_;
  params.fine_steps = tuned_fine_steps_;
  params.beta_range = ema_beta_range_;
  params.max_allow_margin = ema_margin_;
  params.time_scale_factor = ema_time_scale_;
  params.time_offset = ema_time_offset_;
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
    Z = tau_max / (J * w_f);
    invZ = 1.0 / Z;
    invZ2 = invZ * invZ;
    invZ3 = invZ2 * invZ;
    invZ4 = invZ3 * invZ;
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

    const int coarse_steps = std::max(4, tuning_params_.coarse_steps);
    const int fine_steps = std::max(4, tuning_params_.fine_steps);
    const double b_range = std::max(1000.0, tuning_params_.beta_range);
    double best_b_real = 0.0;
    double min_b_abs = std::numeric_limits<double>::max();
    double best_peak = std::numeric_limits<double>::infinity();

    const SolveContext ctx = make_solve_context(t);
    if (fabs(ctx.det) < tolerance) return std::nullopt;

    const double denom = std::max(v_max, 1e-6);
    const double coarse_sign =
        -std::copysign(1.0, T - x0 - 10.0 * v0 / denom);
    const double max_allow = v_max + tuning_params_.max_allow_margin;
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
    tuned.coarse_steps = std::max(4, tuned.coarse_steps);
    tuned.fine_steps = std::max(4, tuned.fine_steps);
    tuned.beta_range = std::clamp(tuned.beta_range, 500.0, 500000.0);
    tuned.max_allow_margin = std::clamp(tuned.max_allow_margin, 0.5, 10.0);
    tuned.time_scale_factor =
        std::clamp(tuned.time_scale_factor, 0.25, 3.0);
    tuned.time_offset = std::clamp(tuned.time_offset, -0.5, 1.5);
    tuning_params_ = tuned;
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
        control_period(second_t(def_sys.control_period).value()) {
    findZ_and_inverses(def_sys.def_bldc.stall_torque.value(),
        def_sys.inertia.value(), radps_t(def_sys.def_bldc.free_speed).value());
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
  
  std::unique_ptr<ICNOR> icnor;
  std::shared_ptr<ICNORLearner> learner_;

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

    return (cut ? 0.0 : orig_output) + ffModel.FF(x0, v0, orig_output, cut);
  }

private:
  std::unique_ptr<ICNOR> constructICNOR(radps_t v_max) {
    DefBLDC bldc2 = plant.def_bldc;
    BasePlant defPlant2 = plant;
    bldc2.stall_current = current_limit;
    double ir =
        (plant.def_bldc.operating_voltage / plant.def_bldc.stall_current)
            .value();

    bldc2.stall_torque = bldc2.stall_torque * current_limit /
                         u_min(plant.def_bldc.stall_current,
                             plant.def_bldc.stall_current *
                                 (plant.circuit_res.value() + ir) / ir) *
                         0.95;
    defPlant2.def_bldc = bldc2;
    auto instance = std::make_unique<ICNOR>(defPlant2, v_max);
    if (learner_) { instance->attachLearner(learner_); }
    return instance;
  }
};

}