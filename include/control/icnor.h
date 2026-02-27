#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
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
#include "util/math/solvers.h"
#include "util/sysdef.h"

#ifdef _MSC_VER
#include "corecrt_math_defines.h"
#endif

/* Inexpensive Constrained Nonlinear Optimal Regulator */

using namespace pdcsu::util;
using namespace pdcsu::units;

namespace pdcsu::control {

namespace icnor_internal {
struct ICNORTuningParameters {
  double z_fudge = 1.0;
  double load_scale = 1.0;
  double friction_scale = 1.0;
};

struct ICNORLearningSample {
  double position;
  double velocity;
  double output_raw;
  double load_nm;
};

struct ICNORCompiledLearningSample {
  double real_vel;
  double Zterm;
  double ZLterm;
  double ZFterm;
};

struct ICNORLearnerMinInfo {
  double tau_max;
  double w_f;
  double Z_init;
  double scaling_factor;
  double friction_init;
};

struct LearningRates {
  static constexpr double kLoadScaleLearningRate = 0.05;
  static constexpr double kFrictionScaleLearningRate = 0.05;
  static constexpr double kZLearningRate = 0.01;
};

class ICNOR;
}

class ICNORLearner : public std::enable_shared_from_this<ICNORLearner> {
public:
  explicit ICNORLearner(std::string &storage_path,
      const icnor_internal::ICNORLearnerMinInfo &min_info);
  ~ICNORLearner();

  void setStoragePath(const std::string &path);
  std::string storagePath() const;

  void putLearningSample(const icnor_internal::ICNORLearningSample &sample);
  void processCompiledSamples();

  void loadAsync();
  void saveAsync();

  icnor_internal::ICNORTuningParameters getCurrentTuning() const;

private:
  static std::string ensureExtension(std::string path);
  static bool endsWithInsensitive(
      const std::string &value, const std::string &ending);
  void waitForIO();

  mutable std::mutex mutex_;
  std::string storage_path_;
  mutable std::mutex io_mutex_;
  std::future<void> io_future_;

  double tuned_z_fudge_ = 1.0;
  double tuned_load_scale_ = 1.0;
  double tuned_friction_scale_ = 1.0;

  double prev_time_s_ = 0.0;
  double init_v_ = 0.0;

  double accum_Zterm_v = 0.0;
  double accum_ZLterm_v = 0.0;
  double accum_ZFterm_v = 0.0;

  const icnor_internal::ICNORLearnerMinInfo min_info_;
  static inline const size_t kSampleWindowSize = 30U;
  std::vector<icnor_internal::ICNORCompiledLearningSample> compiled_samples_;
};

inline ICNORLearner::ICNORLearner(std::string &storage_path,
    const icnor_internal::ICNORLearnerMinInfo &min_info)
    : storage_path_(ensureExtension(std::move(storage_path))),
      min_info_(min_info),
      compiled_samples_() {
  if (!storage_path_.empty()) { loadAsync(); }
  compiled_samples_.reserve(kSampleWindowSize);
}

inline ICNORLearner::~ICNORLearner() { waitForIO(); }

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

inline void ICNORLearner::waitForIO() {
  std::future<void> worker;
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (io_future_.valid()) { worker = std::move(io_future_); }
  }
  if (worker.valid()) { worker.get(); }
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
    std::ifstream in(path);
    if (!in.is_open()) return;

    double tuned_z = 1.0, tuned_load_scale = 1.0, tuned_friction_scale = 1.0;
    if (!(in >> tuned_z >> tuned_load_scale >> tuned_friction_scale)) return;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      tuned_z_fudge_ = std::clamp(tuned_z, 0.3, 3.0);
      tuned_load_scale_ = std::clamp(tuned_load_scale, 0.5, 2.0);
      tuned_friction_scale_ = std::clamp(tuned_friction_scale, 0.5, 2.0);
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
    oss << std::setprecision(17) << tuned_z_fudge_ << ' ' << tuned_load_scale_
        << ' ' << tuned_friction_scale_ << '\n';
    data = oss.str();
  }

  auto task = [path, data]() {
    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) return;
    out << data;
    if (!out.good()) return;
  };

  {
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    io_future_ = std::async(std::launch::async, std::move(task));
  }
}

inline icnor_internal::ICNORTuningParameters
ICNORLearner::getCurrentTuning() const {
  std::lock_guard<std::mutex> lock(mutex_);
  icnor_internal::ICNORTuningParameters params;
  params.z_fudge = tuned_z_fudge_;
  params.load_scale = tuned_load_scale_;
  params.friction_scale = tuned_friction_scale_;
  return params;
}

inline void ICNORLearner::putLearningSample(
    const icnor_internal::ICNORLearningSample &sample) {
  // intint (u(t)-w(t) + (frdir*F+load*L * (w_f / tau_max)))*Z

  double now_s = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::steady_clock::now().time_since_epoch())
                     .count();

  double dt = std::clamp(now_s - prev_time_s_, 0.005, 0.05);
  prev_time_s_ = now_s;

  if (compiled_samples_.size() == 0) {
    if (std::abs(sample.velocity) < min_info_.w_f * 0.05) { return; }
    accum_Zterm_v = sample.velocity;
    accum_ZLterm_v = 0.0;
    accum_ZFterm_v = 0.0;
    init_v_ = sample.velocity;
  } else {
    double zTermAccel =
        std::clamp((sample.output_raw -
                       (accum_Zterm_v + accum_ZLterm_v + accum_ZFterm_v)),
            -min_info_.w_f, min_info_.w_f) *
        min_info_.Z_init;

    double maxZTermAccel =
        min_info_.Z_init * min_info_.w_f * min_info_.scaling_factor;
    accum_Zterm_v += dt * std::clamp(zTermAccel, -maxZTermAccel, maxZTermAccel);
    accum_ZFterm_v -= dt * std::tanh(sample.velocity / min_info_.w_f * 15.0) *
                      min_info_.Z_init * min_info_.w_f / min_info_.tau_max;
    accum_ZLterm_v -= dt * sample.load_nm * min_info_.Z_init * min_info_.w_f /
                      min_info_.tau_max;
  }

  compiled_samples_.push_back(icnor_internal::ICNORCompiledLearningSample{
      sample.velocity, accum_Zterm_v, accum_ZLterm_v, accum_ZFterm_v});

  if (compiled_samples_.size() >= kSampleWindowSize) {
    if (std::abs(init_v_ - sample.velocity) > min_info_.w_f * 0.1) {
      processCompiledSamples();
    }
    compiled_samples_.clear();
    accum_Zterm_v = 0.0;
    accum_ZLterm_v = 0.0;
    accum_ZFterm_v = 0.0;
  }
}

inline void ICNORLearner::processCompiledSamples() {
  std::vector<math::LS3x3::LS3x3Input> inputs;
  for (const auto &sample : compiled_samples_) {
    inputs.push_back(math::LS3x3::LS3x3Input{
        sample.Zterm, sample.ZLterm, sample.ZFterm, sample.real_vel});
  }
  auto sol = math::LS3x3::solve(inputs);
  if (sol.r2 > 0.5) {
    double conf = (1.0 - sol.r2) * 2.0;

    double z_new_fudge = std::clamp(sol.A, 0.25, 3.0);
    double load_new_scale = std::clamp(sol.B, 0.5, 2.0);
    double friction_new_scale = std::clamp(sol.C, 0.5, 2.0);

    tuned_z_fudge_ += (z_new_fudge - tuned_z_fudge_) * conf *
                      icnor_internal::LearningRates::kZLearningRate;
    tuned_load_scale_ += (load_new_scale - tuned_load_scale_) * conf *
                         icnor_internal::LearningRates::kLoadScaleLearningRate;
    tuned_friction_scale_ +=
        (friction_new_scale - tuned_friction_scale_) * conf *
        icnor_internal::LearningRates::kFrictionScaleLearningRate;

    saveAsync();
  }

  compiled_samples_.clear();
}

}  // namespace pdcsu::control

namespace pdcsu::control::icnor_internal {

using ::pdcsu::control::ICNORLearner;

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
  double getZ() const {
    return tau_max_nominal_ / (inertia_nominal_ * free_speed_nominal_);
  }

private:
  inline void findZ_and_inverses(double tau_max, double J, double w_f) {
    if (tau_max <= 0.0 || J <= 0.0 || w_f <= 0.0) {
      std::ostringstream oss;
      oss << "Invalid parameters for findZ: tau_max=" << tau_max << " J=" << J
          << " w_f=" << w_f;
      throw std::runtime_error(oss.str());
    }
    Z = tau_max / (J * w_f);
    invZ = 1.0 / Z;
    invZ2 = invZ * invZ;
    invZ3 = invZ2 * invZ;
    invZ4 = invZ3 * invZ;
  }

  inline void recomputeZ() {
    findZ_and_inverses(
        tau_max_nominal_ * z_fudge_, inertia_nominal_, free_speed_nominal_);
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

  inline ControlEvalContext make_control_eval_context(
      double t, double max_allow) const {
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double tmid = 0.5 * t;
    const double tmid2 = tmid * tmid;
    return ControlEvalContext{t, t2, t3, tmid, tmid2, tmid2 * tmid, max_allow};
  }

  inline void updateTimingBounds() {
    last_distance_to_target_ = std::fabs(T - x0);
    const double denom = std::max(v_max, 1e-6);
    double base_hi =
        std::min(3.0, 1.35 * (last_distance_to_target_ / denom) + 0.35);

    constexpr double kMinBaseHi = 0.15;
    base_hi = std::max(base_hi, kMinBaseHi);
    t_hi_init = base_hi;

    t_lo_init = std::max(0.0, 0.5 * t_hi_init - 0.5);
    last_t_hi_init_ = t_hi_init;
    last_t_lo_init_ = t_lo_init;
  }

  // Computes the maximum control target ICNOR attempts to apply
  inline double max_control_target(const ControlEvalContext &ctx,
      double zeta_val, double alpha_val, double beta_val,
      double gamma_val) const {
    double vmax_ = fabs(zeta_val);
    if (vmax_ > ctx.max_allow) return vmax_;

    double end_val = fabs(
        zeta_val + alpha_val * ctx.t + beta_val * ctx.t2 + gamma_val * ctx.t3);
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

    const double T_at_t = T + P * t;
    ctx.rhs0_const = Z * (T_at_t - x0) - v0 * one_minus_K;
    ctx.rhs0_beta =
        (t3 * Z / 3.0) + 2.0 * t2 + 2.0 * t * invZ + 2.0 * invZ2 * one_minus_K;
    ctx.rhs0_gamma = (-Z * t4 / 4.0) - t3 + 3.0 * t2 * invZ + 6.0 * t * invZ2 +
                     6.0 * invZ3 * one_minus_K;

    ctx.rhs1_const = P - v0 * K;
    ctx.rhs1_beta = -t2 - 2.0 * t * invZ - 2.0 * invZ2 + 2.0 * K * invZ2;
    ctx.rhs1_gamma =
        -t3 - 3.0 * t2 * invZ - 6.0 * t * invZ2 + 6.0 * invZ3 - 6.0 * K * invZ3;
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
    if (t <= 0.0) return std::nullopt;
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
    const double coarse_sign = -std::copysign(1.0, T - x0 - 10.0 * v0 / denom);
    const double max_allow = v_max + max_allow_margin_;
    const ControlEvalContext control_ctx =
        make_control_eval_context(t, max_allow);

    constexpr double kBestAbsGoal = 1e-6;

    auto try_candidate = [&](double b_real, double g_real, double z, double a) {
      double candidate_peak =
          max_control_target(control_ctx, z, a, b_real, g_real);
      if (candidate_peak > max_allow) return false;
      double b_abs = std::fabs(b_real);
      bool better = (b_abs < min_b_abs - 1e-15) ||
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
        if (std::fabs(std::fabs(b_real) - b) <= 1e-12) { coarse_done = true; }
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
        max_offset = std::min(
            fine_steps, static_cast<int>(std::ceil(search_limit / fine_step)));
      }
      bool finished = false;
      for (int i = -max_offset; i <= max_offset && !finished; ++i) {
        if (i != 0 && fine_step > 0.0) {
          double delta = std::fabs(i * fine_step);
          double min_possible = std::fabs(std::fabs(base_b) - delta);
          if (min_possible >= min_b_abs - 1e-15) { continue; }
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

    this->tstar = std::max(best_time, control_period);
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
    return std::make_tuple(
        this->tstar, this->zeta, this->alphaS, this->betaS, this->gammaS);
  }

  void attachLearner(const std::shared_ptr<ICNORLearner> &learner) {
    learner_ = learner;
  }

  void applyTuningParameters(const ICNORTuningParameters &params) {
    ICNORTuningParameters tuned = params;
    tuned.z_fudge = std::clamp(tuned.z_fudge, 0.25, 3.0);
    tuned.load_scale = std::clamp(tuned.load_scale, 0.5, 2.0);
    tuned.friction_scale = std::clamp(tuned.friction_scale, 0.5, 2.0);
    tuning_params_ = tuned;
    z_fudge_ = tuning_params_.z_fudge;
    load_scale_ = tuning_params_.load_scale;
    friction_scale_ = tuning_params_.friction_scale;
    recomputeZ();
    updateTimingBounds();
  }

  const ICNORTuningParameters &getTuningParameters() const {
    return tuning_params_;
  }

  bool hasValidSolution() const { return last_feasible_; }

  double getTstar() const { return tstar; }
  double getZeta() const { return zeta; }
  double getAlpha() const { return alphaS; }
  double getBeta() const { return betaS; }
  double getGamma() const { return gammaS; }

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

  double desaturate(
      double dist_to_target, double desat_thresh, double inner_desat) {
    const double x = std::abs(dist_to_target);

    const double a = inner_desat;
    const double b = 0.6;

    const double k1 = 7.8 / desat_thresh;
    const double k2 = 7.8 / desat_thresh;

    const double r0 = b * (1.0 - std::tanh(k2 * desat_thresh)) * 0.5;

    const double scale = 1.0 / (1.0 - r0);

    const double raw = a * std::tanh(k1 * x) +
                       b * (std::tanh(k2 * (x - desat_thresh)) + 1.0) * 0.5;

    return scale * (raw - r0);
  }

  std::vector<double> getProjectedOutput(
      int steps, double desat_thresh = 15.0, double inner_desat = 0.45) {
    std::vector<double> output(steps);

    double x = x0;
    double v = v0;
    double dt = control_period;

    double K = std::exp(-Z * dt);

    double t = steps <= 1 ? 0.0 : control_period * 0.5;
    for (int i = 0; i < steps; ++i) {
      double uk = computeAverageUkOverInterval(t, dt) * sysvmax;

      output[i] = uk / sysvmax;
      const double dist_to_target = std::abs(x - T);
      if (dist_to_target < desat_thresh) {
        const double P_normalized = P / sysvmax;
        const double scale =
            desaturate(dist_to_target, desat_thresh, inner_desat);
        output[i] = output[i] * scale + P_normalized * (1.0 - scale);
      }

      double v_next = v * K + uk * (1.0 - K);
      double x_next = x + (1.0 - K) / Z * v + uk * (dt - (1.0 - K) / Z);

      v = v_next;
      x = x_next;
      t += dt;
    }

    return output;
  }

  std::vector<double> getProjectedOutput(
      ms_t duration, double desat_thresh = 15.0, double inner_desat = 0.45) {
    int steps = static_cast<int>(duration.value() / control_period);
    return getProjectedOutput(steps, desat_thresh, inner_desat);
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
  PositionErrorAccumulator pos_accumulator_;

  amp_t current_limit;
  double scaling_factor = 1.0;
  radian_t desat_thresh = 15.0_rad_;
  double inner_desat = 0.45;

  std::shared_ptr<ICNORLearner> learner_;
  std::unique_ptr<ICNOR> icnor;

  unsigned int projection_horizon = 3U;

  radian_t T_ = 0_rad_;
  radps_t P_ = 0_radps_;

  std::vector<double> projected_output_;

  size_t projection = 0U;

public:
  ICNORPositionControl(BasePlant plant)
      : plant(plant),
        ffModel(plant),
        hys(0.02_rad_, 0.05_rad_),
        current_limit(plant.def_bldc.stall_current),
        learner_(nullptr),
        icnor(constructICNOR(plant.def_bldc.free_speed * 0.85)) {}

  /* NOTE: call attachLearner after setting constraints */
  std::shared_ptr<ICNORLearner> attachLearner(std::string &storage_path) {
    learner_ = std::make_shared<ICNORLearner>(storage_path,
        ICNORLearnerMinInfo{plant.def_bldc.stall_torque.value(),
            radps_t(plant.def_bldc.free_speed).value(), icnor->getZ(),
            scaling_factor, plant.friction.value()});
    if (icnor) { icnor->attachLearner(learner_); }
    return learner_;
  }

  void setConstraints(radps_t v_max, amp_t current_limit) {
    if (learner_) {
      throw std::runtime_error("Call ICNORPositionControl::setConstraints "
                               "prior to attaching ICNORLearner");
    }

    this->current_limit = current_limit;
    icnor = constructICNOR(v_max);

    ohm_t ir =
        (plant.def_bldc.operating_voltage / plant.def_bldc.stall_current);

    nm_t sys_tau_max =
        plant.def_bldc.stall_torque * ir / (plant.circuit_res + ir);

    double ir_value = ir.value();
    double circuit_res_value = plant.circuit_res.value();
    double min_denom = std::min(plant.def_bldc.stall_current.value(),
        plant.def_bldc.stall_current.value() * (circuit_res_value + ir_value) /
            ir_value);
    double adjusted_tau = (plant.def_bldc.stall_torque.value() *
                           current_limit.value() / min_denom);
    constexpr double kMinTorque = 1e-3;
    if (adjusted_tau < kMinTorque) adjusted_tau = kMinTorque;

    double original_sys_tau_max = sys_tau_max.value();
    scaling_factor = adjusted_tau / original_sys_tau_max;
    scaling_factor = std::min(scaling_factor, 1.0);
  }

  void setDesaturationThresh(radian_t thresh, double inner_desat_frac = 0.45) {
    desat_thresh = std::clamp(thresh, 0.0_rad_, 35.0_rad_);
    inner_desat = std::clamp(inner_desat_frac, 0.0, 1.0);
  }

  void setProjectionHorizon(unsigned int horizon) {
    this->projection_horizon = horizon;
  }

  void setTolerance(radian_t inner, radian_t outer) {
    hys.setTolerance(inner, outer);
  }

  double getOutput(radian_t T, radps_t P, radian_t x0, radps_t v0) {
    constexpr double kPosReoptThreshold = 0.001;
    constexpr double kVelReoptThreshold = 0.005;
    if (projected_output_.size() == 0U ||
        u_abs(T - T_) > radian_t(kPosReoptThreshold) ||
        u_abs(P - P_) > radps_t(kVelReoptThreshold) ||
        projection >= projected_output_.size() - 1) {
      pos_accumulator_.reset();
      icnor->setTargetAndState(T, P, x0, v0);
      icnor->optimize();
      const auto &tuning = icnor->getTuningParameters();
      ffModel.setLoadAdjustments(tuning.load_scale, 0.0, tuning.friction_scale);
      projected_output_ = icnor->getProjectedOutput(
          projection_horizon, desat_thresh.value(), inner_desat);
      projection = 0U;
      T_ = T;
      P_ = P;
    }

    double orig_output = projected_output_[projection++];

    const double v0_normalized =
        v0.value() / radps_t(plant.def_bldc.free_speed).value();
    const double v0_abs_normalized = std::fabs(v0_normalized);

    const double velocity_dependent_scale =
        scaling_factor +
        (1.0 - scaling_factor) * std::min(1.0, v0_abs_normalized);
    orig_output = (orig_output - v0_normalized) * velocity_dependent_scale +
                  v0_normalized;

    bool cut = hys.cut(T, x0);
    const radian_t pos_error = T - x0;
    const radian_t activation_threshold = desat_thresh * 0.5;
    const second_t control_period_sec = plant.control_period;
    const double main_output =
        (cut ? (P / radps_t(plant.def_bldc.free_speed)).value() : orig_output) +
        ffModel.FF(x0, v0, orig_output, cut);
    const double accumulator_output =
        cut ? 0.0
            : pos_accumulator_.update(pos_error, v0, control_period_sec,
                  activation_threshold, main_output);

    if (learner_) {
      learner_->putLearningSample(ICNORLearningSample{x0.value(), v0.value(),
          (main_output + accumulator_output) *
              radps_t(plant.def_bldc.free_speed).value(),
          plant.load_function(x0, v0).value()});
    }

    return main_output + accumulator_output;
  }

  bool hasValidSolution() const {
    if (!icnor) return false;
    return icnor->hasValidSolution();
  }

  double getTstar() const {
    if (!icnor) return 0.0;
    return icnor->getTstar();
  }

  double getZeta() const {
    if (!icnor) return 0.0;
    return icnor->getZeta();
  }

  double getAlpha() const {
    if (!icnor) return 0.0;
    return icnor->getAlpha();
  }

  double getBeta() const {
    if (!icnor) return 0.0;
    return icnor->getBeta();
  }

  double getGamma() const {
    if (!icnor) return 0.0;
    return icnor->getGamma();
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