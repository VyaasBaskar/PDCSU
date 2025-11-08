#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <tuple>
#include <utility>

#include "control/util.h"
#include "util/sysdef.h"

#ifdef _WIN32
#include "corecrt_math_defines.h"
#endif

/* Inexpensive Constrained Nonlinear Optimal Regulator */

using namespace pdcsu::util;
using namespace pdcsu::units;

namespace pdcsu::control::icnor_internal {

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

  // Computes the maximum control target ICNOR attempts to apply
  inline double max_control_target(const ControlEvalContext &ctx) const {
    double vmax_ = fabs(zeta);
    if (vmax_ > ctx.max_allow) return vmax_;

    double end_val =
        fabs(zeta + alpha * ctx.t + beta * ctx.t2 + gamma * ctx.t3);
    if (end_val > ctx.max_allow) return end_val;
    vmax_ = std::max(vmax_, end_val);

    double mid_val = fabs(zeta + alpha * ctx.tmid + beta * ctx.tmid2 +
                          gamma * ctx.tmid3);
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
  inline std::optional<std::tuple<double, double, double, double>>
  solve_if_feasible(double t) {
    std::optional<std::tuple<double, double, double, double>> best_sol;

    int coarse_steps = 8;
    int fine_steps = 10;
    double b_range = 80000.0;
    double best_b_real = 0.0;
    double min_b_abs = std::numeric_limits<double>::max();

    const SolveContext ctx = make_solve_context(t);
    if (fabs(ctx.det) < tolerance) return std::nullopt;

    const double coarse_sign =
        -std::copysign(1.0, T - x0 - 10.0 * v0 / v_max);
    const double max_allow = v_max + 2.0;
    const ControlEvalContext control_ctx =
        make_control_eval_context(t, max_allow);

    constexpr double kBestAbsGoal = 1e-6;

    bool coarse_done = false;
    for (int i = 0; i <= coarse_steps && !coarse_done; ++i) {
      double b = b_range * i / coarse_steps;
      double g = 0.5 * b;
      double b_real = b * coarse_sign;
      double g_real = g * coarse_sign;
      auto sol = solve_zeta_alpha(b_real, g_real, ctx);
      if (!sol) continue;
      auto [z, a] = *sol;
      if (max_control_target(control_ctx) <= max_allow) {
        double b_abs = std::fabs(b_real);
        if (b_abs < min_b_abs) {
          min_b_abs = b_abs;
          best_b_real = b_real;
          best_sol = std::make_tuple(z, a, b_real, g_real);
          if (b_abs <= kBestAbsGoal) { return best_sol; }
          if (b_abs == b) {
            coarse_done = true;
          }
        }
      }
    }

    if (min_b_abs < std::numeric_limits<double>::max()) {
      double fine_b_range = b_range / coarse_steps;
      const double fine_sign = -std::copysign(1.0, T - x0);
      const double base_b = best_b_real / fine_sign;
      const double fine_step = fine_b_range / fine_steps;
      int max_offset = fine_steps;
      double base_abs = std::fabs(base_b);
      double search_limit = base_abs + min_b_abs;
      if (fine_step > 0.0) {
        max_offset =
            std::min(fine_steps,
                static_cast<int>(std::ceil(search_limit / fine_step)));
      }
      bool finished = false;
      for (int i = -max_offset; i <= max_offset && !finished; ++i) {
        if (i != 0) {
          double delta = std::fabs(i * fine_step);
          double min_possible =
              std::fabs(std::fabs(base_b) - delta);
          if (min_possible >= min_b_abs - 1e-15) {
            continue;
          }
        }
        const int sign_count = (i == 0) ? 1 : 2;
        for (int idx = 0; idx < sign_count; ++idx) {
          int sign = (idx == 0) ? 1 : -1;
          double b = base_b + sign * i * fine_step;
          double g = 0.5 * b;
          double b_real = b * fine_sign;
          double g_real = g * fine_sign;
          auto sol = solve_zeta_alpha(b_real, g_real, ctx);
          if (!sol) continue;
          auto [z, a] = *sol;
          if (max_control_target(control_ctx) <= max_allow) {
            double b_abs = std::fabs(b_real);
            if (b_abs < min_b_abs) {
              min_b_abs = b_abs;
              best_b_real = b_real;
              best_sol = std::make_tuple(z, a, b_real, g_real);
              if (b_abs <= kBestAbsGoal) {
                finished = true;
                break;
              }
            }
          }
        }
      }
    }
    if (best_sol) { return best_sol; }

    return std::nullopt;
  }

public:
  // Optimizes the control parameters to minimize time
  std::tuple<double, double, double, double, double> optimize() {
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
      return std::make_tuple(
          1000.0, this->zeta, this->alphaS, this->betaS, this->gammaS);
    }

    auto [lzeta, lalpha, lbeta, lgamma] = *feas_hi;
    std::tuple<double, double, double, double, double> best = {
        t_hi, lzeta, lalpha, lbeta, lgamma};

    for (int i = 0; i < time_bisect_iters; i++) {
      double t_mid = 0.5 * (t_lo + t_hi);
      auto feas_mid = solve_if_feasible(t_mid);
      if (feas_mid) {
        t_hi = t_mid;
        auto [z, a, bta, gma] = *feas_mid;
        best = {t_mid, z, a, bta, gma};
      } else {
        t_lo = t_mid;
      }
    }
    this->tstar = std::get<0>(best);
    this->zeta = std::get<1>(best);
    this->alphaS = std::get<2>(best);
    this->betaS = std::get<3>(best);
    this->gammaS = std::get<4>(best);
    return best;
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
  }

  void setTarget(radian_t T, radps_t P) {
    this->T = T.value();
    this->P = P.value();
    this->t_hi_init =
        std::min(3.0, 1.35 * std::abs(T.value() - x0) / v_max + 0.35);
    this->t_lo_init = std::max(-0.5, 0.5 * this->t_hi_init - 0.5);
  }

  void setState(radian_t x0, radps_t v0) {
    this->x0 = x0.value();
    this->v0 = v0.value();
  }

  void setTargetAndState(radian_t T, radps_t P, radian_t x0, radps_t v0) {
    setTarget(T, P);
    setState(x0, v0);
  }

  // Note: call optimize() prior to this
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

  // Note: call optimize() prior to this
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

  // Note: call optimize() prior to this
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
  ICNOR* icnor;

  amp_t current_limit;
  double scaling_factor = 1.0;

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
        icnor(constructICNOR(plant.def_bldc.free_speed * 0.85)),
        current_limit(plant.def_bldc.stall_current) {}

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
  ICNOR* constructICNOR(radps_t v_max) {
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
    return new ICNOR(defPlant2, v_max);
  }
};

}