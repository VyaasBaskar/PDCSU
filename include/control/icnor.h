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

  // Computes the maximum control target ICNOR attempts to apply
  inline double max_control_target(double t) {
    double vmax_ = fabs(zeta);
    if (vmax_ > v_max + tolerance) return vmax_;

    double t2 = t * t;
    double t3 = t2 * t;
    double end_val = fabs(zeta + alpha * t + beta * t2 + gamma * t3);
    if (end_val > v_max + tolerance) return end_val;
    vmax_ = std::max(vmax_, end_val);

    double tmid = t / 2.0;
    double tmid2 = tmid * tmid;
    double tmid3 = tmid2 * tmid;
    double mid_val = fabs(zeta + alpha * tmid + beta * tmid2 + gamma * tmid3);
    if (mid_val > v_max + tolerance) return mid_val;
    vmax_ = std::max(vmax_, mid_val);

    return vmax_;
  }

  // Solves for zeta and alpha, given beta and gamma
  inline std::optional<std::pair<double, double>> solve_zeta_alpha(
      double beta, double gamma, double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;

    double K = exp(-Z * t);

    double M00 = Z * t - 1.0 + K;
    double M01 = 0.5 * Z * t2 - t + (1.0 - K) * invZ;
    double rhs0 = Z * (T - (gamma * t4) / 4.0 - x0) +
                  (t3 * (beta * Z - 3.0 * gamma)) / 3.0 + 2.0 * beta * t2 +
                  3.0 * gamma * t2 * invZ + 2.0 * beta * t * invZ +
                  6.0 * gamma * t * invZ2 - v0 * (1.0 - K) +
                  2.0 * beta * invZ2 * (1.0 - K) +
                  6.0 * gamma * invZ3 * (1.0 - K);

    double M10 = 1.0 - K;
    double M11 = t + invZ * (K - 1.0);

    double rhs1 = P - beta * t2 - gamma * t3 - 2.0 * beta * t * invZ -
                  3.0 * gamma * t2 * invZ - 2.0 * beta * invZ2 -
                  6.0 * gamma * t * invZ2 + 6.0 * gamma * invZ3 - v0 * K +
                  2.0 * beta * K * invZ2 - 6.0 * gamma * K * invZ3;

    double det = M00 * M11 - M01 * M10;
    if (fabs(det) < tolerance) return std::nullopt;

    zeta = (rhs0 * M11 - M01 * rhs1) / det;
    alpha = (M00 * rhs1 - rhs0 * M10) / det;
    return std::make_pair(zeta, alpha);
  }

  // Solves the system, if possible in the specified time
  inline std::optional<std::tuple<double, double, double, double>>
  solve_if_feasible(double t) {
    std::optional<std::tuple<double, double, double, double>> best_sol;

    int coarse_steps = 20;
    int fine_steps = 40;
    double b_range = 160000.0;
    double best_b_real = 0.0;
    double min_b_abs = std::numeric_limits<double>::max();

    for (int i = 0; i <= coarse_steps; ++i) {
      double b = b_range * i / coarse_steps;
      double g = 0.5 * b;
      double b_real = b * -std::copysign(1.0, T - x0 - 10.0 * v0 / v_max);
      double g_real = g * -std::copysign(1.0, T - x0 - 10.0 * v0 / v_max);
      auto sol = solve_zeta_alpha(b_real, g_real, t);
      if (!sol) continue;
      auto [z, a] = *sol;
      if (max_control_target(t) <= v_max + 2.0) {
        double b_abs = std::abs(b_real);
        if (b_abs < min_b_abs) {
          min_b_abs = b_abs;
          best_b_real = b_real;
          best_sol = std::make_tuple(z, a, b_real, g_real);
        }
      }
    }

    if (min_b_abs < std::numeric_limits<double>::max()) {
      double fine_b_range = b_range / coarse_steps;
      for (int i = -fine_steps; i <= fine_steps; ++i) {
        for (int sign : {-1, 1}) {
          double b = (best_b_real / -std::copysign(1.0, T - x0)) +
                     sign * i * fine_b_range / fine_steps;
          double g = 0.5 * b;
          double b_real = b * -std::copysign(1.0, T - x0);
          double g_real = g * -std::copysign(1.0, T - x0);
          auto sol = solve_zeta_alpha(b_real, g_real, t);
          if (!sol) continue;
          auto [z, a] = *sol;
          if (max_control_target(t) <= v_max + 2.0) {
            double b_abs = std::abs(b_real);
            if (b_abs < min_b_abs) {
              min_b_abs = b_abs;
              best_b_real = b_real;
              best_sol = std::make_tuple(z, a, b_real, g_real);
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
                         plant.def_bldc.stall_current *
                         (plant.circuit_res.value() + ir) / ir;
    defPlant2.def_bldc = bldc2;
    return new ICNOR(defPlant2, v_max);
  }
};

}