#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <tuple>
#include <utility>

#include "util/sysdef.h"

/* Inexpensive Constrained Nonlinear Optimal Regulator */

namespace pdcsu::control::icnor_internal {

// Halton sequence generation
static constexpr double vdc(int k, int base) {
  double v = 0.0, denom = 1.0;
  while (k) {
    int rem = k % base;
    k /= base;
    denom *= base;
    v += rem / denom;
  }
  return v;
}

static constexpr std::pair<double, double> vdc_pair(
    int i, int base1, int base2) {
  return {vdc(i, base1), vdc(i, base2)};
}

}

using namespace pdcsu::control::icnor_internal;
using namespace pdcsu::util;
using namespace pdcsu::units;

namespace pdcsu::control {

class ICNOR {
private:
  double Z;  // Z = τ_max / (J * w_f)

  double x0, v0;
  double T, P;

  double v_max;

  double control_period;

  // Solver constants
  const double t_lo_init = 1e-6, t_hi_init = 4.0;
  const int time_bisect_iters = 15;
  const int feasibility_trials = 60;
  double beta_gamma_scale = 5.0;
  const double tolerance = 1e-3;

  // Precomputed constants
  double invZ, invZ2, invZ3, invZ4;
  std::array<std::array<double, 4>, 2> A{};
  std::array<double, 2> b{};

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

  inline void setBetaGammaScale() {
    beta_gamma_scale =
        1.5 * v_max / (t_hi_init * t_hi_init * t_hi_init * t_hi_init);
  }

  // Computes matrices used in solving for the control parameters
  inline void precompute_sys(double t, double E) {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;

    A[0][0] = t - invZ + E * invZ;
    A[0][1] = 0.5 * t2 - t * invZ + invZ2 - E * invZ2;
    A[0][2] =
        t3 / 3.0 - t2 * invZ + 2.0 * t * invZ2 - 2.0 * invZ3 + 2.0 * E * invZ3;
    A[0][3] = t4 / 4.0 - t3 * invZ + 3.0 * t2 * invZ2 - 6.0 * t * invZ3 +
              6.0 * invZ4 - 6.0 * E * invZ4;
    b[0] = T - x0 - v0 * invZ + v0 * E * invZ;

    A[1][0] = 1.0 - E;
    A[1][1] = A[0][0];
    A[1][2] = t2 - 2.0 * t * invZ + 2.0 * invZ2 - 2.0 * E * invZ2;
    A[1][3] =
        t3 - 3.0 * t2 * invZ + 6.0 * t * invZ2 - 6.0 * invZ3 + 6.0 * E * invZ3;
    b[1] = P - v0 * E;
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

    double a = 3.0 * gamma;
    double b = 2.0 * beta;
    double c = alpha;

    if (fabs(a) < tolerance) {
      if (fabs(b) > tolerance) {
        double s = -c / b;
        if (s > 0.0 && s < t) {
          double s2 = s * s;
          double val = fabs(zeta + alpha * s + beta * s2 + gamma * s2 * s);
          if (val > v_max + tolerance) return val;
          vmax_ = std::max(vmax_, val);
        }
      }
    } else {
      double disc = b * b - 4.0 * a * c;
      if (disc >= 0.0) {
        double sqrt_disc = sqrt(disc);
        double s1 = (-b + sqrt_disc) / (2 * a);
        double s2 = (-b - sqrt_disc) / (2 * a);
        for (double s : {s1, s2}) {
          if (s > 0.0 && s < t) {
            double s2v = s * s;
            double val = fabs(zeta + alpha * s + beta * s2v + gamma * s2v * s);
            if (val > v_max + tolerance) return val;
            vmax_ = std::max(vmax_, val);
          }
        }
      }
    }
    return vmax_;
  }

  // Solves for zeta and alpha, given beta and gamma
  inline std::optional<std::pair<double, double>> solve_zeta_alpha(
      double beta, double gamma) {
    double M00 = A[0][0], M01 = A[0][1], N00 = A[0][2], N01 = A[0][3];
    double M10 = A[1][0], M11 = A[1][1], N10 = A[1][2], N11 = A[1][3];

    double rhs0 = b[0] - (N00 * beta + N01 * gamma);
    double rhs1 = b[1] - (N10 * beta + N11 * gamma);

    double det = M00 * M11 - M01 * M10;
    if (fabs(det) < tolerance) return std::nullopt;

    zeta = (rhs0 * M11 - M01 * rhs1) / det;
    alpha = (M00 * rhs1 - rhs0 * M10) / det;
    return std::make_pair(zeta, alpha);
  }

  // Solves the system, if possible in the specified time
  inline std::optional<std::tuple<double, double, double, double>>
  solve_if_feasible(double t) {
    double E_t = exp(-Z * t);

    precompute_sys(t, E_t);

    double scale = beta_gamma_scale *
                   std::max({1.0, fabs(P), fabs(v0), fabs(v_max)}) /
                   std::max(1.0, t);

    for (int i = 1; i <= feasibility_trials; i++) {
      auto [u, v] = vdc_pair(i, 2, 3);
      double beta = (u - 0.5) * 2 * scale;
      double gamma = (v - 0.5) * 2 * scale;
      auto sol = solve_zeta_alpha(beta, gamma);
      if (!sol) continue;
      auto [zeta, alpha] = *sol;

      if (max_control_target(t) <= v_max + tolerance)
        return std::make_tuple(zeta, alpha, beta, gamma);
    }
    // zeta = alpha = beta = gamma = 0.0;
    return std::nullopt;
  }

public:
  // Optimizes the control parameters to minimize time
  std::tuple<double, double, double, double, double> optimize() {
    double t_lo = t_lo_init, t_hi = t_hi_init;

    auto feas_hi = solve_if_feasible(t_hi);
    while (!feas_hi && t_hi < 1e6) {
      t_hi *= 2.0;
      feas_hi = solve_if_feasible(t_hi);
    }
    if (!feas_hi) {
      if (T > x0) {
        zeta = v_max;
        alpha = beta = gamma = 0.0;
      } else {
        zeta = -v_max;
        alpha = beta = gamma = 0.0;
      }
      return std::make_tuple(-1.0, zeta, alpha, beta, gamma);
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

  ICNOR(DefBLDC def_bldc, BasePlant def_sys, radps_t v_max)
      : x0(0.0),
        v0(0.0),
        T(0.0),
        P(0.0),
        v_max(v_max.value()),
        control_period(second_t(def_sys.control_period).value()) {
    findZ_and_inverses(def_bldc.stall_torque.value(), def_sys.inertia.value(),
        radps_t(def_bldc.free_speed).value());
    zeta = alphaS = betaS = gammaS = 0.0;
    setBetaGammaScale();
  }

  void setTarget(radian_t T, radps_t P) {
    this->T = T.value();
    this->P = P.value();
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

  // Note: call optimize() prior to this
  std::vector<double> getProjectedOutput(int steps) {
    std::vector<double> output(steps);
    double t = control_period / 2.0;
    for (int i = 0; i < steps; i++) {
      output[i] = zeta + alphaS * t + betaS * t * t + gammaS * t * t * t;
      if (t > tstar) { output[i] = P; }
      t += control_period;
    }
    return output;
  }

  // Note: call optimize() prior to this
  std::vector<double> getProjectedOutput(ms_t duration) {
    int steps = static_cast<int>(duration.value() / control_period);
    return getProjectedOutput(steps);
  }

  double getVmax() { return v_max; }
  double getZ() { return Z; }  // TODO remove these
};

}