#pragma once

#include <algorithm>
#include <cmath>

#include "util/sysdef.h"

using namespace pdcsu::util;
using namespace pdcsu::units;

namespace pdcsu::control {

class FFModel {
private:
  BasePlant base_plant;
  ohm_t ir;
  radps_t omega_max_ = 600.0_radps_;
  UnitDivision<scalar_t, nm_t> velFF_conversion;
  double load_scale_ = 1.0;
  double load_bias_nm_ = 0.0;
  double friction_scale_ = 1.0;

public:
  FFModel(BasePlant def_sys)
      : base_plant(def_sys),
        ir(base_plant.def_bldc.operating_voltage /
            base_plant.def_bldc.stall_current),
        omega_max_(base_plant.def_bldc.free_speed),
        velFF_conversion((base_plant.circuit_res + ir) /
                         (ir * def_sys.def_bldc.stall_torque)) {}

  void setLoadAdjustments(double scale, double bias_nm, double friction_scale) {
    load_scale_ = std::clamp(scale, 0.5, 2.0);
    load_bias_nm_ = bias_nm;
    friction_scale_ = std::clamp(friction_scale, 0.5, 2.0);
  }

  double FF(radian_t theta, radps_t omega, bool cut) const {
    nm_t load = base_plant.load_function(theta, omega);
    nm_t viscous_load = base_plant.viscous_damping * u_abs(omega);
    viscous_load = u_copysign(viscous_load, omega);

    nm_t total_external = load + viscous_load;
    nm_t friction_load = 0.0_Nm_;

    if (!cut) {
      friction_load = base_plant.friction * friction_scale_ *
                      u_tanh(1_rad_ * 2.0 * omega /
                             omega_max_);  // Magic number 2.0 adjusted to
                                           // match experimental data
    }

    nm_t total_load = total_external + friction_load;
    nm_t adjusted_nm = total_load * load_scale_ + load_bias_nm_ * 1_Nm_;

    return (adjusted_nm * velFF_conversion).value();
  }
};

class SymmetricHysteresis {
private:
  radian_t inner_hyst;
  radian_t outer_hyst;
  bool reached;

public:
  SymmetricHysteresis(radian_t inner, radian_t outer)
      : inner_hyst(inner), outer_hyst(outer), reached(false) {}

  bool cut(radian_t setpoint, radian_t pos) {
    if (reached && u_abs(setpoint - pos) > outer_hyst) {
      reached = false;
    } else if (u_abs(setpoint - pos) < inner_hyst) {
      reached = true;
    }

    return reached;
  }

  void setTolerance(radian_t inner, radian_t outer) {
    inner_hyst = inner;
    outer_hyst = outer;
  }
};

class PositionErrorAccumulator {
private:
  UnitCompound<radian_t, second_t> integral_ = 0.0_rad_ * 0.0_s_;
  UnitCompound<radian_t, second_t> max_integral_ = 0.04_rad_ * 0.0_s_;
  double max_output_ = 0.04;
  second_t kD = 0.007_s_;

public:
  PositionErrorAccumulator() = default;

  void setMaxOutput(double max_output) {
    max_output_ = std::abs(max_output);
    max_integral_ = max_output_ * 1.5_rad_ * 1_s_;
  }
  double update(radian_t position_error, radps_t current_velocity,
      second_t control_period, radian_t activation_threshold,
      double main_controller_output = 0.0) {
    if (std::abs(main_controller_output) > 1.5 * max_output_) {
      integral_ = 0.0_rad_ * 0.0_s_;
      return 0.0;
    }
    position_error -= current_velocity * kD;
    integral_ += position_error * control_period;
    integral_ = std::clamp(integral_, -max_integral_, max_integral_);
    if (position_error > activation_threshold)
      integral_ *=
          1 - std::abs(u_tanh(1_rad_ * position_error / activation_threshold));
    else
      integral_ *=
          std::abs(u_tanh(1_rad_ * position_error / activation_threshold));

    return std::clamp(integral_.value(), -max_output_, max_output_);
  }

  void reset() { integral_ = 0.0_rad_ * 0.0_s_; }

  double getIntegral() const { return integral_.value(); }
};

}