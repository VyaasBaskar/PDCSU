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
  UnitDivision<scalar_t, nm_t> velFF_conversion;
  double load_scale_ = 1.0;
  double load_bias_nm_ = 0.0;
  double friction_scale_ = 1.0;

public:
  FFModel(BasePlant def_sys)
      : base_plant(def_sys),
        ir(base_plant.def_bldc.operating_voltage /
            base_plant.def_bldc.stall_current),
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
    nm_t friction_load = 0.0_u_Nm;

    // const radps_t speed = u_abs(omega);

    if (!cut) {
      friction_load = u_copysign(base_plant.friction * friction_scale_, omega);
    }

    nm_t total_load = total_external + friction_load;
    nm_t adjusted_nm =
        total_load * load_scale_ + load_bias_nm_ * 1_u_Nm;

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

}