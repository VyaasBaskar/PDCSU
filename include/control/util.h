#pragma once

#include "util/sysdef.h"

using namespace pdcsu::util;
using namespace pdcsu::units;

namespace pdcsu::control {

class FFModel {
private:
  BasePlant base_plant;
  UnitDivision<radps_t, nm_t> velFF_conversion;

public:
  FFModel(BasePlant def_sys)
      : base_plant(def_sys),
        velFF_conversion(
            def_sys.def_bldc.free_speed / def_sys.def_bldc.stall_torque) {}

  radps_t FF(radian_t theta, radps_t omega, bool cut) const {
    nm_t load = base_plant.load_function(theta, omega);
    nm_t viscous_load = (base_plant.viscous_damping * omega);
    nm_t friction_load = u_copysign(base_plant.friction, omega);
    if (cut) viscous_load = friction_load = 0_u_Nm;
    return (load + viscous_load + friction_load) * velFF_conversion;
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
};

}