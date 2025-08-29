#pragma once

#include "util/sysdef.h"

using namespace pdcsu::util;
using namespace pdcsu::units;

namespace pdcsu::control {

class FFModel {
private:
  BasePlant base_plant;
  ohm_t ir;
  UnitDivision<scalar_t, nm_t> velFF_conversion;

public:
  FFModel(BasePlant def_sys)
      : base_plant(def_sys),
        ir(base_plant.def_bldc.operating_voltage /
            base_plant.def_bldc.stall_current),
        velFF_conversion((base_plant.circuit_res + ir) /
                         (ir * def_sys.def_bldc.stall_torque)) {}

  double FF(radian_t theta, radps_t omega, double cdir, bool cut) const {
    nm_t load = base_plant.load_function(theta, omega);
    nm_t viscous_load = (base_plant.viscous_damping * omega);
    nm_t friction_load = u_copysign(base_plant.friction, omega);

    if (u_abs(omega) < 0.03_u_radps)
      friction_load = u_copysign(friction_load, 1.0 * cdir);
    else if ((cdir > 0.0 && omega < 0.0_u_radps) ||
             (cdir < 0.0 && omega > 0.0_u_radps))
      friction_load = 0.0_u_Nm;

    if (cut) viscous_load = friction_load = 0_u_Nm;
    return ((load + viscous_load + friction_load) * velFF_conversion).value();
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