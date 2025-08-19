#pragma once

#include <algorithm>

#include "util/sysdef.h"
#include "util/units.h"

using namespace pdcsu::units;
using namespace pdcsu::util;

namespace pdcsu::simulation {

struct SimBLDC {
  static radps_t predict_velocity(second_t dt, radps_t v0, double DC,
      amp_t I_lim, nm_t load, kgm2_t inertia, DefBLDC def_bldc,
      ohm_t circuit_res) {
    double DC_dmax = (I_lim / def_bldc.stall_current).value();
    double v_as_pct = (v0 / def_bldc.free_speed).value();
    DC = std::clamp(DC, v_as_pct - DC_dmax, v_as_pct + DC_dmax);

    // TODO: possibly consider brake mode case

    ohm_t winding_res = 12_u_V / def_bldc.stall_current;
    nm_t torque_limited =
        def_bldc.stall_torque * winding_res / (winding_res + circuit_res);

    radps_t w_conv =
        def_bldc.free_speed * (scalar_t(DC) - load / torque_limited);
    auto conv_rate = torque_limited / (inertia * radps_t(def_bldc.free_speed));

    return w_conv +
           (v0 - w_conv) * std::pow(M_E, -conv_rate.value() * dt.value());
  }

  static radian_t predict_position(
      second_t dt, radian_t x0, radps_t v0, radps_t v1) {
    return x0 + (v0 + v1) * 0.5 * dt;
  }
};

}