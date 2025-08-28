#pragma once

#include <algorithm>
#include <chrono>

#include "util/sysdef.h"
#include "util/units.h"

using namespace pdcsu::units;
using namespace pdcsu::util;

namespace pdcsu::simulation {

struct SimHelper {
  static radps_t predict_velocity(second_t dt, radps_t v0, double DC,
      amp_t I_lim, nm_t load, kgm2_t inertia, DefBLDC def_bldc,
      ohm_t circuit_res) {
    DC = std::clamp(DC, -1.0, 1.0);
    ohm_t winding_res = def_bldc.operating_voltage / def_bldc.stall_current;

    double DC_dmax = (I_lim / def_bldc.stall_current *
                      (winding_res + circuit_res) / winding_res)
                         .value();

    double v_as_pct = (v0 / radps_t(def_bldc.free_speed)).value();
    DC = std::clamp(DC, v_as_pct - DC_dmax, v_as_pct + DC_dmax);

    // TODO: possibly consider coast mode case

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

  static amp_t predict_current(
      radps_t v, double DC, amp_t I_lim, DefBLDC def_bldc, ohm_t circuit_res) {
    amp_t current =
        (scalar_t(DC) - v / def_bldc.free_speed) * def_bldc.stall_current;
    ohm_t winding_res = def_bldc.operating_voltage / def_bldc.stall_current;
    current *= scalar_t(winding_res / (winding_res + circuit_res));
    return u_clamp(current, -I_lim, I_lim);
  }
};

class SimBLDC {
public:
  SimBLDC(BasePlant plant) : plant(plant) {}

  void Tick(ms_t dt) {
    radps_t v0 = vel;

    nm_t viscous = plant.viscous_damping * vel;
    nm_t load_func = plant.load_function(pos, vel);
    nm_t friction =
        1_u_Nm * std::min(std::abs(plant.friction.value()),
                     std::abs((load_func + viscous).value() +
                              DC * plant.def_bldc.stall_torque.value()));
    friction = u_copysign(nm_t(friction), vel);
    nm_t inh_load = load_func + viscous + friction;

    vel = SimHelper::predict_velocity(dt, v0, DC, I_lim, inh_load + load,
        plant.inertia, plant.def_bldc, plant.circuit_res);
    pos = SimHelper::predict_position(dt, pos, v0, vel);
    current = SimHelper::predict_current(
        vel, DC, I_lim, plant.def_bldc, plant.circuit_res);
  }

  void SetCurrentLimit(amp_t limit) { I_lim = limit; }
  void SetLoad(nm_t load) { this->load = load; }

  void setControlTarget(double DC) {
    this->DC = DC;
    DC = std::clamp(DC, -1.0, 1.0);
  }

  radps_t getVelocity() const { return vel; }
  radian_t getPosition() const { return pos; }
  amp_t getCurrent() const { return current; }

  ms_t getTime() {
    return ms_t(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now().time_since_epoch())
            .count());
  }

private:
  amp_t I_lim = 20_u_A;
  nm_t load = 0_u_Nm;

  double DC = 0.0;

  radps_t vel = 0_u_radps;
  radian_t pos = 0_u_rad;
  amp_t current = 0_u_A;

  BasePlant plant;
};

}