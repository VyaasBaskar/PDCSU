#pragma once

#include <functional>

#include "units.h"

/* Sysdef -- Defining System Dynamics */

using namespace pdcsu::units;

namespace pdcsu::util {
struct DefBLDC {
  amp_t stall_current;
  amp_t free_current;
  nm_t stall_torque;
  rpm_t free_speed;

  volt_t operating_voltage;

  DefBLDC(amp_t stall_current, amp_t free_current, nm_t stall_torque,
      rpm_t free_speed, volt_t operating_voltage = 12_u_V)
      : stall_current(stall_current),
        free_current(free_current),
        stall_torque(stall_torque),
        free_speed(free_speed),
        operating_voltage(operating_voltage) {}
};

struct BasePlant {
  DefBLDC def_bldc;

  kgm2_t inertia;
  nm_t friction;
  UnitDivision<nm_t, rpm_t> viscous_damping;

  std::function<nm_t(radian_t, radps_t)> load_function;

  ms_t control_period;
};

struct DefLinearSys : BasePlant {
  int num_motors;
  UnitDivision<radian_t, meter_t> gear_ratio;

  mps2_t effective_gravity = 9.81_u_mps2;

  DefLinearSys(DefBLDC def_bldc, int num_motors,
      UnitDivision<radian_t, meter_t> gear_ratio, mps2_t effective_gravity,
      kg_t mass, newton_t friction,
      UnitDivision<newton_t, rpm_t> viscous_damping, ms_t control_period)
      : BasePlant(plantConstructor(def_bldc, num_motors, gear_ratio,
            effective_gravity, mass, friction, viscous_damping,
            control_period)),
        num_motors(num_motors),
        gear_ratio(gear_ratio) {}

private:
  BasePlant plantConstructor(DefBLDC def_bldc, int num_motors,
      UnitDivision<radian_t, meter_t> gear_ratio, mps2_t effective_gravity,
      kg_t mass, newton_t friction,
      UnitDivision<newton_t, rpm_t> viscous_damping, ms_t control_period) {
    auto loadfn = [mass, effective_gravity, gear_ratio, num_motors](
                      radian_t theta, radps_t omega) -> nm_t {
      return nm_t(
          mass * effective_gravity * 1_u_rad / (gear_ratio * num_motors));
    };
    kgm2_t refl_inertia =
        mass * 1_u_rad * 1_u_rad / (gear_ratio * gear_ratio * num_motors);

    nm_t fric = friction * 1_u_rad / (gear_ratio * num_motors);
    UnitDivision<nm_t, rpm_t> viscfric =
        viscous_damping * 1_u_rad / (gear_ratio * num_motors);

    return BasePlant{
        def_bldc, refl_inertia, fric, viscfric, loadfn, control_period};
  }
};

struct DefArmSys : BasePlant {
  int num_motors;
  scalar_t gear_ratio;

  DefArmSys(DefBLDC def_bldc, int num_motors, scalar_t gear_ratio,
      std::function<nm_t(radian_t, radps_t)> loadfn_0, kgm2_t inertia,
      nm_t friction, UnitDivision<nm_t, rpm_t> viscous_damping,
      ms_t control_period)
      : BasePlant(plantConstructor(def_bldc, num_motors, gear_ratio, loadfn_0,
            inertia, friction, viscous_damping, control_period)),
        num_motors(num_motors),
        gear_ratio(gear_ratio) {}

private:
  BasePlant plantConstructor(DefBLDC def_bldc, int num_motors,
      scalar_t gear_ratio, std::function<nm_t(radian_t, radps_t)> loadfn_0,
      kgm2_t inertia, nm_t friction, UnitDivision<nm_t, rpm_t> viscous_damping,
      ms_t control_period) {
    kgm2_t refl_inertia = inertia / (gear_ratio * gear_ratio * num_motors);

    auto loadfn = [loadfn_0, gear_ratio, num_motors](
                      radian_t theta, radps_t omega) -> nm_t {
      return loadfn_0(theta / gear_ratio, omega / gear_ratio) /
             (gear_ratio * num_motors);
    };

    nm_t fric = friction / (gear_ratio * num_motors);
    UnitDivision<nm_t, rpm_t> viscfric =
        viscous_damping / (gear_ratio * num_motors);

    return BasePlant{
        def_bldc, refl_inertia, fric, viscfric, loadfn, control_period};
  }
};

}