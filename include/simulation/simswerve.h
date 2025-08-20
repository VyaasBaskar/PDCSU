#pragma once

#include "simulation/simbldc.h"
#include "util/units.h"

using namespace pdcsu::util;
using namespace pdcsu::units;

namespace pdcsu::simulation {

#define SWERVE_MOTOR_DEF DefBLDC(366_u_A, 2_u_A, 7.09_u_Nm, 6000_u_rpm, 12_u_V)
#define DRIVE_PLANT                                                           \
  DefLinearSys(SWERVE_MOTOR_DEF, 4, 6.75_u_rot / (4.0_u_in * M_PI), 0_u_mps2, \
      125_u_lb, 0_u_N, 0_u_N / 0_u_rpm, 10_u_ms, 0.028_u_ohm)
#define STEER_PLANT                                                    \
  DefArmSys(                                                           \
      SWERVE_MOTOR_DEF, 1, 150_u_rot / 7_u_rot,                        \
      [](radian_t x, radps_t y) { return 0_u_Nm; },                    \
      0.285_u_lb * 1_u_in * 1_u_in, 0_u_Nm, 0_u_Nm / 0_u_rpm, 10_u_ms, \
      0.028_u_ohm)

class SimModule {
public:
  SimModule(DefBLDC steer_bldc, ohm_t circuit_res)
      : steer_bldc(STEER_PLANT), drive_bldc(DRIVE_PLANT) {}

  void Tick() {
    steer_bldc.Tick(10_u_ms);
    drive_bldc.Tick(10_u_ms);
  }

  SimBLDC& getSteerBLDC() { return steer_bldc; }
  SimBLDC& getDriveBLDC() { return drive_bldc; }

private:
  SimBLDC steer_bldc;
  SimBLDC drive_bldc;
};

}