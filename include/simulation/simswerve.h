#pragma once

#include "simulation/simbldc.h"
#include "util/units.h"

using namespace pdcsu::util;
using namespace pdcsu::units;

// NOTE: this file containes a very simplified swerve simulation

namespace pdcsu::simulation {

#define DRIVE_GR (6.75_u_rot / (4.0_u_in * M_PI))
#define STEER_GR (150_u_rot / 7.0_u_rot)
#define SWERVE_MOTOR_DEF DefBLDC(366_u_A, 2_u_A, 7.09_u_Nm, 6000_u_rpm, 12_u_V)
#define DRIVE_PLANT                                                      \
  DefLinearSys(SWERVE_MOTOR_DEF, 4, DRIVE_GR, 0_u_mps2, 134_u_lb, 0_u_N, \
      0_u_N / 0_u_rpm, 10_u_ms, 0.028_u_ohm)
#define STEER_PLANT                                                    \
  DefArmSys(                                                           \
      SWERVE_MOTOR_DEF, 1, STEER_GR,                                   \
      [](radian_t x, radps_t y) { return 0_u_Nm; },                    \
      0.285_u_lb * 1_u_in * 1_u_in, 0_u_Nm, 0_u_Nm / 0_u_rpm, 10_u_ms, \
      0.028_u_ohm)

// This class is heavily simplified
class SimModule {
public:
  SimModule() : steer_bldc(STEER_PLANT), drive_bldc(DRIVE_PLANT) {}

  void Tick(degree_t direction, double drive) {
    degree_t error = direction - steer_bldc.getPosition();
    bool flip = false;
    while (error > 90_u_deg) {
      error -= 180_u_deg;
      flip = !flip;
    }
    while (error < -90_u_deg) {
      error += 180_u_deg;
      flip = !flip;
    }
    scalar_t pTerm = error / 1_u_deg * 0.2;
    scalar_t dTerm =
        scalar_t(steer_bldc.getVelocity().to_base() * 180.0 / M_PI) * -0.004;

    steer_bldc.setControlTarget((pTerm + dTerm).value());
    steer_bldc.SetCurrentLimit(80_u_A);

    drive_bldc.setControlTarget(drive * (flip ? -1.0 : 1.0));
    drive_bldc.SetCurrentLimit(120_u_A);

    steer_bldc.Tick(10_u_ms);
    drive_bldc.Tick(10_u_ms);
  }

  inch_t getDrivePos() const { return drive_bldc.getPosition() / DRIVE_GR; }
  fps_t getDriveVel() const { return drive_bldc.getVelocity() / DRIVE_GR; }
  radian_t getSteerPos() const { return steer_bldc.getPosition() / STEER_GR; }

private:
  SimBLDC steer_bldc;
  SimBLDC drive_bldc;
};

// This class is heavily simplified
class SimSwerve {
private:
  SimModule FL;
  SimModule FR;
  SimModule BL;
  SimModule BR;

  std::vector<inch_t> last_pos = {0_u_in, 0_u_in, 0_u_in, 0_u_in};

  inch_t x = 0_u_in;
  inch_t y = 0_u_in;

public:
  SimSwerve() {}

  void Tick(fps_t vx, fps_t vy, degps_t w) {
    std::vector<degree_t> rdirs = {135_u_deg, 225_u_deg, -45_u_deg, 45_u_deg};
    units::inch_t radius = 14_u_in * std::sqrt(2.0);

    std::vector<std::pair<units::degree_t, double>> targets;

    for (const degree_t& rdir : rdirs) {
      fps_t v_x = vx + radius * u_cos(-rdir) * w / 1_u_rad;
      fps_t v_y = vy + radius * u_sin(-rdir) * w / 1_u_rad;

      fps_t v_mag = fps_t(std::hypot(v_x.value(), v_y.value()));
      degree_t dir = 90_u_deg - u_atan2(v_y, v_x);

      targets.emplace_back(dir, v_mag);
    }

    FR.Tick(targets[0].first, targets[0].second);
    BR.Tick(targets[1].first, targets[1].second);
    BL.Tick(targets[2].first, targets[2].second);
    FL.Tick(targets[3].first, targets[3].second);

    SimModule* modules[] = {&FL, &FR, &BL, &BR};

    inch_t dx = 0_u_in;
    inch_t dy = 0_u_in;
    for (int i = 0; i < 4; i++) {
      SimModule* module = modules[i];
      inch_t newpos = module->getDrivePos();
      inch_t dpt = newpos - last_pos[i];
      last_pos[i] = newpos;
      degree_t pos = module->getSteerPos();
      dx += dpt * u_cos(pos);
      dy += dpt * u_sin(pos);
    }

    x += dx / 4.0;
    y += dy / 4.0;
  }

  inch_t getX() const { return x; }
  inch_t getY() const { return y; }
};

}