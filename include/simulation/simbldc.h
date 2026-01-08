#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <random>

#include "util/sysdef.h"
#include "util/units.h"

#ifdef _MSC_VER
#include <corecrt_math_defines.h>
#endif

using namespace pdcsu::units;
using namespace pdcsu::util;

namespace pdcsu::simulation {

struct SimHelper {
  inline static std::mt19937 gen{std::random_device{}()};
  inline static std::uniform_real_distribution<double> dis_vel_q{-1.0, 1.0};
  inline static std::normal_distribution<double> dis_vel_n{0.0, 0.0005};
  inline static std::normal_distribution<double> dis_pos_n{0.0, 0.06};

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
           (v0 - w_conv) * std::exp(-conv_rate.value() * dt.value());
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

  static radps_t vel_noise(radps_t v, radps_t v_max) {
    return v * 0.05 * dis_vel_q(gen) + v_max * dis_vel_n(gen);
  }

  static radian_t pos_noise() { return 1_u_rad * dis_pos_n(gen); }
};

class SimBLDC {
public:
  SimBLDC(BasePlant plant) : plant(plant) {
    setSensorLatency(15_u_ms, 15_u_ms, 20_u_ms);
    setActuatorLatency(10_u_ms);
  }

  void Tick(ms_t dt) {
    sim_time_ += dt;

    // Actuator latency
    while (!actuator_queue_.empty() &&
           actuator_queue_.front().application_time <= sim_time_) {
      this->DC = actuator_queue_.front().DC;
      actuator_queue_.pop_front();
    }

    // Sensor latency
    if (hasSensorLatency()) {
      sensor_history_.push_back({pos, vel, current});
      size_t max_history = static_cast<size_t>(
          std::max(1000.0 / plant.control_period.value(), 100.0));
      if (sensor_history_.size() > max_history) { sensor_history_.pop_front(); }
    }

    radps_t v0 = vel;

    nm_t viscous = plant.viscous_damping * vel;
    nm_t load_func = plant.load_function(pos, vel);
    const double applied_dc = std::clamp(DC, -1.0, 1.0);

    nm_t ext_torque = load_func + viscous + load;
    nm_t motor_torque_guess = plant.def_bldc.stall_torque * applied_dc;
    nm_t drive_balance = motor_torque_guess - ext_torque;

    const radps_t speed = u_abs(vel);
    const radps_t stick_velocity =
        u_min(1e-3_u_radps, plant.def_bldc.free_speed * 0.01);

    nm_t static_friction = plant.friction * 1.05;

    nm_t friction = 0_u_Nm;

    if (speed < stick_velocity) {
      if (drive_balance > 0_u_Nm) {
        friction = u_min(drive_balance, static_friction);

      } else {
        friction = u_max(-drive_balance, -static_friction);
      }
    } else {
      friction =
          plant.friction *
          u_tanh(1_u_rad * 2.0 * vel /
                 radps_t(
                     plant.def_bldc.free_speed));  // Magic number 2.0 adjusted
                                                   // to match experimental data
    }

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
    double clamped_dc = std::clamp(DC, -1.0, 1.0);
    if (actuator_latency_ == 0_u_ms) {
      this->DC = clamped_dc;
    } else {
      actuator_queue_.push_back({clamped_dc, sim_time_ + actuator_latency_});
    }
  }

  void setActuatorLatency(ms_t latency) {
    actuator_latency_ = latency;
    actuator_queue_.clear();
  }

  void setSensorLatency(
      ms_t pos_latency, ms_t vel_latency, ms_t current_latency) {
    sensor_pos_latency_ = pos_latency;
    sensor_vel_latency_ = vel_latency;
    sensor_current_latency_ = current_latency;
    sensor_history_.clear();
  }

  radps_t getVelocity() const {
    radps_t v = getDelayedSensor(sensor_vel_latency_, vel);
    return v + SimHelper::vel_noise(v, plant.def_bldc.free_speed);
  }

  radian_t getPosition() const {
    radian_t p = getDelayedSensor(sensor_pos_latency_, pos);
    return p + SimHelper::pos_noise();
  }

  amp_t getCurrent() const {
    return getDelayedSensor(sensor_current_latency_, current);
  }

  ms_t getTime() {
    return ms_t(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now().time_since_epoch())
            .count());
  }

private:
  struct DelayedCommand {
    double DC;
    ms_t application_time;
  };

  struct SensorSnapshot {
    radian_t position;
    radps_t velocity;
    amp_t current;
  };

  bool hasSensorLatency() const {
    return sensor_pos_latency_ != 0_u_ms || sensor_vel_latency_ != 0_u_ms ||
           sensor_current_latency_ != 0_u_ms;
  }

  template <typename T>
  T getDelayedSensor(ms_t latency, T current_value) const {
    if (latency == 0_u_ms || sensor_history_.empty()) { return current_value; }

    size_t delay_steps = calculateDelaySteps(latency);
    if (delay_steps >= sensor_history_.size()) { return current_value; }

    return getSensorFromHistory<T>(sensor_history_.size() - 1 - delay_steps);
  }

  size_t calculateDelaySteps(ms_t latency) const {
    if (plant.control_period == 0_u_ms) return 0;
    return static_cast<size_t>(std::max(
        0.0, std::round(latency.value() / plant.control_period.value())));
  }

  template <typename T> T getSensorFromHistory(size_t index) const {
    const auto& snapshot = sensor_history_[index];
    if constexpr (std::is_same_v<T, radian_t>) {
      return snapshot.position;
    } else if constexpr (std::is_same_v<T, radps_t>) {
      return snapshot.velocity;
    } else if constexpr (std::is_same_v<T, amp_t>) {
      return snapshot.current;
    }
    return T{};
  }

  amp_t I_lim = 20_u_A;
  nm_t load = 0_u_Nm;

  double DC = 0.0;

  radps_t vel = 0_u_radps;
  radian_t pos = 0_u_rad;
  amp_t current = 0_u_A;

  BasePlant plant;

  ms_t sim_time_ = 0_u_ms;
  ms_t actuator_latency_ = 0_u_ms;
  ms_t sensor_pos_latency_ = 0_u_ms;
  ms_t sensor_vel_latency_ = 0_u_ms;
  ms_t sensor_current_latency_ = 0_u_ms;
  std::deque<DelayedCommand> actuator_queue_;
  std::deque<SensorSnapshot> sensor_history_;
};

}