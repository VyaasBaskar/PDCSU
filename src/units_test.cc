#include "util/units.h"

#include <chrono>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace pdcsu::units;

namespace {

constexpr double kEpsilon = 1e-9;

void expect_true(bool condition, const std::string &message) {
  if (!condition) { throw std::runtime_error(message); }
}

void expect_near(double actual, double expected, double tolerance,
    const std::string &message) {
  if (std::abs(actual - expected) > tolerance) {
    std::ostringstream oss;
    oss << message << " (expected " << expected << ", actual " << actual
        << ", tolerance " << tolerance << ")";
    throw std::runtime_error(oss.str());
  }
}

void expect_equal(const std::string &actual, const std::string &expected,
    const std::string &message) {
  if (actual != expected) {
    std::ostringstream oss;
    oss << message << " (expected \"" << expected << "\", actual \"" << actual
        << "\")";
    throw std::runtime_error(oss.str());
  }
}

void test_basic_construction() {
  static_assert(is_same_dimension_v<meter_t, foot_t>,
      "meter_t and foot_t must share the same dimension");
  static_assert(!is_same_dimension_v<meter_t, second_t>,
      "meter_t and second_t must not share the same dimension");

  meter_t default_length;
  expect_near(default_length.value(), 0.0, kEpsilon,
      "Default-constructed meter_t should be zero");

  meter_t five_meters(5.0);
  expect_near(five_meters.value(), 5.0, kEpsilon,
      "meter_t value() should expose the raw magnitude");
  expect_near(five_meters.to_base(), 5.0, kEpsilon,
      "meter_t to_base() should match the stored magnitude");

  foot_t three_feet(3.0);
  const double expected_base_feet = 3.0 * 3048.0 / 10000.0;
  expect_near(three_feet.to_base(), expected_base_feet, kEpsilon,
      "foot_t to_base() should apply the foot to meter conversion factor");

  meter_t converted_from_feet(three_feet);
  expect_near(converted_from_feet.to_base(), three_feet.to_base(), kEpsilon,
      "Conversion from foot_t to meter_t should preserve base value");
}

void test_arithmetic_operations() {
  meter_t a(2.0);
  meter_t b(3.5);

  auto sum = a + b;
  expect_near(sum.value(), 5.5, kEpsilon, "Addition of meter_t failed");

  auto diff = b - a;
  expect_near(diff.value(), 1.5, kEpsilon, "Subtraction of meter_t failed");

  auto scaled = a * 3.0;
  expect_near(
      scaled.value(), 6.0, kEpsilon, "Scaling meter_t by scalar failed");

  auto scaled_left = 3.0 * a;
  expect_near(
      scaled_left.value(), 6.0, kEpsilon, "Left scalar multiplication failed");

  auto divided = b / 2.0;
  expect_near(divided.value(), 1.75, kEpsilon, "Division by scalar failed");

  auto velocity = 10.0_m_ / 2.0_s_;
  expect_near(velocity.value(), 5.0, kEpsilon,
      "Velocity magnitude mismatch for m/s division");
  expect_equal(velocity.dims(), "m s^-1",
      "Velocity dims() should show length over time");

  auto acceleration = velocity / 2.0_s_;
  expect_near(acceleration.value(), 2.5, kEpsilon,
      "Acceleration magnitude mismatch for (m/s)/s");
  expect_equal(acceleration.dims(), "m s^-2",
      "Acceleration dims() should show length over time squared");

  auto area = 2.0_m_ * 3.0_m_;
  expect_near(area.value(), 6.0, kEpsilon,
      "Area magnitude mismatch for meter multiplication");
  expect_equal(area.dims(), "m^2", "Area dims() should reflect squared length");

  auto torque = 4.0_N_ * 0.5_m_;
  expect_near(torque.value(), 2.0, kEpsilon,
      "Torque magnitude mismatch for N*m multiplication");
  expect_equal(torque.dims(), "m^2 kg s^-2",
      "Torque dims() should be length^2 * mass / time^2");

  auto dimensionless = meter_t(6.0) / meter_t(3.0);
  expect_near(dimensionless.value(), 2.0, kEpsilon,
      "Division of like units should yield correct scalar magnitude");
  expect_true(dimensionless.dims().empty(),
      "Division of like units should result in dimensionless dims()");

  meter_t length(2.0);
  scalar_t scale(3.0);
  length *= scale;
  expect_near(length.value(), 6.0, kEpsilon,
      "Multiplying by scalar_t should scale the base unit");
  length /= scale;
  expect_near(length.value(), 2.0, kEpsilon,
      "Dividing by scalar_t should restore the original magnitude");
}

void test_comparisons() {
  meter_t two_a(2.0);
  meter_t two_b(2.0);
  meter_t three(3.0);

  expect_true(two_a == two_b, "Equal meter_t instances should compare equal");
  expect_true(two_a != three, "Different magnitudes should not compare equal");
  expect_true(three > two_a, "Greater magnitude should compare larger");
  expect_true(two_a < three, "Smaller magnitude should compare smaller");
  expect_true(three >= two_b, "Greater-or-equal comparison failed");
  expect_true(two_b <= three, "Less-or-equal comparison failed");

  auto copy = meter_t::from_base(two_a.to_base());
  expect_true(copy == two_a, "Constructing from base should preserve equality");
}

void test_utility_functions() {
  auto negative_length = -5.0_m_;
  auto positive_length = u_abs(negative_length);
  expect_near(positive_length.value(), 5.0, kEpsilon,
      "u_abs should produce positive magnitude");

  auto power = u_pow(3.0_m_, 2.0);
  expect_near(power.value(), 9.0, kEpsilon,
      "u_pow should raise the magnitude to the provided exponent");

  auto copy_signed = u_copysign(5.0_m_, -3.0_m_);
  expect_near(copy_signed.value(), -5.0, kEpsilon,
      "u_copysign with unit sign should transfer the sign");

  auto copy_signed_double = u_copysign(5.0_m_, -1.0);
  expect_near(copy_signed_double.value(), -5.0, kEpsilon,
      "u_copysign with double sign should transfer the sign");

  auto clamped_high = u_clamp(7.0_m_, 2.0_m_, 5.0_m_);
  expect_near(clamped_high.value(), 5.0, kEpsilon,
      "u_clamp should clamp to the provided maximum");

  auto clamped_low = u_clamp(1.0_m_, 2.0_m_, 5.0_m_);
  expect_near(clamped_low.value(), 2.0, kEpsilon,
      "u_clamp should clamp to the provided minimum");

  auto min_val = u_min(4.0_m_, 8.0_m_);
  expect_near(min_val.value(), 4.0, kEpsilon, "u_min should pick the smaller");

  auto max_val = u_max(4.0_m_, 8.0_m_);
  expect_near(max_val.value(), 8.0, kEpsilon, "u_max should pick the larger");

  // Floor operation
  auto floor_positive = u_floor(3.7_m_);
  expect_near(floor_positive.value(), 3.0, kEpsilon,
      "u_floor should round down positive values");

  auto floor_negative = u_floor(-3.7_m_);
  expect_near(floor_negative.value(), -4.0, kEpsilon,
      "u_floor should round down negative values");

  auto floor_exact = u_floor(5.0_m_);
  expect_near(floor_exact.value(), 5.0, kEpsilon,
      "u_floor should preserve exact integer values");

  // Ceiling operation
  auto ceil_positive = u_ceil(3.2_m_);
  expect_near(ceil_positive.value(), 4.0, kEpsilon,
      "u_ceil should round up positive values");

  auto ceil_negative = u_ceil(-3.2_m_);
  expect_near(ceil_negative.value(), -3.0, kEpsilon,
      "u_ceil should round up negative values");

  auto ceil_exact = u_ceil(5.0_m_);
  expect_near(ceil_exact.value(), 5.0, kEpsilon,
      "u_ceil should preserve exact integer values");

  // Round operation
  auto round_up = u_round(3.6_m_);
  expect_near(
      round_up.value(), 4.0, kEpsilon, "u_round should round up values >= 0.5");

  auto round_down = u_round(3.4_m_);
  expect_near(round_down.value(), 3.0, kEpsilon,
      "u_round should round down values < 0.5");

  auto round_half = u_round(3.5_m_);
  expect_near(round_half.value(), 4.0, kEpsilon, "u_round should round 0.5 up");

  auto round_negative = u_round(-3.6_m_);
  expect_near(round_negative.value(), -4.0, kEpsilon,
      "u_round should round negative values correctly");

  // Modulo operation - unit % unit
  auto mod_unit = 10.0_m_ % 3.0_m_;
  expect_near(mod_unit.value(), 1.0, kEpsilon,
      "Unit modulo unit should compute remainder correctly");

  auto mod_unit_exact = 9.0_m_ % 3.0_m_;
  expect_near(mod_unit_exact.value(), 0.0, kEpsilon,
      "Unit modulo unit should return zero for exact division");

  // Modulo operation - unit % int
  auto mod_int = 10.0_m_ % 3;
  expect_near(mod_int.value(), 1.0, kEpsilon,
      "Unit modulo int should compute remainder correctly");

  auto mod_int_exact = 12.0_m_ % 4;
  expect_near(mod_int_exact.value(), 0.0, kEpsilon,
      "Unit modulo int should return zero for exact division");

  // Modulo operation - unit % double
  auto mod_double = 14.0_m_ % 10.5;
  expect_near(mod_double.value(), 3.5, kEpsilon,
      "Unit modulo double should compute remainder correctly");

  auto mod_double_exact = 10.0_m_ % 2.5;
  expect_near(mod_double_exact.value(), 0.0, kEpsilon,
      "Unit modulo double should return zero for exact division");

  // Square root operation
  auto sqrt_simple = u_sqrt(9.0_m_ * 1.0_m_);
  expect_near(sqrt_simple.value(), 3.0, kEpsilon,
      "u_sqrt should compute square root of area to get length");

  auto sqrt_fractional = u_sqrt(2.25_m_ * 1.0_m_);
  expect_near(sqrt_fractional.value(), 1.5, kEpsilon,
      "u_sqrt should handle fractional results correctly");

  auto sqrt_velocity_squared = u_sqrt(16.0_mps_ * 1.0_mps_);
  expect_near(sqrt_velocity_squared.value(), 4.0, kEpsilon,
      "u_sqrt should work with derived units");

  // Verify sqrt dimensions are halved
  auto area = 4.0_m_ * 4.0_m_;
  auto sqrt_area = u_sqrt(area);
  expect_equal(sqrt_area.dims(), "m",
      "u_sqrt should halve dimension exponents (area -> length)");
}

void test_trigonometric_functions() {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTrigTolerance = 1e-6;

  auto zero = 0.0_rad_;
  expect_near(
      u_sin(zero), 0.0, kEpsilon, "u_sin should match std::sin for radians");
  expect_near(
      u_cos(zero), 1.0, kEpsilon, "u_cos should match std::cos for radians");

  auto ninety_deg = 90.0_deg_;
  expect_near(u_sin(ninety_deg), 1.0, kTrigTolerance,
      "u_sin should accept degree_t values");
  expect_near(u_cos(180.0_deg_), -1.0, kTrigTolerance,
      "u_cos should handle degree_t values");

  auto asin_val = u_asin(1.0);
  expect_near(asin_val.value(), kPi / 2.0, 1e-9,
      "u_asin should return pi/2 for input 1.0");

  auto atan_val = u_atan(1.0);
  expect_near(atan_val.value(), kPi / 4.0, 1e-9,
      "u_atan should return pi/4 for input 1.0");

  auto atan2_val = u_atan2(1.0_m_, 1.0_m_);
  expect_near(atan2_val.value(), kPi / 4.0, 1e-9,
      "u_atan2 should produce pi/4 for equal positive inputs");

  expect_near(u_tan(45.0_deg_), 1.0, kTrigTolerance,
      "u_tan should accept degree_t values");
}

void test_literal_conversions() {
  constexpr double kPi = 3.14159265358979323846;
  auto inch = 1.0_in_;
  expect_near(inch.to_base(), 0.0254, 1e-12,
      "1 inch should convert to 0.0254 meters in base units");
  expect_equal(inch.dims(), "in", "inch_t dims() should report \"in\"");

  auto foot = foot_t(12.0_in_);
  expect_near(
      foot.value(), 1.0, kEpsilon, "12 inches should equal 1 foot in value()");
  expect_equal(foot.dims(), "ft", "foot_t dims() should report \"ft\"");

  auto pound = 1.0_lb_;
  expect_near(pound.to_base(), 0.4535924, 1e-7,
      "1 pound should convert to 0.4535924 kilograms in base units");
  expect_equal(pound.dims(), "lb", "pound_t dims() should report \"lb\"");

  auto kilogram = kg_t(pound);
  expect_near(kilogram.value(), 0.4535924, 1e-7,
      "Converting pound_t to kg_t should preserve base value");
  expect_equal(kilogram.dims(), "kg", "kg_t dims() should report \"kg\"");

  auto rpm = 60.0_rpm_;
  expect_near(rpm.to_base(), 2.0 * kPi, 5e-5,
      "60 RPM should equal 2*pi rad/s in base units");
  expect_equal(
      rpm.dims(), "min^-1 rot", "rpm_t dims() should be rotation per minute");

  auto radps = radps_t(2.0);
  expect_equal(
      radps.dims(), "s^-1 rad", "radps_t dims() should be radians per second");

  auto degps2 = 30.0_degps2_;
  expect_equal(degps2.dims(), "s^-2 deg",
      "degps2_t dims() should be degrees per second squared");
}

void test_dimension_tagging() {
  auto square_foot = 2.0_ft_ * 3.0_ft_;
  expect_equal(square_foot.dims(), "ft^2",
      "Multiplying feet should yield ft^2 dimensions");

  auto mixed_area = 1.0_ft_ * 1.0_m_;
  expect_equal(mixed_area.dims(), "m^2",
      "Mixed metric/imperial length multiplication should resolve to mixed "
      "metric tag");

  auto total_length = 1.0_m_ + 3.0_ft_;
  expect_equal(total_length.dims(), "m",
      "Adding different length tags should retain the metric tag");
  expect_near(total_length.value(), 1.0 + 0.9144, 1e-6,
      "1 meter + 3 feet should be 1.9144 meters");
}

void test_compound_units() {
  auto mass = 2.0_kg_;
  auto accel = 3.0_mps2_;
  auto force = mass * accel;
  expect_equal(force.dims(), "m kg s^-2",
      "Force should have dimensions of mass*length/time^2");
  expect_near(
      force.value(), 6.0, kEpsilon, "2 kg * 3 m/s^2 should equal 6 newtons");

  auto distance = 5.0_m_;
  auto work = force * distance;
  expect_equal(work.dims(), "m^2 kg s^-2",
      "Work should have dimensions of mass*length^2/time^2");
  expect_near(work.value(), 30.0, kEpsilon, "6 N * 5 m should equal 30 joules");

  auto power = work / 2.0_s_;
  expect_equal(power.dims(), "m^2 kg s^-3",
      "Power should have dimensions of joules per second");
  expect_near(
      power.value(), 15.0, kEpsilon, "30 J / 2 s should equal 15 watts");

  auto frequency = 2.0_Hz_;
  auto period = scalar_t(1.0) / frequency;
  expect_equal(period.dims(), "s", "Inverse of frequency should yield seconds");
  expect_near(
      period.value(), 0.5, kEpsilon, "1 / 2 Hz should equal 0.5 seconds");
}

void test_performance_benchmark() {
  using clock = std::chrono::high_resolution_clock;
  constexpr int iterations = 100'000'000;

  meter_t unit_val(1.0);
  auto unit_start = clock::now();
  for (int i = 0; i < iterations; ++i) {
    unit_val += meter_t(0.1);
    unit_val -= meter_t(0.05);
    unit_val *= 1.000001;
    unit_val /= 1.000001;
  }
  auto unit_end = clock::now();

  double double_val = 1.0;
  auto double_start = clock::now();
  for (int i = 0; i < iterations; ++i) {
    double_val += 0.1;
    double_val -= 0.05;
    double_val *= 1.000001;
    double_val /= 1.000001;
  }
  auto double_end = clock::now();

  expect_near(unit_val.value(), double_val, 1e-9,
      "Unit and double computations should yield equivalent magnitudes");

  auto unit_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      unit_end - unit_start)
                     .count();
  auto double_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      double_end - double_start)
                       .count();

  expect_true(unit_ns > 0, "Measured unit duration must be positive");
  expect_true(double_ns > 0, "Measured double duration must be positive");

  const double ops_per_iteration = 4.0;
  const double unit_avg_ns =
      static_cast<double>(unit_ns) / (ops_per_iteration * iterations);
  const double double_avg_ns =
      static_cast<double>(double_ns) / (ops_per_iteration * iterations);

  std::cout << "[INFO] (Avg time per operation) <Units>: " << unit_avg_ns
            << " ns, <Doubles>: " << double_avg_ns
            << " ns, ratio: " << (unit_avg_ns / double_avg_ns) << '\n';
}

}  // namespace

int main() {
  struct TestCase {
    const char *name;
    void (*func)();
  };

  const TestCase tests[] = {
      {"basic_construction", test_basic_construction},
      {"arithmetic_operations", test_arithmetic_operations},
      {"comparisons", test_comparisons},
      {"utility_functions", test_utility_functions},
      {"trigonometric_functions", test_trigonometric_functions},
      {"literal_conversions", test_literal_conversions},
      {"dimension_tagging", test_dimension_tagging},
      {"compound_units", test_compound_units},
      {"performance_benchmark", test_performance_benchmark},
  };

  int failures = 0;
  for (const auto &test : tests) {
    try {
      test.func();
      std::cout << "[PASS] " << test.name << '\n';
    } catch (const std::exception &ex) {
      std::cerr << "[FAIL] " << test.name << ": " << ex.what() << '\n';
      ++failures;
    }
  }

  const auto total = static_cast<int>(sizeof(tests) / sizeof(TestCase));
  if (failures > 0) {
    std::cerr << failures << " of " << total << " test(s) failed.\n";
    return 1;
  }

  std::cout << "All " << total << " test(s) passed.\n";
  return 0;
}
