#pragma once

#include <cassert>
#include <cmath>
#include <string>
#include <type_traits>
#include <vector>

#include "../units.h"

namespace pdcsu::util::math {

// Helper trait to check if a type is a pdcsu::units::Unit
namespace detail {
template <typename T> struct is_unit_type : std::false_type {};
template <typename Fac, typename L, typename M, typename T, typename I,
    typename R, typename LTag, typename MTag, typename TTag, typename ITag,
    typename RTag>
struct is_unit_type<
    pdcsu::units::Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag>>
    : std::true_type {};
}  // namespace detail

template <typename UT, size_t N> class uVec {
  static_assert(N > 0, "uVec can not be created with less than one dimension.");
  static_assert(detail::is_unit_type<UT>::value,
      "uVec can only be created with unit types. Unit_t is invalid.");

  using T = UT;

private:
  std::vector<T> data;

public:
  // Default constructor initializes with zeros
  uVec() : data{N, T()} {}

  // Constructs an N-dimensional vector from N unit-type values
  uVec(std::initializer_list<T> dims) : uVec() {
    assert(dims.size() == N && "uVec must be constructed with N dimensions.");
    std::copy(dims.begin(), dims.end(), data.begin());
  }

  // Constructs a 2D vector from magnitude and angle
  // If angleIsBearing is true, 0_deg is +y and angles are measured clockwise
  // Otherwise, 0_deg is +x and angles are measured counter-clockwise
  uVec(T magnitude, pdcsu::units::degree_t theta, bool angleIsBearing = false)
      : uVec() {
    assert(N == 2 && "Polar constructor can only be used with 2D vectors.");
    if (angleIsBearing) { theta = 90_u_deg - theta; }
    data[0] = magnitude * pdcsu::units::u_cos(theta);
    data[1] = magnitude * pdcsu::units::u_sin(theta);
  }

  // Constructs 2D vector from a pair
  uVec(const std::pair<T, T>& vec) : uVec() {
    assert(N == 2 && "Pair constructor can only be used with 2D vectors.");
    data = {vec.first, vec.second};
  }

  // Copy constructor for uVec
  uVec(const uVec<UT, N>& other) : data(other.data) {}

  // Adds uVec<UT, N> to this and returns result
  [[nodiscard]] uVec<UT, N> operator+(const uVec<UT, N>& other) const {
    uVec<UT, N> result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = data[i] + other[i];
    }
    return result;
  }

  // Subtracts uVec<UT, N> from this and returns result
  [[nodiscard]] uVec<UT, N> operator-(const uVec<UT, N>& other) const {
    uVec<UT, N> result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = data[i] - other[i];
    }
    return result;
  }

  [[nodiscard]] uVec<UT, N> operator*(const double scalar) const {
    uVec<UT, N> result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = data[i] * scalar;
    }
    return result;
  }

  friend uVec<UT, N> operator*(double lhs, const uVec<UT, N>& rhs) {
    return rhs * lhs;
  }

  [[nodiscard]] uVec<UT, N> operator/(const double scalar) const {
    uVec<UT, N> result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = data[i] / scalar;
    }
    return result;
  }

  uVec<UT, N>& operator+=(const uVec<UT, N>& other) {
    for (size_t i = 0; i < N; ++i) {
      data[i] += other[i];
    }
    return *this;
  }

  uVec<UT, N>& operator-=(const uVec<UT, N>& other) {
    for (size_t i = 0; i < N; ++i) {
      data[i] -= other[i];
    }
    return *this;
  }

  uVec<UT, N>& operator*=(const double scalar) {
    for (size_t i = 0; i < N; ++i) {
      data[i] *= scalar;
    }
    return *this;
  }

  uVec<UT, N>& operator/=(const double scalar) {
    for (size_t i = 0; i < N; ++i) {
      data[i] /= scalar;
    }
    return *this;
  }

  uVec<UT, N>& operator=(const uVec<UT, N>& other) {
    if (this != &other) { data = other.data; }
    return *this;
  }

  // Uses 'safe' double comparison
  [[nodiscard]] bool operator==(const uVec<UT, N>& other) const {
    constexpr double epsilon = 1e-9;
    for (size_t i = 0; i < N; ++i) {
      if (std::abs(data[i].to_base() - other[i].to_base()) > epsilon) {
        return false;
      }
    }
    return true;
  }

  // Returns a vector rotated by a given angle. Default is clockwise rotation.
  [[nodiscard]] uVec<UT, N> rotate(
      pdcsu::units::degree_t angle, bool clockwise = true) const {
    static_assert(N == 2, "Rotation is only defined for 2D vectors.");
    if (clockwise) { angle = -angle; }
    double cos_val = pdcsu::units::u_cos(angle);
    double sin_val = pdcsu::units::u_sin(angle);
    return {data[0] * cos_val - data[1] * sin_val,
        data[0] * sin_val + data[1] * cos_val};
  }

  // Returns the dot product of this vector and another with the units of the
  // other vector
  template <typename UT2>
  [[nodiscard]] auto dot(const uVec<UT2, N>& other) const {
    auto result = data[0] * other[0];
    for (size_t i = 1; i < N; ++i) {
      result += data[i] * other[i];
    }
    return result;
  }

  // Returns the cross product of this vector and another
  // Cross product is only defined for 3D vectors
  template <typename UT2>
  [[nodiscard]] auto cross(const uVec<UT2, N>& other) const {
    static_assert(N == 3, "Cross product is only defined for 3D vectors.");
    auto x = data[1] * other[2] - data[2] * other[1];
    auto y = data[2] * other[0] - data[0] * other[2];
    auto z = data[0] * other[1] - data[1] * other[0];
    using ResultType = decltype(x);
    return uVec<ResultType, N>{x, y, z};
  }

  // Returns the magnitude of this vector
  [[nodiscard]] T magnitude() const {
    auto result = data[0] * data[0];
    for (size_t i = 1; i < N; ++i) {
      result += data[i] * data[i];
    }
    return T::from_base(std::sqrt(result.to_base()));
  }

  // Returns the unit vector of this vector
  [[nodiscard]] uVec<UT, N> unit() const {
    return *this / magnitude().to_base();
  }

  // Projects this vector onto another and returns
  template <typename UT2>
  [[nodiscard]] uVec<UT, N> projectOntoAnother(
      const uVec<UT2, N>& other) const {
    return other.projectOntoThis(*this);
  }

  // Projects another vector onto this and returns
  template <typename UT2>
  [[nodiscard]] uVec<UT2, N> projectOntoThis(const uVec<UT2, N>& other) const {
    assert(N == 2 && "Projection is only defined for 2D vectors.");
    double unit_x = unit()[0].to_base();
    double unit_y = unit()[1].to_base();
    auto dot_result = dot(other);
    return {UT2::from_base(unit_x * dot_result.to_base()),
        UT2::from_base(unit_y * dot_result.to_base())};
  }

  // Returns the angle of this vector
  // If angleIsBearing is true, 0_deg is +y and angles are measured clockwise
  // Otherwise, 0_deg is +x and angles are measured counter-clockwise
  [[nodiscard]] pdcsu::units::degree_t angle(
      bool angleIsBearing = false) const {
    assert(N == 2 && "Angle can only be calculated for 2D vectors.");
    if (angleIsBearing) {
      auto rad = pdcsu::units::u_atan2(data[0], data[1]);
      return pdcsu::units::degree_t(rad.to_base());
    }
    try {
      auto rad = pdcsu::units::u_atan2(data[1], data[0]);
      return pdcsu::units::degree_t(rad.to_base());
    } catch (std::exception& exc) {
      (void)exc;
      return 0_u_deg;
    }
  }

  // Returns the angle between this vector and another
  template <typename UT2>
  [[nodiscard]] pdcsu::units::degree_t angleTo(
      const uVec<UT2, N>& other, bool angleIsBearing = false) const {
    return other.angle(angleIsBearing) - angle(angleIsBearing);
  }

  // Returns a modified vector with a given delta added to its magnitude
  [[nodiscard]] uVec<UT, N> AddToMagnitude(T delta) const {
    assert(N == 2 && "AddToMagnitude is only defined for 2D vectors.");
    T new_magnitude = T::from_base(magnitude().to_base() + delta.to_base());
    return {new_magnitude, angle(true), true};
  }

  // Returns a modified vector to the resized magniutde
  [[nodiscard]] uVec<UT, N> resize(T magnitude) const {
    assert(N == 2 && "resize is only defined for 2D vectors.");
    return {magnitude, angle(true), true};
  }

  // Const and non-const accessors for vector elements
  const T& operator[](size_t i) const { return data[i]; }
  T& operator[](size_t i) { return data[i]; }

  // Returns a pair representation of a 2D vector
  [[nodiscard]] std::pair<T, T> toPair() const {
    static_assert(N == 2 && "toPair can only be used with 2D vectors.");
    return {data[0], data[1]};
  }

  // Returns string representation of this vector
  [[nodiscard]] std::string toString() const {
    std::string output = "<";
    for (size_t i = 0; i < N; ++i) {
      output += std::to_string(data[i].value());
      if (i < N - 1) output += ", ";
    }
    output += ">";
    return output;
  }
};

// Commonly used vector types

// 1D vector, pdcsu::units::inch_t
using Vector1D = uVec<pdcsu::units::inch_t, 1>;
// 2D vector, pdcsu::units::inch_t
using Vector2D = uVec<pdcsu::units::inch_t, 2>;
// 3D vector, pdcsu::units::inch_t
using Vector3D = uVec<pdcsu::units::inch_t, 3>;

}  // namespace pdcsu::util::math