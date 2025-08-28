#pragma once
#include <corecrt_math_defines.h>

#include <ratio>
#include <sstream>
#include <string>
#include <type_traits>

/* A PDCSU Units Library to enforce type safety for physical quantities */

namespace pdcsu::units {
// Base Unit
template <typename Fac = std::ratio<1>, typename L = std::ratio<0>,
    typename M = std::ratio<0>, typename T = std::ratio<0>,
    typename I = std::ratio<0>, typename R = std::ratio<0>>

struct Unit {
  using L_exp = L;
  using M_exp = M;
  using T_exp = T;
  using I_exp = I;
  using R_exp = R;
  using Fac_ = Fac;

  double value_;
  static constexpr double factor = Fac::num * 1.0 / Fac::den;

  constexpr explicit Unit(double v) : value_(v) {}

  constexpr double to_base() const { return value_ * factor; }

  template <typename Ratio>
  void __internal_concat_dim(std::ostringstream &oss, const char *name) {
    if constexpr (Ratio::num != 0) {
      if constexpr (Ratio::num == 1 && Ratio::den == 1)
        oss << name << " ";
      else if constexpr (Ratio::den != 1)
        oss << name << "^" << Ratio::num << "/" << Ratio::den << " ";
      else
        oss << name << "^" << Ratio::num << " ";
    }
  }

  std::string dims() const {
    std::ostringstream oss;

    __internal_concat_dim<L_exp>(oss, "m");
    __internal_concat_dim<M_exp>(oss, "kg");
    __internal_concat_dim<T_exp>(oss, "s");
    __internal_concat_dim<I_exp>(oss, "A");
    __internal_concat_dim<R_exp>(oss, "rad");

    std::string result = oss.str();
    if (!result.empty()) { result.pop_back(); }
    return result;
  }

  template <typename Fac2>
  constexpr Unit(const Unit<Fac2, L, M, T, I, R> &o)
      : value_(o.to_base() / factor) {}

  constexpr double value() const { return value_; }

  template <typename Fac2>
  constexpr auto operator+(const Unit<Fac2, L, M, T, I, R> &o) const {
    return Unit(value_ + o.to_base() / factor);
  }

  template <typename Fac2>
  constexpr auto operator-(const Unit<Fac2, L, M, T, I, R> &o) const {
    return Unit(value_ - o.to_base() / factor);
  }

  constexpr Unit operator-() const { return Unit(-value_); }

  constexpr Unit operator*(double s) const { return Unit(value_ * s); }
  constexpr Unit operator/(double s) const { return Unit(value_ / s); }

  constexpr Unit operator*=(double s) {
    value_ *= s;
    return *this;
  }

  constexpr Unit operator/=(double s) {
    value_ /= s;
    return *this;
  }

  template <typename Fac2>
  constexpr Unit &operator+=(const Unit<Fac2, L, M, T, I, R> &o) {
    value_ += o.to_base() / factor;
    return *this;
  }

  template <typename Fac2>
  constexpr Unit &operator-=(const Unit<Fac2, L, M, T, I, R> &o) {
    value_ -= o.to_base() / factor;
    return *this;
  }

  template <typename Fac2>
  constexpr Unit &operator*=(const Unit<Fac2, std::ratio<0>, std::ratio<0>,
      std::ratio<0>, std::ratio<0>, std::ratio<0>> &o) {
    value_ *= o.to_base() / factor;
    return *this;
  }

  template <typename Fac2>
  constexpr Unit &operator/=(const Unit<Fac2, std::ratio<0>, std::ratio<0>,
      std::ratio<0>, std::ratio<0>, std::ratio<0>> &o) {
    value_ /= o.to_base() / factor;
    return *this;
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2>
  constexpr auto operator*(const Unit<Fac2, L2, M2, T2, I2, R2> &o) const {
    return Unit<std::ratio_multiply<Fac, Fac2>, std::ratio_add<L, L2>,
        std::ratio_add<M, M2>, std::ratio_add<T, T2>, std::ratio_add<I, I2>,
        std::ratio_add<R, R2>>(value() * o.value());
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2>
  constexpr auto operator/(const Unit<Fac2, L2, M2, T2, I2, R2> &o) const {
    return Unit<std::ratio_divide<Fac, Fac2>, std::ratio_subtract<L, L2>,
        std::ratio_subtract<M, M2>, std::ratio_subtract<T, T2>,
        std::ratio_subtract<I, I2>, std::ratio_subtract<R, R2>>(
        value() / o.value());
  }

  template <typename Fac2>
  constexpr bool operator==(const Unit<Fac2, L, M, T, I, R> &o) const {
    return value_ == o.to_base() / factor;
  }
  template <typename Fac2>
  constexpr bool operator!=(const Unit<Fac2, L, M, T, I, R> &o) const {
    return value_ != o.to_base() / factor;
  }
  template <typename Fac2>
  constexpr bool operator<(const Unit<Fac2, L, M, T, I, R> &o) const {
    return value_ < o.to_base() / factor;
  }
  template <typename Fac2>
  constexpr bool operator<=(const Unit<Fac2, L, M, T, I, R> &o) const {
    return value_ <= o.to_base() / factor;
  }
  template <typename Fac2>
  constexpr bool operator>(const Unit<Fac2, L, M, T, I, R> &o) const {
    return value_ > o.to_base() / factor;
  }
  template <typename Fac2>
  constexpr bool operator>=(const Unit<Fac2, L, M, T, I, R> &o) const {
    return value_ >= o.to_base() / factor;
  }
};

/* Compound Unit Creator */

template <typename U1, typename U2>
using UnitCompound =
    Unit<std::ratio_multiply<typename U1::Fac_, typename U2::Fac_>,
        std::ratio_add<typename U1::L_exp, typename U2::L_exp>,
        std::ratio_add<typename U1::M_exp, typename U2::M_exp>,
        std::ratio_add<typename U1::T_exp, typename U2::T_exp>,
        std::ratio_add<typename U1::I_exp, typename U2::I_exp>,
        std::ratio_add<typename U1::R_exp, typename U2::R_exp>>;

/* Divided Unit Creator */
template <typename U1, typename U2>
using UnitDivision =
    Unit<std::ratio_divide<typename U1::Fac_, typename U2::Fac_>,
        std::ratio_subtract<typename U1::L_exp, typename U2::L_exp>,
        std::ratio_subtract<typename U1::M_exp, typename U2::M_exp>,
        std::ratio_subtract<typename U1::T_exp, typename U2::T_exp>,
        std::ratio_subtract<typename U1::I_exp, typename U2::I_exp>,
        std::ratio_subtract<typename U1::R_exp, typename U2::R_exp>>;

/* Base Unit Aliases */
using scalar_t = Unit<std::ratio<1>>;
using meter_t = Unit<std::ratio<1>, std::ratio<1>>;
using foot_t = Unit<std::ratio<3048, 10000>, std::ratio<1>>;
using inch_t = Unit<std::ratio<254, 10000>, std::ratio<1>>;
using kg_t = Unit<std::ratio<1>, std::ratio<0>, std::ratio<1>>;
using pound_t =
    Unit<std::ratio<4535924, 10000000>, std::ratio<0>, std::ratio<1>>;
using second_t =
    Unit<std::ratio<1>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using minute_t =
    Unit<std::ratio<60>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using ms_t =
    Unit<std::ratio<1, 1000>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using amp_t = Unit<std::ratio<1>, std::ratio<0>, std::ratio<0>, std::ratio<0>,
    std::ratio<1>>;
using radian_t = Unit<std::ratio<1>, std::ratio<0>, std::ratio<0>,
    std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using degree_t = Unit<std::ratio<1745329, 100000000>, std::ratio<0>,
    std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using rotation_t = Unit<std::ratio<62832, 10000>, std::ratio<0>, std::ratio<0>,
    std::ratio<0>, std::ratio<0>, std::ratio<1>>;

// Derived Unit Aliases
using mps_t = UnitDivision<meter_t, second_t>;
using fps_t = UnitDivision<foot_t, second_t>;
using mps2_t = UnitDivision<mps_t, second_t>;
using fps2_t = UnitDivision<fps_t, second_t>;
using newton_t = UnitCompound<kg_t, mps2_t>;
using joule_t = UnitCompound<newton_t, meter_t>;
using watt_t = UnitDivision<joule_t, second_t>;
using hertz_t = UnitDivision<scalar_t, second_t>;
using kgm2_t = UnitCompound<UnitCompound<kg_t, meter_t>, meter_t>;
using coulomb_t = UnitCompound<amp_t, second_t>;
using volt_t = UnitDivision<joule_t, coulomb_t>;
using ohm_t = UnitDivision<volt_t, amp_t>;
using rpm_t = UnitDivision<rotation_t, minute_t>;
using nm_t = UnitCompound<newton_t, meter_t>;
using radps_t = UnitDivision<radian_t, second_t>;
using radps2_t = UnitDivision<radps_t, second_t>;
using degps_t = UnitDivision<degree_t, second_t>;
using degps2_t = UnitDivision<degps_t, second_t>;

// Absolute value
template <typename Fac, typename L, typename M, typename T, typename I,
    typename R>
static inline Unit<Fac, L, M, T, I, R> u_abs(
    const Unit<Fac, L, M, T, I, R> &u) {
  return Unit<Fac, L, M, T, I, R>(std::abs(u.value()));
}

// Power operation
template <typename Fac, typename L, typename M, typename T, typename I>
constexpr auto u_pow(const Unit<Fac, L, M, T, I> &u, double exp) {
  return Unit<Fac, L, M, T, I>(std::pow(u.value(), exp));
}

// Copysign
template <typename U1, typename U2> constexpr auto u_copysign(U1 u, U2 sign) {
  static_assert(
      std::is_base_of_v<
          Unit<typename U1::Fac_, typename U1::L_exp, typename U1::M_exp,
              typename U1::T_exp, typename U1::I_exp, typename U1::R_exp>,
          U1> &&
      std::is_base_of_v<
          Unit<typename U2::Fac_, typename U2::L_exp, typename U2::M_exp,
              typename U2::T_exp, typename U2::I_exp, typename U2::R_exp>,
          U2>);
  return Unit<typename U1::Fac_, typename U1::L_exp, typename U1::M_exp,
      typename U1::T_exp, typename U1::I_exp, typename U1::R_exp>(
      std::copysign(u.value(), sign.value()));
}
template <typename Fac, typename L, typename M, typename T, typename I,
    typename R>
auto u_copysign(const Unit<Fac, L, M, T, I, R> &u, double sign) {
  return Unit<Fac, L, M, T, I, R>(std::copysign(u.value(), sign));
}

// Clamp
template <typename Fac, typename L, typename M, typename T, typename I,
    typename R>
Unit<Fac, L, M, T, I, R> u_clamp(const Unit<Fac, L, M, T, I, R> &u,
    const Unit<Fac, L, M, T, I, R> &min, const Unit<Fac, L, M, T, I, R> &max) {
  return Unit<Fac, L, M, T, I, R>(
      std::clamp(u.value(), min.value(), max.value()));
}

// Trigonometric functions
static inline double u_sin(const radian_t &a) { return std::sin(a.to_base()); }
static inline double u_cos(const radian_t &a) { return std::cos(a.to_base()); }
static inline double u_tan(const radian_t &a) { return std::tan(a.to_base()); }

static inline double u_sin(const degree_t &a) { return std::sin(a.to_base()); }
static inline double u_cos(const degree_t &a) { return std::cos(a.to_base()); }
static inline double u_tan(const degree_t &a) { return std::tan(a.to_base()); }

static inline radian_t u_asin(double x) { return radian_t(std::asin(x)); }
static inline radian_t u_acos(double x) { return radian_t(std::acos(x)); }
static inline radian_t u_atan(double x) { return radian_t(std::atan(x)); }

// atan2
template <typename UY, typename UX>
static inline radian_t u_atan2(const UY &y, const UX &x) {
  static_assert(std::is_same_v<typename UY::L_exp, typename UX::L_exp> &&
                    std::is_same_v<typename UY::M_exp, typename UX::M_exp> &&
                    std::is_same_v<typename UY::T_exp, typename UX::T_exp> &&
                    std::is_same_v<typename UY::I_exp, typename UX::I_exp> &&
                    std::is_same_v<typename UY::R_exp, typename UX::R_exp>,
      "atan2 arguments must have same dimensions");
  return radian_t(std::atan2(y.to_base(), x.to_base()));
}

// Literals

// Scalar
inline scalar_t operator"" _u_(long double v) { return scalar_t(v); }
inline scalar_t operator"" _u_(unsigned long long v) {
  return scalar_t(static_cast<double>(v));
}

// Length
inline meter_t operator"" _u_m(long double v) { return meter_t(v); }
inline meter_t operator"" _u_m(unsigned long long v) {
  return meter_t(static_cast<double>(v));
}
inline foot_t operator"" _u_ft(long double v) { return foot_t(v); }
inline foot_t operator"" _u_ft(unsigned long long v) {
  return foot_t(static_cast<double>(v));
}
inline inch_t operator"" _u_in(long double v) { return inch_t(v); }
inline inch_t operator"" _u_in(unsigned long long v) {
  return inch_t(static_cast<double>(v));
}

// Mass
inline kg_t operator"" _u_kg(long double v) { return kg_t(v); }
inline kg_t operator"" _u_kg(unsigned long long v) {
  return kg_t(static_cast<double>(v));
}
inline pound_t operator"" _u_lb(long double v) { return pound_t(v); }
inline pound_t operator"" _u_lb(unsigned long long v) {
  return pound_t(static_cast<double>(v));
}

// Time
inline second_t operator"" _u_s(long double v) { return second_t(v); }
inline second_t operator"" _u_s(unsigned long long v) {
  return second_t(static_cast<double>(v));
}
inline minute_t operator"" _u_min(long double v) { return minute_t(v); }
inline minute_t operator"" _u_min(unsigned long long v) {
  return minute_t(static_cast<double>(v));
}
inline ms_t operator"" _u_ms(long double v) { return ms_t(v); }
inline ms_t operator"" _u_ms(unsigned long long v) {
  return ms_t(static_cast<double>(v));
}

// Current
inline amp_t operator"" _u_A(long double v) { return amp_t(v); }
inline amp_t operator"" _u_A(unsigned long long v) {
  return amp_t(static_cast<double>(v));
}

// Angles
inline degree_t operator"" _u_deg(long double v) { return degree_t(v); }
inline degree_t operator"" _u_deg(unsigned long long v) {
  return degree_t(static_cast<double>(v));
}
inline radian_t operator"" _u_rad(long double v) { return radian_t(v); }
inline radian_t operator"" _u_rad(unsigned long long v) {
  return radian_t(static_cast<double>(v));
}
inline rotation_t operator"" _u_rot(long double v) { return rotation_t(v); }
inline rotation_t operator"" _u_rot(unsigned long long v) {
  return rotation_t(static_cast<double>(v));
}

// Derived units
inline volt_t operator"" _u_V(long double v) { return volt_t(v); }
inline volt_t operator"" _u_V(unsigned long long v) {
  return volt_t(static_cast<double>(v));
}
inline ohm_t operator"" _u_ohm(long double v) { return ohm_t(v); }
inline ohm_t operator"" _u_ohm(unsigned long long v) {
  return ohm_t(static_cast<double>(v));
}
inline hertz_t operator"" _u_Hz(long double v) { return hertz_t(v); }
inline hertz_t operator"" _u_Hz(unsigned long long v) {
  return hertz_t(static_cast<double>(v));
}
inline newton_t operator"" _u_N(long double v) { return newton_t(v); }
inline newton_t operator"" _u_N(unsigned long long v) {
  return newton_t(static_cast<double>(v));
}
inline rpm_t operator"" _u_rpm(long double v) { return rpm_t(v); }
inline rpm_t operator"" _u_rpm(unsigned long long v) {
  return rpm_t(static_cast<double>(v));
}
inline nm_t operator"" _u_Nm(long double v) { return nm_t(v); }
inline nm_t operator"" _u_Nm(unsigned long long v) {
  return nm_t(static_cast<double>(v));
}
inline radps_t operator"" _u_radps(long double v) { return radps_t(v); }
inline radps_t operator"" _u_radps(unsigned long long v) {
  return radps_t(static_cast<double>(v));
}
inline radps2_t operator"" _u_radps2(long double v) { return radps2_t(v); }
inline radps2_t operator"" _u_radps2(unsigned long long v) {
  return radps2_t(static_cast<double>(v));
}
inline degps_t operator"" _u_degps(long double v) { return degps_t(v); }
inline degps_t operator"" _u_degps(unsigned long long v) {
  return degps_t(static_cast<double>(v));
}
inline degps2_t operator"" _u_degps2(long double v) { return degps2_t(v); }
inline degps2_t operator"" _u_degps2(unsigned long long v) {
  return degps2_t(static_cast<double>(v));
}
inline mps_t operator"" _u_mps(long double v) { return mps_t(v); }
inline mps_t operator"" _u_mps(unsigned long long v) {
  return mps_t(static_cast<double>(v));
}
inline mps2_t operator"" _u_mps2(long double v) { return mps2_t(v); }
inline mps2_t operator"" _u_mps2(unsigned long long v) {
  return mps2_t(static_cast<double>(v));
}
inline fps_t operator"" _u_fps(long double v) { return fps_t(v); }
inline fps_t operator"" _u_fps(unsigned long long v) {
  return fps_t(static_cast<double>(v));
}
inline fps2_t operator"" _u_fps2(long double v) { return fps2_t(v); }
inline fps2_t operator"" _u_fps2(unsigned long long v) {
  return fps2_t(static_cast<double>(v));
}
inline kgm2_t operator"" _u_kgm2(long double v) { return kgm2_t(v); }
inline kgm2_t operator"" _u_kgm2(unsigned long long v) {
  return kgm2_t(static_cast<double>(v));
}

}
