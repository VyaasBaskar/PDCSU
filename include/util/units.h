#pragma once
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <type_traits>

/* A PDCSU Units Library to enforce type safety for physical quantities */

namespace pdcsu::units {
namespace detail {

constexpr std::int64_t abs_i64(std::int64_t v) { return v < 0 ? -v : v; }

constexpr std::int64_t gcd_i64(std::int64_t a, std::int64_t b) {
  return b == 0 ? abs_i64(a) : gcd_i64(b, a % b);
}

constexpr std::int64_t lcm_i64(std::int64_t a, std::int64_t b) {
  return (a == 0 || b == 0) ? 0 : abs_i64(a / gcd_i64(a, b) * b);
}

template <std::int64_t Num, std::int64_t Den = 1> struct Fraction {
  static_assert(Den != 0, "Fraction denominator cannot be zero.");

private:
  static constexpr std::int64_t g = gcd_i64(Num, Den);
  static constexpr std::int64_t adjusted_num = (Den < 0 ? -Num : Num);
  static constexpr std::int64_t adjusted_den = abs_i64(Den);

public:
  static constexpr std::int64_t num = adjusted_num / (g == 0 ? 1 : g);
  static constexpr std::int64_t den = adjusted_den / (g == 0 ? 1 : g);
};

template <typename F> struct fraction_negate {
  using type = Fraction<-F::num, F::den>;
};
template <typename F>
using fraction_negate_t = typename fraction_negate<F>::type;

template <typename F1, typename F2> struct fraction_add {
  static constexpr std::int64_t common_den = lcm_i64(F1::den, F2::den);
  using type = Fraction<F1::num *(common_den / F1::den) +
                            F2::num *(common_den / F2::den),
      common_den>;
};
template <typename F1, typename F2>
using fraction_add_t = typename fraction_add<F1, F2>::type;

template <typename F1, typename F2> struct fraction_subtract {
  static constexpr std::int64_t common_den = lcm_i64(F1::den, F2::den);
  using type = Fraction<F1::num *(common_den / F1::den) -
                            F2::num *(common_den / F2::den),
      common_den>;
};
template <typename F1, typename F2>
using fraction_subtract_t = typename fraction_subtract<F1, F2>::type;

template <typename F1, typename F2> struct fraction_multiply {
  using type = Fraction<F1::num * F2::num, F1::den * F2::den>;
};
template <typename F1, typename F2>
using fraction_multiply_t = typename fraction_multiply<F1, F2>::type;

template <typename F1, typename F2> struct fraction_divide {
  static_assert(F2::num != 0, "Cannot divide by zero fraction.");
  using type = Fraction<F1::num * F2::den, F1::den * F2::num>;
};
template <typename F1, typename F2>
using fraction_divide_t = typename fraction_divide<F1, F2>::type;

template <typename F> inline constexpr bool fraction_is_zero_v = (F::num == 0);

template <typename F1, typename F2>
inline constexpr bool fractions_equal_v =
    (F1::num == F2::num && F1::den == F2::den);

enum class UnitSystem : std::uint8_t { Metric, Imperial, Mixed };

template <UnitSystem SystemValue, const char *SymbolLiteral> struct DimTagBase {
  static constexpr UnitSystem system = SystemValue;
  static constexpr const char *symbol() { return SymbolLiteral; }
};

inline constexpr const char kSymbolM[] = "m";
inline constexpr const char kSymbolFt[] = "ft";
inline constexpr const char kSymbolIn[] = "in";
inline constexpr const char kSymbolKg[] = "kg";
inline constexpr const char kSymbolLb[] = "lb";
inline constexpr const char kSymbolS[] = "s";
inline constexpr const char kSymbolMin[] = "min";
inline constexpr const char kSymbolMs[] = "ms";
inline constexpr const char kSymbolA[] = "A";
inline constexpr const char kSymbolRad[] = "rad";
inline constexpr const char kSymbolDeg[] = "deg";
inline constexpr const char kSymbolRot[] = "rot";

using LengthMetricTag = DimTagBase<UnitSystem::Metric, kSymbolM>;
using LengthFootTag = DimTagBase<UnitSystem::Imperial, kSymbolFt>;
using LengthInchTag = DimTagBase<UnitSystem::Imperial, kSymbolIn>;
using LengthMixedTag = DimTagBase<UnitSystem::Mixed, kSymbolM>;

using MassMetricTag = DimTagBase<UnitSystem::Metric, kSymbolKg>;
using MassPoundTag = DimTagBase<UnitSystem::Imperial, kSymbolLb>;
using MassMixedTag = DimTagBase<UnitSystem::Mixed, kSymbolKg>;

using TimeSecondTag = DimTagBase<UnitSystem::Metric, kSymbolS>;
using TimeMinuteTag = DimTagBase<UnitSystem::Imperial, kSymbolMin>;
using TimeMillisecondTag = DimTagBase<UnitSystem::Metric, kSymbolMs>;
using TimeMixedTag = DimTagBase<UnitSystem::Mixed, kSymbolS>;

using CurrentAmpTag = DimTagBase<UnitSystem::Metric, kSymbolA>;
using CurrentMixedTag = DimTagBase<UnitSystem::Mixed, kSymbolA>;

using AngleRadTag = DimTagBase<UnitSystem::Metric, kSymbolRad>;
using AngleDegreeTag = DimTagBase<UnitSystem::Imperial, kSymbolDeg>;
using AngleRotationTag = DimTagBase<UnitSystem::Imperial, kSymbolRot>;
using AngleMixedTag = DimTagBase<UnitSystem::Mixed, kSymbolRad>;

template <typename Exp1, typename Exp2, typename Tag1, typename Tag2,
    typename MixedTag>
struct combine_dim_tag {
  using type = std::conditional_t<fraction_is_zero_v<Exp1>,
      std::conditional_t<fraction_is_zero_v<Exp2>, Tag1, Tag2>,
      std::conditional_t<fraction_is_zero_v<Exp2>, Tag1,
          std::conditional_t<std::is_same_v<Tag1, Tag2>, Tag1, MixedTag>>>;
};

template <typename Exp1, typename Exp2, typename Tag1, typename Tag2,
    typename MixedTag>
using combine_dim_tag_t =
    typename combine_dim_tag<Exp1, Exp2, Tag1, Tag2, MixedTag>::type;

}  // namespace detail
// Base Unit
template <typename Fac = detail::Fraction<1>, typename L = detail::Fraction<0>,
    typename M = detail::Fraction<0>, typename T = detail::Fraction<0>,
    typename I = detail::Fraction<0>, typename R = detail::Fraction<0>,
    typename LTag = detail::LengthMetricTag,
    typename MTag = detail::MassMetricTag,
    typename TTag = detail::TimeSecondTag,
    typename ITag = detail::CurrentAmpTag, typename RTag = detail::AngleRadTag>

struct Unit {
  using L_exp = L;
  using M_exp = M;
  using T_exp = T;
  using I_exp = I;
  using R_exp = R;
  using Fac_ = Fac;
  using L_tag = LTag;
  using M_tag = MTag;
  using T_tag = TTag;
  using I_tag = ITag;
  using R_tag = RTag;

  double base_value_;
  static constexpr double factor = Fac::num * 1.0 / Fac::den;

private:
  template <typename Lr, typename Mr, typename Tr, typename Ir, typename Rr>
  struct DimensionSummary {
    static constexpr auto length_num = Lr::num;
    static constexpr auto length_den = Lr::den;
    static constexpr auto mass_num = Mr::num;
    static constexpr auto mass_den = Mr::den;
    static constexpr auto time_num = Tr::num;
    static constexpr auto time_den = Tr::den;
    static constexpr auto current_num = Ir::num;
    static constexpr auto current_den = Ir::den;
    static constexpr auto angle_num = Rr::num;
    static constexpr auto angle_den = Rr::den;
  };

  template <typename LTagT, typename MTagT, typename TTagT, typename ITagT,
      typename RTagT>
  struct TagSummary {
    static constexpr detail::UnitSystem length_system = LTagT::system;
    static constexpr const char *length_symbol = LTagT::symbol();
    static constexpr detail::UnitSystem mass_system = MTagT::system;
    static constexpr const char *mass_symbol = MTagT::symbol();
    static constexpr detail::UnitSystem time_system = TTagT::system;
    static constexpr const char *time_symbol = TTagT::symbol();
    static constexpr detail::UnitSystem current_system = ITagT::system;
    static constexpr const char *current_symbol = ITagT::symbol();
    static constexpr detail::UnitSystem angle_system = RTagT::system;
    static constexpr const char *angle_symbol = RTagT::symbol();
  };

public:
  template <typename L2, typename M2, typename T2, typename I2, typename R2>
  static constexpr bool same_dimensions() {
    return detail::fractions_equal_v<L, L2> &&
           detail::fractions_equal_v<M, M2> &&
           detail::fractions_equal_v<T, T2> &&
           detail::fractions_equal_v<I, I2> && detail::fractions_equal_v<R, R2>;
  }

  template <typename L2, typename M2, typename T2, typename I2, typename R2>
  static constexpr bool is_dimensionless() {
    return detail::fraction_is_zero_v<L2> && detail::fraction_is_zero_v<M2> &&
           detail::fraction_is_zero_v<T2> && detail::fraction_is_zero_v<I2> &&
           detail::fraction_is_zero_v<R2>;
  }

  constexpr Unit() : base_value_(0.0) {}
  constexpr explicit Unit(double v) : base_value_(v * factor) {}

  constexpr double to_base() const { return base_value_; }

  static constexpr Unit from_base(double base) {
    Unit u;
    u.base_value_ = base;
    return u;
  }

  template <typename Ratio>
  void __internal_concat_dim(
      std::ostringstream &oss, const char *symbol) const {
    if constexpr (Ratio::num != 0) {
      if constexpr (Ratio::num == 1 && Ratio::den == 1)
        oss << symbol << " ";
      else if constexpr (Ratio::den != 1)
        oss << symbol << "^" << Ratio::num << "/" << Ratio::den << " ";
      else
        oss << symbol << "^" << Ratio::num << " ";
    }
  }

  std::string dims() const {
    std::ostringstream body_stream;

    if constexpr (!detail::fraction_is_zero_v<L_exp>) {
      __internal_concat_dim<L_exp>(body_stream, L_tag::symbol());
    }
    if constexpr (!detail::fraction_is_zero_v<M_exp>) {
      __internal_concat_dim<M_exp>(body_stream, M_tag::symbol());
    }
    if constexpr (!detail::fraction_is_zero_v<T_exp>) {
      __internal_concat_dim<T_exp>(body_stream, T_tag::symbol());
    }
    if constexpr (!detail::fraction_is_zero_v<I_exp>) {
      __internal_concat_dim<I_exp>(body_stream, I_tag::symbol());
    }
    if constexpr (!detail::fraction_is_zero_v<R_exp>) {
      __internal_concat_dim<R_exp>(body_stream, R_tag::symbol());
    }

    std::string body = body_stream.str();
    if (!body.empty()) { body.pop_back(); }
    return body;
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2,
      typename = std::enable_if_t<same_dimensions<L2, M2, T2, I2, R2>(), int>>
  constexpr Unit(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o)
      : base_value_(o.to_base()) {
    using __pdcsu_units_lhs_dims [[maybe_unused]] =
        DimensionSummary<L_exp, M_exp, T_exp, I_exp, R_exp>;
    using __pdcsu_units_rhs_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    using __pdcsu_units_lhs_tags [[maybe_unused]] =
        TagSummary<L_tag, M_tag, T_tag, I_tag, R_tag>;
    using __pdcsu_units_rhs_tags [[maybe_unused]] =
        TagSummary<LTag2, MTag2, TTag2, ITag2, RTag2>;
    static_assert(same_dimensions<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit conversion requires matching (L,M,T,I,R); compare "
        "__pdcsu_units_lhs_dims vs __pdcsu_units_rhs_dims and "
        "__pdcsu_units_lhs_tags vs __pdcsu_units_rhs_tags.");
  }

  constexpr double value() const { return base_value_ / factor; }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr Unit operator+(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) const {
    using __pdcsu_units_lhs_dims [[maybe_unused]] =
        DimensionSummary<L_exp, M_exp, T_exp, I_exp, R_exp>;
    using __pdcsu_units_rhs_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    using __pdcsu_units_lhs_tags [[maybe_unused]] =
        TagSummary<L_tag, M_tag, T_tag, I_tag, R_tag>;
    using __pdcsu_units_rhs_tags [[maybe_unused]] =
        TagSummary<LTag2, MTag2, TTag2, ITag2, RTag2>;
    static_assert(same_dimensions<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit::operator+: requires matching (L,M,T,I,R); compare "
        "__pdcsu_units_lhs_dims vs __pdcsu_units_rhs_dims and "
        "__pdcsu_units_lhs_tags vs __pdcsu_units_rhs_tags.");
    return Unit::from_base(base_value_ + o.to_base());
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr Unit operator-(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) const {
    using __pdcsu_units_lhs_dims [[maybe_unused]] =
        DimensionSummary<L_exp, M_exp, T_exp, I_exp, R_exp>;
    using __pdcsu_units_rhs_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    using __pdcsu_units_lhs_tags [[maybe_unused]] =
        TagSummary<L_tag, M_tag, T_tag, I_tag, R_tag>;
    using __pdcsu_units_rhs_tags [[maybe_unused]] =
        TagSummary<LTag2, MTag2, TTag2, ITag2, RTag2>;
    static_assert(same_dimensions<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit::operator-: requires matching (L,M,T,I,R); compare "
        "__pdcsu_units_lhs_dims vs __pdcsu_units_rhs_dims and "
        "__pdcsu_units_lhs_tags vs __pdcsu_units_rhs_tags.");
    return Unit::from_base(base_value_ - o.to_base());
  }

  constexpr Unit operator-() const { return Unit::from_base(-base_value_); }

  constexpr Unit operator*(double s) const {
    return Unit::from_base(base_value_ * s);
  }
  constexpr Unit operator/(double s) const {
    return Unit::from_base(base_value_ / s);
  }

  constexpr Unit operator*=(double s) {
    base_value_ *= s;
    return *this;
  }

  constexpr Unit operator/=(double s) {
    base_value_ /= s;
    return *this;
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr Unit &operator+=(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) {
    using __pdcsu_units_lhs_dims [[maybe_unused]] =
        DimensionSummary<L_exp, M_exp, T_exp, I_exp, R_exp>;
    using __pdcsu_units_rhs_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    using __pdcsu_units_lhs_tags [[maybe_unused]] =
        TagSummary<L_tag, M_tag, T_tag, I_tag, R_tag>;
    using __pdcsu_units_rhs_tags [[maybe_unused]] =
        TagSummary<LTag2, MTag2, TTag2, ITag2, RTag2>;
    static_assert(same_dimensions<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit::operator+= requires matching (L,M,T,I,R); compare "
        "__pdcsu_units_lhs_dims vs __pdcsu_units_rhs_dims and "
        "__pdcsu_units_lhs_tags vs __pdcsu_units_rhs_tags.");
    base_value_ += o.to_base();
    return *this;
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr Unit &operator-=(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) {
    using __pdcsu_units_lhs_dims [[maybe_unused]] =
        DimensionSummary<L_exp, M_exp, T_exp, I_exp, R_exp>;
    using __pdcsu_units_rhs_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    using __pdcsu_units_lhs_tags [[maybe_unused]] =
        TagSummary<L_tag, M_tag, T_tag, I_tag, R_tag>;
    using __pdcsu_units_rhs_tags [[maybe_unused]] =
        TagSummary<LTag2, MTag2, TTag2, ITag2, RTag2>;
    static_assert(same_dimensions<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit::operator-= requires matching (L,M,T,I,R); compare "
        "__pdcsu_units_lhs_dims vs __pdcsu_units_rhs_dims and "
        "__pdcsu_units_lhs_tags vs __pdcsu_units_rhs_tags.");
    base_value_ -= o.to_base();
    return *this;
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr Unit &operator*=(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) {
    using __pdcsu_units_scaling_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    static_assert(is_dimensionless<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit::operator*= requires the scaling unit to be "
        "dimensionless (L = M = T = I = R = 0). Inspect "
        "__pdcsu_units_scaling_dims for exponent details.");
    base_value_ *= o.to_base();
    return *this;
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr Unit &operator/=(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) {
    using __pdcsu_units_scaling_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    static_assert(is_dimensionless<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit::operator/= requires the scaling unit to be "
        "dimensionless (L = M = T = I = R = 0). Inspect "
        "__pdcsu_units_scaling_dims for exponent details.");
    base_value_ /= o.to_base();
    return *this;
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr auto operator*(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) const {
    using ResultLTag = detail::combine_dim_tag_t<L_exp, L2, LTag, LTag2,
        detail::LengthMixedTag>;
    using ResultMTag =
        detail::combine_dim_tag_t<M_exp, M2, MTag, MTag2, detail::MassMixedTag>;
    using ResultTTag =
        detail::combine_dim_tag_t<T_exp, T2, TTag, TTag2, detail::TimeMixedTag>;
    using ResultITag = detail::combine_dim_tag_t<I_exp, I2, ITag, ITag2,
        detail::CurrentMixedTag>;
    using ResultRTag = detail::combine_dim_tag_t<R_exp, R2, RTag, RTag2,
        detail::AngleMixedTag>;
    using ResultUnit = Unit<detail::fraction_multiply_t<Fac, Fac2>,
        detail::fraction_add_t<L, L2>, detail::fraction_add_t<M, M2>,
        detail::fraction_add_t<T, T2>, detail::fraction_add_t<I, I2>,
        detail::fraction_add_t<R, R2>, ResultLTag, ResultMTag, ResultTTag,
        ResultITag, ResultRTag>;
    return ResultUnit::from_base(base_value_ * o.to_base());
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr auto operator/(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) const {
    using ResultLTag = detail::combine_dim_tag_t<L_exp,
        detail::fraction_negate_t<L2>, LTag, LTag2, detail::LengthMixedTag>;
    using ResultMTag = detail::combine_dim_tag_t<M_exp,
        detail::fraction_negate_t<M2>, MTag, MTag2, detail::MassMixedTag>;
    using ResultTTag = detail::combine_dim_tag_t<T_exp,
        detail::fraction_negate_t<T2>, TTag, TTag2, detail::TimeMixedTag>;
    using ResultITag = detail::combine_dim_tag_t<I_exp,
        detail::fraction_negate_t<I2>, ITag, ITag2, detail::CurrentMixedTag>;
    using ResultRTag = detail::combine_dim_tag_t<R_exp,
        detail::fraction_negate_t<R2>, RTag, RTag2, detail::AngleMixedTag>;
    using ResultUnit = Unit<detail::fraction_divide_t<Fac, Fac2>,
        detail::fraction_subtract_t<L, L2>, detail::fraction_subtract_t<M, M2>,
        detail::fraction_subtract_t<T, T2>, detail::fraction_subtract_t<I, I2>,
        detail::fraction_subtract_t<R, R2>, ResultLTag, ResultMTag, ResultTTag,
        ResultITag, ResultRTag>;
    return ResultUnit::from_base(base_value_ / o.to_base());
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr bool operator==(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) const {
    using __pdcsu_units_lhs_dims [[maybe_unused]] =
        DimensionSummary<L_exp, M_exp, T_exp, I_exp, R_exp>;
    using __pdcsu_units_rhs_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    using __pdcsu_units_lhs_tags [[maybe_unused]] =
        TagSummary<L_tag, M_tag, T_tag, I_tag, R_tag>;
    using __pdcsu_units_rhs_tags [[maybe_unused]] =
        TagSummary<LTag2, MTag2, TTag2, ITag2, RTag2>;
    static_assert(same_dimensions<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit comparison (==) requires matching (L,M,T,I,R); "
        "compare __pdcsu_units_lhs_dims vs __pdcsu_units_rhs_dims and "
        "__pdcsu_units_lhs_tags vs __pdcsu_units_rhs_tags.");
    return base_value_ == o.to_base();
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr bool operator!=(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) const {
    using __pdcsu_units_lhs_dims [[maybe_unused]] =
        DimensionSummary<L_exp, M_exp, T_exp, I_exp, R_exp>;
    using __pdcsu_units_rhs_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    using __pdcsu_units_lhs_tags [[maybe_unused]] =
        TagSummary<L_tag, M_tag, T_tag, I_tag, R_tag>;
    using __pdcsu_units_rhs_tags [[maybe_unused]] =
        TagSummary<LTag2, MTag2, TTag2, ITag2, RTag2>;
    static_assert(same_dimensions<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit comparison (!=) requires matching (L,M,T,I,R); "
        "compare __pdcsu_units_lhs_dims vs __pdcsu_units_rhs_dims and "
        "__pdcsu_units_lhs_tags vs __pdcsu_units_rhs_tags.");
    return base_value_ != o.to_base();
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr bool operator<(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) const {
    using __pdcsu_units_lhs_dims [[maybe_unused]] =
        DimensionSummary<L_exp, M_exp, T_exp, I_exp, R_exp>;
    using __pdcsu_units_rhs_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    using __pdcsu_units_lhs_tags [[maybe_unused]] =
        TagSummary<L_tag, M_tag, T_tag, I_tag, R_tag>;
    using __pdcsu_units_rhs_tags [[maybe_unused]] =
        TagSummary<LTag2, MTag2, TTag2, ITag2, RTag2>;
    static_assert(same_dimensions<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit comparison (<) requires matching (L,M,T,I,R); "
        "compare __pdcsu_units_lhs_dims vs __pdcsu_units_rhs_dims and "
        "__pdcsu_units_lhs_tags vs __pdcsu_units_rhs_tags.");
    return base_value_ < o.to_base();
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr bool operator<=(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) const {
    using __pdcsu_units_lhs_dims [[maybe_unused]] =
        DimensionSummary<L_exp, M_exp, T_exp, I_exp, R_exp>;
    using __pdcsu_units_rhs_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    using __pdcsu_units_lhs_tags [[maybe_unused]] =
        TagSummary<L_tag, M_tag, T_tag, I_tag, R_tag>;
    using __pdcsu_units_rhs_tags [[maybe_unused]] =
        TagSummary<LTag2, MTag2, TTag2, ITag2, RTag2>;
    static_assert(same_dimensions<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit comparison (<=) requires matching (L,M,T,I,R); "
        "compare __pdcsu_units_lhs_dims vs __pdcsu_units_rhs_dims and "
        "__pdcsu_units_lhs_tags vs __pdcsu_units_rhs_tags.");
    return base_value_ <= o.to_base();
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr bool operator>(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) const {
    using __pdcsu_units_lhs_dims [[maybe_unused]] =
        DimensionSummary<L_exp, M_exp, T_exp, I_exp, R_exp>;
    using __pdcsu_units_rhs_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    using __pdcsu_units_lhs_tags [[maybe_unused]] =
        TagSummary<L_tag, M_tag, T_tag, I_tag, R_tag>;
    using __pdcsu_units_rhs_tags [[maybe_unused]] =
        TagSummary<LTag2, MTag2, TTag2, ITag2, RTag2>;
    static_assert(same_dimensions<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit comparison (>) requires matching (L,M,T,I,R); "
        "compare __pdcsu_units_lhs_dims vs __pdcsu_units_rhs_dims and "
        "__pdcsu_units_lhs_tags vs __pdcsu_units_rhs_tags.");
    return base_value_ > o.to_base();
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr bool operator>=(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) const {
    using __pdcsu_units_lhs_dims [[maybe_unused]] =
        DimensionSummary<L_exp, M_exp, T_exp, I_exp, R_exp>;
    using __pdcsu_units_rhs_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    using __pdcsu_units_lhs_tags [[maybe_unused]] =
        TagSummary<L_tag, M_tag, T_tag, I_tag, R_tag>;
    using __pdcsu_units_rhs_tags [[maybe_unused]] =
        TagSummary<LTag2, MTag2, TTag2, ITag2, RTag2>;
    static_assert(same_dimensions<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit comparison (>=) requires matching (L,M,T,I,R); "
        "compare __pdcsu_units_lhs_dims vs __pdcsu_units_rhs_dims and "
        "__pdcsu_units_lhs_tags vs __pdcsu_units_rhs_tags.");
    return base_value_ >= o.to_base();
  }

  template <typename Fac2, typename L2, typename M2, typename T2, typename I2,
      typename R2, typename LTag2, typename MTag2, typename TTag2,
      typename ITag2, typename RTag2>
  constexpr Unit operator%(
      const Unit<Fac2, L2, M2, T2, I2, R2, LTag2, MTag2, TTag2, ITag2, RTag2>
          &o) const {
    using __pdcsu_units_lhs_dims [[maybe_unused]] =
        DimensionSummary<L_exp, M_exp, T_exp, I_exp, R_exp>;
    using __pdcsu_units_rhs_dims [[maybe_unused]] =
        DimensionSummary<L2, M2, T2, I2, R2>;
    using __pdcsu_units_lhs_tags [[maybe_unused]] =
        TagSummary<L_tag, M_tag, T_tag, I_tag, R_tag>;
    using __pdcsu_units_rhs_tags [[maybe_unused]] =
        TagSummary<LTag2, MTag2, TTag2, ITag2, RTag2>;
    static_assert(same_dimensions<L2, M2, T2, I2, R2>(),
        "pdcsu::units::Unit::operator% requires matching (L,M,T,I,R); compare "
        "__pdcsu_units_lhs_dims vs __pdcsu_units_rhs_dims and "
        "__pdcsu_units_lhs_tags vs __pdcsu_units_rhs_tags.");
    return Unit(std::fmod(value(), o.value()));
  }

  constexpr Unit operator%(int s) const {
    return Unit(std::fmod(value(), static_cast<double>(s)));
  }

  constexpr Unit operator%(double s) const {
    return Unit(std::fmod(value(), s));
  }

  friend constexpr Unit operator*(double lhs, const Unit &rhs) {
    return Unit::from_base(lhs * rhs.to_base());
  }

  friend constexpr auto operator/(double lhs, const Unit &rhs) {
    return Unit<detail::Fraction<1>>::from_base(lhs) / rhs;
  }
};

template <typename U1, typename U2>
inline constexpr bool is_same_dimension_v =
    detail::fractions_equal_v<typename U1::L_exp, typename U2::L_exp> &&
    detail::fractions_equal_v<typename U1::M_exp, typename U2::M_exp> &&
    detail::fractions_equal_v<typename U1::T_exp, typename U2::T_exp> &&
    detail::fractions_equal_v<typename U1::I_exp, typename U2::I_exp> &&
    detail::fractions_equal_v<typename U1::R_exp, typename U2::R_exp>;

/* Compound Unit Creator */

template <typename U1, typename U2>
using UnitCompound =
    Unit<detail::fraction_multiply_t<typename U1::Fac_, typename U2::Fac_>,
        detail::fraction_add_t<typename U1::L_exp, typename U2::L_exp>,
        detail::fraction_add_t<typename U1::M_exp, typename U2::M_exp>,
        detail::fraction_add_t<typename U1::T_exp, typename U2::T_exp>,
        detail::fraction_add_t<typename U1::I_exp, typename U2::I_exp>,
        detail::fraction_add_t<typename U1::R_exp, typename U2::R_exp>,
        detail::combine_dim_tag_t<typename U1::L_exp, typename U2::L_exp,
            typename U1::L_tag, typename U2::L_tag, detail::LengthMixedTag>,
        detail::combine_dim_tag_t<typename U1::M_exp, typename U2::M_exp,
            typename U1::M_tag, typename U2::M_tag, detail::MassMixedTag>,
        detail::combine_dim_tag_t<typename U1::T_exp, typename U2::T_exp,
            typename U1::T_tag, typename U2::T_tag, detail::TimeMixedTag>,
        detail::combine_dim_tag_t<typename U1::I_exp, typename U2::I_exp,
            typename U1::I_tag, typename U2::I_tag, detail::CurrentMixedTag>,
        detail::combine_dim_tag_t<typename U1::R_exp, typename U2::R_exp,
            typename U1::R_tag, typename U2::R_tag, detail::AngleMixedTag>>;

/* Divided Unit Creator */
template <typename U1, typename U2>
using UnitDivision =
    Unit<detail::fraction_divide_t<typename U1::Fac_, typename U2::Fac_>,
        detail::fraction_subtract_t<typename U1::L_exp, typename U2::L_exp>,
        detail::fraction_subtract_t<typename U1::M_exp, typename U2::M_exp>,
        detail::fraction_subtract_t<typename U1::T_exp, typename U2::T_exp>,
        detail::fraction_subtract_t<typename U1::I_exp, typename U2::I_exp>,
        detail::fraction_subtract_t<typename U1::R_exp, typename U2::R_exp>,
        detail::combine_dim_tag_t<typename U1::L_exp,
            detail::fraction_negate_t<typename U2::L_exp>, typename U1::L_tag,
            typename U2::L_tag, detail::LengthMixedTag>,
        detail::combine_dim_tag_t<typename U1::M_exp,
            detail::fraction_negate_t<typename U2::M_exp>, typename U1::M_tag,
            typename U2::M_tag, detail::MassMixedTag>,
        detail::combine_dim_tag_t<typename U1::T_exp,
            detail::fraction_negate_t<typename U2::T_exp>, typename U1::T_tag,
            typename U2::T_tag, detail::TimeMixedTag>,
        detail::combine_dim_tag_t<typename U1::I_exp,
            detail::fraction_negate_t<typename U2::I_exp>, typename U1::I_tag,
            typename U2::I_tag, detail::CurrentMixedTag>,
        detail::combine_dim_tag_t<typename U1::R_exp,
            detail::fraction_negate_t<typename U2::R_exp>, typename U1::R_tag,
            typename U2::R_tag, detail::AngleMixedTag>>;

/* Base Unit Aliases */
using scalar_t = Unit<detail::Fraction<1>>;
using meter_t = Unit<detail::Fraction<1>, detail::Fraction<1>>;
using foot_t = Unit<detail::Fraction<3048, 10000>, detail::Fraction<1>,
    detail::Fraction<0>, detail::Fraction<0>, detail::Fraction<0>,
    detail::Fraction<0>, detail::LengthFootTag>;
using inch_t = Unit<detail::Fraction<254, 10000>, detail::Fraction<1>,
    detail::Fraction<0>, detail::Fraction<0>, detail::Fraction<0>,
    detail::Fraction<0>, detail::LengthInchTag>;
using kg_t =
    Unit<detail::Fraction<1>, detail::Fraction<0>, detail::Fraction<1>>;
using pound_t = Unit<detail::Fraction<4535924, 10000000>, detail::Fraction<0>,
    detail::Fraction<1>, detail::Fraction<0>, detail::Fraction<0>,
    detail::Fraction<0>, detail::LengthMetricTag, detail::MassPoundTag>;
using second_t = Unit<detail::Fraction<1>, detail::Fraction<0>,
    detail::Fraction<0>, detail::Fraction<1>>;
using minute_t =
    Unit<detail::Fraction<60>, detail::Fraction<0>, detail::Fraction<0>,
        detail::Fraction<1>, detail::Fraction<0>, detail::Fraction<0>,
        detail::LengthMetricTag, detail::MassMetricTag, detail::TimeMinuteTag>;
using ms_t = Unit<detail::Fraction<1, 1000>, detail::Fraction<0>,
    detail::Fraction<0>, detail::Fraction<1>, detail::Fraction<0>,
    detail::Fraction<0>, detail::LengthMetricTag, detail::MassMetricTag,
    detail::TimeMillisecondTag>;
using amp_t = Unit<detail::Fraction<1>, detail::Fraction<0>,
    detail::Fraction<0>, detail::Fraction<0>, detail::Fraction<1>>;
using radian_t =
    Unit<detail::Fraction<1>, detail::Fraction<0>, detail::Fraction<0>,
        detail::Fraction<0>, detail::Fraction<0>, detail::Fraction<1>>;
using degree_t = Unit<detail::Fraction<1745329, 100000000>, detail::Fraction<0>,
    detail::Fraction<0>, detail::Fraction<0>, detail::Fraction<0>,
    detail::Fraction<1>, detail::LengthMetricTag, detail::MassMetricTag,
    detail::TimeSecondTag, detail::CurrentAmpTag, detail::AngleDegreeTag>;
using rotation_t = Unit<detail::Fraction<62832, 10000>, detail::Fraction<0>,
    detail::Fraction<0>, detail::Fraction<0>, detail::Fraction<0>,
    detail::Fraction<1>, detail::LengthMetricTag, detail::MassMetricTag,
    detail::TimeSecondTag, detail::CurrentAmpTag, detail::AngleRotationTag>;

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
    typename R, typename LTag, typename MTag, typename TTag, typename ITag,
    typename RTag>
static inline Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag> u_abs(
    const Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag> &u) {
  return Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag>::from_base(
      std::abs(u.to_base()));
}

// Power operation
template <typename Fac, typename L, typename M, typename T, typename I,
    typename R, typename LTag, typename MTag, typename TTag, typename ITag,
    typename RTag>
constexpr auto u_pow(
    const Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag> &u,
    double exp) {
  return Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag>::from_base(
      std::pow(u.to_base(), exp));
}

// Floor operation
template <typename Fac, typename L, typename M, typename T, typename I,
    typename R, typename LTag, typename MTag, typename TTag, typename ITag,
    typename RTag>
static inline Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag> u_floor(
    const Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag> &u) {
  return Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag>::from_base(
      std::floor(u.to_base()));
}

// Ceiling operation
template <typename Fac, typename L, typename M, typename T, typename I,
    typename R, typename LTag, typename MTag, typename TTag, typename ITag,
    typename RTag>
static inline Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag> u_ceil(
    const Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag> &u) {
  return Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag>::from_base(
      std::ceil(u.to_base()));
}

// Round
template <typename Fac, typename L, typename M, typename T, typename I,
    typename R, typename LTag, typename MTag, typename TTag, typename ITag,
    typename RTag>
static inline Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag> u_round(
    const Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag> &u) {
  return Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag>::from_base(
      std::round(u.to_base()));
}

// Square root
template <typename Fac, typename L, typename M, typename T, typename I,
    typename R, typename LTag, typename MTag, typename TTag, typename ITag,
    typename RTag>
constexpr auto u_sqrt(
    const Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag> &u) {
  using ResultL = detail::fraction_multiply_t<L, detail::Fraction<1, 2>>;
  using ResultM = detail::fraction_multiply_t<M, detail::Fraction<1, 2>>;
  using ResultT = detail::fraction_multiply_t<T, detail::Fraction<1, 2>>;
  using ResultI = detail::fraction_multiply_t<I, detail::Fraction<1, 2>>;
  using ResultR = detail::fraction_multiply_t<R, detail::Fraction<1, 2>>;
  using ResultUnit = Unit<Fac, ResultL, ResultM, ResultT, ResultI, ResultR,
      LTag, MTag, TTag, ITag, RTag>;
  const double sqrt_factor = std::sqrt(Fac::num * 1.0 / Fac::den);
  return ResultUnit::from_base(std::sqrt(u.to_base()) * sqrt_factor);
}

// Copysign
template <typename U1, typename U2> constexpr auto u_copysign(U1 u, U2 sign) {
  static_assert(
      std::is_base_of<
          Unit<typename U1::Fac_, typename U1::L_exp, typename U1::M_exp,
              typename U1::T_exp, typename U1::I_exp, typename U1::R_exp,
              typename U1::L_tag, typename U1::M_tag, typename U1::T_tag,
              typename U1::I_tag, typename U1::R_tag>,
          U1>::value,
      "u_copysign requires the first argument to be a pdcsu::units::Unit "
      "type.");
  static_assert(
      std::is_base_of<
          Unit<typename U2::Fac_, typename U2::L_exp, typename U2::M_exp,
              typename U2::T_exp, typename U2::I_exp, typename U2::R_exp,
              typename U2::L_tag, typename U2::M_tag, typename U2::T_tag,
              typename U2::I_tag, typename U2::R_tag>,
          U2>::value,
      "u_copysign requires the second argument to be a pdcsu::units::Unit "
      "type.");
  return Unit<typename U1::Fac_, typename U1::L_exp, typename U1::M_exp,
      typename U1::T_exp, typename U1::I_exp, typename U1::R_exp,
      typename U1::L_tag, typename U1::M_tag, typename U1::T_tag,
      typename U1::I_tag, typename U1::R_tag>(
      std::copysign(u.value(), sign.value()));
}
template <typename Fac, typename L, typename M, typename T, typename I,
    typename R, typename LTag, typename MTag, typename TTag, typename ITag,
    typename RTag>
auto u_copysign(const Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag> &u,
    double sign) {
  return Unit<Fac, L, M, T, I, R, LTag, MTag, TTag, ITag, RTag>::from_base(
      std::copysign(u.to_base(), sign));
}

// Clamp
template <typename FacU, typename FacMin, typename FacMax, typename L,
    typename M, typename T, typename I, typename R, typename LTagU,
    typename MTagU, typename TTagU, typename ITagU, typename RTagU,
    typename LTagMin, typename MTagMin, typename TTagMin, typename ITagMin,
    typename RTagMin, typename LTagMax, typename MTagMax, typename TTagMax,
    typename ITagMax, typename RTagMax>
Unit<FacU, L, M, T, I, R, LTagU, MTagU, TTagU, ITagU, RTagU> u_clamp(
    const Unit<FacU, L, M, T, I, R, LTagU, MTagU, TTagU, ITagU, RTagU> &u,
    const Unit<FacMin, L, M, T, I, R, LTagMin, MTagMin, TTagMin, ITagMin,
        RTagMin> &min,
    const Unit<FacMax, L, M, T, I, R, LTagMax, MTagMax, TTagMax, ITagMax,
        RTagMax> &max) {
  auto base = u.to_base();
  const auto min_base = min.to_base();
  const auto max_base = max.to_base();
  if (base < min_base) {
    base = min_base;
  } else if (base > max_base) {
    base = max_base;
  }
  return Unit<FacU, L, M, T, I, R, LTagU, MTagU, TTagU, ITagU,
      RTagU>::from_base(base);
}

// Min/Max
template <typename FacA, typename FacB, typename L, typename M, typename T,
    typename I, typename R, typename LTagA, typename MTagA, typename TTagA,
    typename ITagA, typename RTagA, typename LTagB, typename MTagB,
    typename TTagB, typename ITagB, typename RTagB>
constexpr Unit<FacA, L, M, T, I, R, LTagA, MTagA, TTagA, ITagA, RTagA> u_min(
    const Unit<FacA, L, M, T, I, R, LTagA, MTagA, TTagA, ITagA, RTagA> &a,
    const Unit<FacB, L, M, T, I, R, LTagB, MTagB, TTagB, ITagB, RTagB> &b) {
  return (a.to_base() < b.to_base())
             ? a
             : Unit<FacA, L, M, T, I, R, LTagA, MTagA, TTagA, ITagA, RTagA>(b);
}

template <typename FacA, typename FacB, typename L, typename M, typename T,
    typename I, typename R, typename LTagA, typename MTagA, typename TTagA,
    typename ITagA, typename RTagA, typename LTagB, typename MTagB,
    typename TTagB, typename ITagB, typename RTagB>
constexpr Unit<FacA, L, M, T, I, R, LTagA, MTagA, TTagA, ITagA, RTagA> u_max(
    const Unit<FacA, L, M, T, I, R, LTagA, MTagA, TTagA, ITagA, RTagA> &a,
    const Unit<FacB, L, M, T, I, R, LTagB, MTagB, TTagB, ITagB, RTagB> &b) {
  return (a.to_base() > b.to_base())
             ? a
             : Unit<FacA, L, M, T, I, R, LTagA, MTagA, TTagA, ITagA, RTagA>(b);
}

// Trigonometric functions
static inline double u_sin(const radian_t &a) { return std::sin(a.to_base()); }
static inline double u_cos(const radian_t &a) { return std::cos(a.to_base()); }
static inline double u_tan(const radian_t &a) { return std::tan(a.to_base()); }
static inline double u_tanh(const radian_t &a) {
  return std::tanh(a.to_base());
}

static inline double u_sin(const degree_t &a) { return std::sin(a.to_base()); }
static inline double u_cos(const degree_t &a) { return std::cos(a.to_base()); }
static inline double u_tan(const degree_t &a) { return std::tan(a.to_base()); }
static inline double u_tanh(const degree_t &a) {
  return std::tanh(a.to_base());
}

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
