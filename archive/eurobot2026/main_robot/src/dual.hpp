#ifndef DUAL_NUMBER_HPP
#define DUAL_NUMBER_HPP

#include <iostream>
#include <vector>

#include <eigen3/Eigen/Dense>

// ---------------------------------------------------------------------
// Declaration of the Dual structure
// ---------------------------------------------------------------------

template <int N>
struct Dual {

  public:
    Dual(double const& a = 0.0)
      : val(a){ grad.setConstant(double(0.0)); }
    Dual(double const& a, int k)
      : Dual(a){ grad[k]=double(1.0); }
    Dual(double const& a, Eigen::Matrix<double,N,1> const& v)
      : val(a), grad(v) {}
    Dual& operator=(Dual const& other) = default;
    Dual(Dual const& other) = default;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  public:

    // Cast operators
    
    operator double() const
    {
      return double(this->val);
    }

    // Compound operators

    Dual<N>& operator+=(Dual<N> const& y) {
      *this = *this + y;
      return *this;
    }

    Dual<N>& operator-=(Dual<N> const& y) {
      *this = *this - y;
      return *this;
    }

    Dual<N>& operator*=(Dual<N> const& y) {
      *this = *this * y;
      return *this;
    }

    Dual<N>& operator/=(Dual<N> const& y) {
      *this = *this / y;
      return *this;
    }

    // Compound with scalar operators.

    Dual<N>& operator+=(double const& s) {
      *this = *this + s;
      return *this;
    }

    Dual<N>& operator-=(double const& s) {
      *this = *this - s;
      return *this;
    }

    Dual<N>& operator*=(double const& s) {
      *this = *this * s;
      return *this;
    }

    Dual<N>& operator/=(double const& s) {
      *this = *this / s;
      return *this;
    }
  
  public:
    double val;
    Eigen::Matrix<double,N,1> grad;

}; // struct Dual

namespace Eigen {
  template<int N> using VectorXDual = Eigen::Matrix<Dual<N>,Eigen::Dynamic,1>;
}

template <int N>
std::ostream& operator<<(std::ostream& os, Dual<N> const& a)
{
  os << a.val;
  for(int i=0; i<N; i++) os << " + " << a.grad(i) << "\u0395" << i;
  return os;
}

// ---------------------------------------------------------------------
// Declaration of unary operators
// ---------------------------------------------------------------------

// Unary +
template <int N>
inline Dual<N> const& operator+(Dual<N> const& f) {
  return f;
}

// Unary -
template <int N>
inline Dual<N> operator-(Dual<N> const& f) {
  return Dual<N>(-f.val, -f.grad);
}

// ---------------------------------------------------------------------
// Declaration of binary operators
// ---------------------------------------------------------------------

// Binary +
template <int N>
inline Dual<N> operator+(Dual<N> const& f, Dual<N> const& g) {
  return Dual<N>(f.val + g.val, f.grad + g.grad);
}

// Binary + with a scalar: x + s
template <int N>
inline Dual<N> operator+(Dual<N> const& f, double s) {
  return Dual<N>(f.val + s, f.grad);
}

// Binary + with a scalar: s + x
template <int N>
inline Dual<N> operator+(double s, Dual<N> const& f) {
  return Dual<N>(f.val + s, f.grad);
}

// Binary -
template <int N>
inline Dual<N> operator-(Dual< N> const& f, Dual<N> const& g) {
  return Dual<N>(f.val - g.val, f.grad - g.grad);
}

// Binary - with a scalar: x - s
template <int N>
inline Dual<N> operator-(Dual<N> const& f, double s) {
  return Dual<N>(f.val - s, f.grad);
}

// Binary - with a scalar: s - x
template <int N>
inline Dual<N> operator-(double s, Dual<N> const& f) {
  return Dual<N>(s - f.val, -f.grad);
}

// Binary *
template <int N>
inline Dual<N> operator*(Dual<N> const& f, Dual<N> const& g) {
  return Dual<N>(f.val * g.val, f.val * g.grad + f.grad * g.val);
}

// Binary * with a scalar: x * s
template <int N>
inline Dual<N> operator*(Dual<N> const& f, double s) {
  return Dual<N>(f.val * s, f.grad * s);
}

// Binary * with a scalar: s * x
template <int N>
inline Dual<N> operator*(double s, Dual<N> const& f) {
  return Dual<N>(f.val * s, f.grad * s);
}

// Binary /
template <int N>
inline Dual<N> operator/(Dual<N> const& f, Dual<N> const& g) {
  double const g_a_inverse = double(1.0) / g.val;
  double const f_a_by_g_a = f.val * g_a_inverse;
  return Dual<N>(f_a_by_g_a, (f.grad - f_a_by_g_a*g.grad)*g_a_inverse);
}

// Binary / with a scalar: s / x
template <int N>
inline Dual<N> operator/(double s, Dual<N> const& g) {
  double const minus_s_g_a_inverse2 = -s / (g.val * g.val);
  return Dual<N>(s / g.val, g.grad * minus_s_g_a_inverse2);
}

// Binary / with a scalar: x / s
template <int N>
inline Dual<N> operator/(Dual<N> const& f, double s) {
  double const s_inverse = double(1.0) / s;
  return Dual<N>(f.val * s_inverse, f.grad * s_inverse);
}

// ---------------------------------------------------------------------
// Binary comparison
// ---------------------------------------------------------------------

#define DEFINE_DUAL_COMPARISON_OPERATOR(op)                             \
  template <int N>                                          \
  constexpr bool operator op(Dual<N> const& f, double const& g) {          \
    return (f.val op g);                                                  \
  }                                                                     \
  template <int N>                                          \
  constexpr bool operator op(double const& f, Dual<N> const& g) {          \
    return (f op g.val);                                                  \
  }                                                                     \
  template <int N>                                          \
  constexpr bool operator op(Dual<N> const& f, Dual<N> const& g) {  \
    return (f.val op g.val);                                                \
  }                                                                     

DEFINE_DUAL_COMPARISON_OPERATOR(<);
DEFINE_DUAL_COMPARISON_OPERATOR(<=);
DEFINE_DUAL_COMPARISON_OPERATOR(>);
DEFINE_DUAL_COMPARISON_OPERATOR(>=);
DEFINE_DUAL_COMPARISON_OPERATOR(==);
DEFINE_DUAL_COMPARISON_OPERATOR(!=);

#undef DEFINE_DUAL_COMPARISON_OPERATOR

// ---------------------------------------------------------------------
// Usual functions
// ---------------------------------------------------------------------

// abs(x + h) ~= abs(x) + sgn(x)h
using std::abs;
template <int N>
inline Dual<N> abs(Dual<N> const& f) {
  return Dual<N>(abs(f.val), copysign(double(1),f.val)*f.grad );
}

// Categorize scalar part as zero, subnormal, normal, infinite, NaN, or implementation-defined.
using std::fpclassify;
template <int N>
inline int fpclassify(Dual<N> const& f) {
  return fpclassify(f.val);
}

// copysign(a + da, b + db) ~= sgn(b)|a| + (sgn(a)sgn(b) da + 2|a|δ(b) db)
using std::copysign;
template <int N>
inline Dual<N> copysign(Dual<N> const& f, const Dual<N> g) {
  double d = fpclassify(g) == FP_ZERO ? std::numeric_limits<double>::infinity() : double(0);
  double sa = copysign(double(1), f.val);  // sgn(a)
  double sb = copysign(double(1), g.val);  // sgn(b)
  return Dual<N>(copysign(f.val, g.val), sa * sb * f.grad + abs(f.val) * d * g.grad);
}

// log(a + h) ~= log(a) + h / a
using std::log;
template <int N>
inline Dual<N> log(Dual<N> const& f) {
  double const a_inverse = double(1.0) / f.val;
  return Dual<N>(log(f.val), f.grad * a_inverse);
}

// log10(a + h) ~= log10(a) + h / (a log(10))
using std::log10;
template <int N>
inline Dual<N> log10(Dual<N> const& f) {
  double const a_inverse = double(1.0) / (f.val * log(double(10.0)));
  return Dual<N>(log10(f.val), f.grad * a_inverse);
}

// log1p(a + h) ~= log1p(a) + h / (1 + a)
using std::log1p;
template <int N>
inline Dual<N> log1p(Dual<N> const& f) {
  double const a_inverse = double(1.0) / (double(1.0) + f.val);
  return Dual<N>(log1p(f.val), f.grad * a_inverse);
}

// exp(a + h) ~= exp(a) + exp(a) h
using std::exp;
template <int N>
inline Dual<N> exp(Dual<N> const& f) {
  double const tmp = exp(f.val);
  return Dual<N>(tmp, tmp * f.grad);
}

// expm1(a + h) ~= expm1(a) + exp(a) h
using std::expm1;
template <int N>
inline Dual<N> expm1(Dual<N> const& f) {
  double const tmp = expm1(f.val);
  double const expa = tmp + double(1.0);  // exp(a) = expm1(a) + 1
  return Dual<N>(tmp, expa * f.grad);
}

// sqrt(a + h) ~= sqrt(a) + h / (2 sqrt(a))
using std::sqrt;
template <int N>
inline Dual<N> sqrt(Dual<N> const& f) {
  double const tmp = sqrt(f.val);
  double const two_a_inverse = double(1.0) / (double(2.0) * tmp);
  return Dual<N>(tmp, f.grad * two_a_inverse);
}

// cos(a + h) ~= cos(a) - sin(a) h
using std::cos;
template <int N>
inline Dual<N> cos(Dual<N> const& f) {
  return Dual<N>(cos(f.val), -sin(f.val) * f.grad);
}

// acos(a + h) ~= acos(a) - 1 / sqrt(1 - a^2) h
using std::acos;
template <int N>
inline Dual<N> acos(Dual<N> const& f) {
  double const tmp = -double(1.0) / sqrt(double(1.0) - f.val * f.val);
  return Dual<N>(acos(f.val), tmp * f.grad);
}

// sin(a + h) ~= sin(a) + cos(a) h
using std::sin;
template <int N>
inline Dual<N> sin(Dual<N> const& f) {
  return Dual<N>(sin(f.val), cos(f.val) * f.grad);
}

// asin(a + h) ~= asin(a) + 1 / sqrt(1 - a^2) h
using std::asin;
template <int N>
inline Dual<N> asin(Dual<N> const& f) {
  double const tmp = double(1.0) / sqrt(double(1.0) - f.val * f.val);
  return Dual<N>(asin(f.val), tmp * f.grad);
}

// tan(a + h) ~= tan(a) + (1 + tan(a)^2) h
using std::tan;
template <int N>
inline Dual<N> tan(Dual<N> const& f) {
  double const tan_a = tan(f.val);
  double const tmp = double(1.0) + tan_a * tan_a;
  return Dual<N>(tan_a, tmp * f.grad);
}

// atan(a + h) ~= atan(a) + 1 / (1 + a^2) h
using std::atan;
template <int N>
inline Dual<N> atan(Dual<N> const& f) {
  double const tmp = double(1.0) / (double(1.0) + f.val * f.val);
  return Dual<N>(atan(f.val), tmp * f.grad);
}

// sinh(a + h) ~= sinh(a) + cosh(a) h
using std::sinh;
template <int N>
inline Dual<N> sinh(Dual<N> const& f) {
  return Dual<N>(sinh(f.val), cosh(f.val) * f.grad);
}

// cosh(a + h) ~= cosh(a) + sinh(a) h
using std::cosh;
template <int N>
inline Dual<N> cosh(Dual<N> const& f) {
  return Dual<N>(cosh(f.val), sinh(f.val) * f.grad);
}

// tanh(a + h) ~= tanh(a) + (1 - tanh(a)^2) h
using std::tanh;
template <int N>
inline Dual<N> tanh(Dual<N> const& f) {
  double const tanh_a = tanh(f.val);
  double const tmp = double(1.0) - tanh_a * tanh_a;
  return Dual<N>(tanh_a, tmp * f.grad);
}

// atan2(b + db, a + da) ~= atan2(b, a) + (- b da + a db) / (a^2 + b^2)
using std::atan2;
template <int N>
inline Dual<N> atan2(Dual<N> const& g, Dual<N> const& f) {
  //   f = a + da
  //   g = b + db
  double const tmp = double(1.0) / (f.val * f.val + g.val * g.val);
  return Dual<N>(atan2(g.val, f.val), tmp * (-g.val * f.grad + f.val * g.grad));
}

// norm(x + h) ~= norm(x) + 2x h
using std::norm;
template <int N>
inline Dual<N> norm(Dual<N> const& f) {
  return Dual<N>(norm(f.val), double(2) * f.val * f.grad);
}

// pow -- base is a differentiable function, exponent is a constant.
// (a+da)^p ~= a^p + p*a^(p-1) da
using std::pow;
template <int N>
inline Dual<N> pow(Dual<N> const& f, double g) {
  double const tmp = g * pow(f.val, g - double(1.0));
  return Dual<N>(pow(f.val, g), tmp * f.grad);
}

// pow -- base is a constant, exponent is a differentiable function.
// We have various special cases, see the comment for pow(Dual, Dual) for
// analysis:
// 1. For f > 0 we have: (f)^(g + dg) ~= f^g + f^g log(f) dg
// 2. For f == 0 and g > 0 we have: (f)^(g + dg) ~= f^g
// 3. For f < 0 and integer g we have: (f)^(g + dg) ~= f^g but if dg
// != 0, the derivatives are not defined and we return NaN.
template <int N>
inline Dual<N> pow(double f, Dual<N> const& g) {
  Dual<N> result;

  if (fpclassify(f) == FP_ZERO && g > 0) {
    // Handle case 2.
    result = Dual<N>(double(0.0));
  } else {
    if (f < 0 && g == floor(g.val)) {  // Handle case 3.
      result = Dual<N>(pow(f, g.val));
      for (int i = 0; i < N; i++) {
        if (fpclassify(g.grad[i]) != FP_ZERO) {
          // Return a NaN when g.grad != 0.
          result.grad[i] = std::numeric_limits<double>::quiet_NaN();
        }
      }
    } else {
      // Handle case 1.
      double const tmp = pow(f, g.val);
      result = Dual<N>(tmp, log(f) * tmp * g.grad);
    }
  }

  return result;
}

// exp2(x + h) = 2^(x+h) ~= 2^x + h*2^x*log(2)
using std::exp2;
template <int N>
inline Dual<N> exp2(Dual<N> const& f) {
  double const tmp = exp2(f.val);
  double const derivative = tmp * log(double(2));
  return Dual<N>(tmp, f.grad * derivative);
}

// log2(x + h) ~= log2(x) + h / (x * log(2))
using std::log2;
template <int N>
inline Dual<N> log2(Dual<N> const& f) {
  double const derivative = double(1.0) / (f.val * log(double(2)));
  return Dual<N>(log2(f.val), f.grad * derivative);
}

// Determines whether the scalar part of the Dual is finite.
using std::isfinite;
template <int N>
inline bool isfinite(Dual<N> const& f) {
  return isfinite(f.val) && f.grad.array().isfinite().all();
}

// Determines whether the scalar part of the Dual is infinite.
using std::isinf;
template <int N>
inline bool isinf(Dual<N> const& f) {
  return isinf(f.val) && f.grad.array().isInf().any();
}

// Determines whether the scalar part of the Dual is NaN.
using std::isnan;
template <int N>
inline bool isnan(Dual<N> const& f) {
  return isnan(f.val) && f.grad.array().isNaN().any();
}

// Determines whether the scalar part of the Dual is neither zero, subnormal,
// infinite, nor NaN.
using std::isnormal;
template <int N>
inline bool isnormal(Dual<N> const& f) {
  return isnormal(f.val) && f.grad.array().isnormal().all();
}

// ---------------------------------------------------------------------

#endif // DUAL_NUMBER_HPP
