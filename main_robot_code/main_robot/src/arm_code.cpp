#include <vector>

#include <eigen3/Eigen/Dense>

#include "dual.hpp"
#include "arm_code.h"

//--------------------------------------------------------------------------------------------------

  // ************************************************************************************************

  double constexpr l1 = 85.5e-3;   // [m]
  double constexpr l2 = 95.5e-3;   // [m]
  double constexpr l3 = 61.0e-3;   // [m]

  // ************************************************************************************************

  namespace Eigen {
    template<typename T> using Vector2 = Eigen::Matrix<T,2,1>;
    template<typename T> using Vector3 = Eigen::Matrix<T,3,1>;
    template<typename T> using Affine2 = Eigen::Transform<T,2,Eigen::Affine>;
    template<typename T> using Translation2 = Eigen::Translation<T,2>;
    template<typename T> using Matrix2 = Eigen::Matrix<T,2,2>;
  }

  template<typename T>
  Eigen::Affine2<T> exp(Eigen::Vector3<T> const&);

  template<typename T>
  Eigen::Affine2<T> exp(
    T const& tx,      // [m]
    T const& ty,      // [m]
    T const& theta);  // [rad]

  template<typename T>
  Eigen::Vector3<T> log(Eigen::Affine2<T> const&);

  template<int N, int M>
  Eigen::Matrix<double,M,N> pinv(Eigen::Matrix<double,N,M> const&);

  template<typename T>
  Eigen::Affine2<T> get_arm_transform(T const* x);

  template<typename T>
  T mod(T const theta_rad);

  bool solve_inverse_kinematics(
    Eigen::Affine2d const& pt,
    double* x);


template<typename T>
inline Eigen::Affine2<T> exp(Eigen::Vector3<T> const& x)
{
  // Get translation and rotation parts
  T const tx    = x(0);
  T const ty    = x(1);
  T const theta = x(2);

  // Return the final transformation
  T const cos_theta = cos(theta);
  T const sin_theta = sin(theta);
  Eigen::Affine2<T> Tf;
  Tf.translation() << tx, ty;
  Tf.linear() << cos_theta, -sin_theta,
                 sin_theta,  cos_theta;
  Tf.makeAffine();
  return Tf;
}

//--------------------------------------------------------------------------------------------------

template<typename T>
Eigen::Affine2<T> exp(
  T const& tx,
  T const& ty,
  T const& theta)
{
  Eigen::Vector3<T> const r{tx,ty,theta};
  return exp(r);
}

//--------------------------------------------------------------------------------------------------

template<typename T>
inline Eigen::Vector3<T> log(Eigen::Affine2<T> const& Tf)
{
  Eigen::Vector3<T> r;
  r(0) = Tf.translation().x();
  r(1) = Tf.translation().y();
  Eigen::Matrix2<T> const R = Tf.linear();
  r(2) = atan2(R(1,0),R(0,0));
  return r;
}

//--------------------------------------------------------------------------------------------------

template<int M,int N>
Eigen::Matrix<double,N,M> pinv(Eigen::Matrix<double,M,N> const& X)
{
  // Perform SVD decomposition
  Eigen::JacobiSVD<Eigen::Matrix<double,M,N>> svd(
    X, Eigen::ComputeFullU | Eigen::ComputeFullV);

  int constexpr DMIN = (M<N) ? M : N;
  Eigen::Matrix<double,N,M> inv_S = Eigen::Matrix<double,N,M>::Zero();
  Eigen::Matrix<double,DMIN,1> const s = svd.singularValues();
  assert( s.size() == DMIN );
  for(int i=0; i<DMIN; i++) if(s(i)>1e-15) inv_S(i,i) = 1./s(i);

  // Compute the pseudo-inverse
  Eigen::Matrix<double,N,M> const pinv_X =
    svd.matrixV() * inv_S * svd.matrixU().transpose();
  return pinv_X;
}

//--------------------------------------------------------------------------------------------------

template<typename T>
inline Eigen::Affine2<T> get_arm_transform(T const* x)
{
  Eigen::Affine2<T> const T1 = exp(T(0.),T(0.),x[0]);
  Eigen::Affine2<T> const T2 = exp(T(l1),T(0.),x[1]);
  Eigen::Affine2<T> const T3 = exp(T(l2),T(0.),x[2]);
  Eigen::Affine2<T> const T4 = exp(T(l3),T(0.),T(0.));
  return T1*T2*T3*T4;
}

//--------------------------------------------------------------------------------------------------

template<typename T>
T mod(T const theta_rad)
{
  return atan2(sin(theta_rad),cos(theta_rad));
}

//--------------------------------------------------------------------------------------------------

inline bool solve_inverse_kinematics(
  Eigen::Affine2d const& Tt,
  double* x)
{
  // Get the initial position
  int constexpr N = 3;
  using DualN = Dual<N>;
  using VectorNd = Eigen::Matrix<double,N,1>;
  Eigen::Affine2<DualN> const inv_Tt = Tt.inverse().cast<DualN>();
  Eigen::Map<VectorNd> xk(x);

  // Run the optimization
  int iter_idx = 0;
  int constexpr max_iters = 20;
  while (iter_idx<max_iters)
  {
    // Get the current parameters
    std::vector<DualN> xk_dual(N);
    for(int i=0; i<N; i++) xk_dual[i] = DualN(xk(i),i);

    // Get the current transform and compute the error
    // [NOTE] T(x) = Tt*exp(err) => err = log(T(x)*inv_Tt)
    Eigen::Affine2<DualN> const T_arm = get_arm_transform(xk_dual.data());
    Eigen::Affine2<DualN> const Te = T_arm * inv_Tt;
    Eigen::Vector3<DualN> e = log(Te);

    // Check the error
    bool ok = true;
    ok &= (std::fabs(e(0).val)<1e-3);
    ok &= (std::fabs(e(1).val)<1e-3);
    ok &= (std::fabs(e(2).val)<M_PI/180.);
    if(ok){
      //std::cout << "\t=> e = " << e.cast<double>().transpose() << std::endl;
      xk(0) = mod(xk(0));
      xk(1) = mod(xk(1));
      xk(2) = mod(xk(2));
      return true;
    }

    // Get the Jacobian matrix
    Eigen::Matrix<double,3,N> Je;
    for(int i=0; i<3; i++)
      for(int j=0; j<N; j++)
        Je(i,j) = e(i).grad(j);
    Eigen::Matrix<double,N,3> const pinv_Je = pinv(Je);

    // Update the current solution
    // [NOTE] Newton-Raphson iteration:
    // 0 = e + J*dx <=> min || J*dx - (-e) ||
    //              <=> dx = -(J+)*e
    VectorNd const dx = -pinv_Je*(e.cast<double>());
    xk = xk + dx;
    iter_idx += 1;
  }

  return false;
}

// // ************************************************************************************************

// int main(int argc, char* argv[])
// {
//   /*// Set the initial guess
//   Eigen::Vector3d const rt{l1+l2,-l3,-M_PI_2};
//   Eigen::Affine2d const Tt = exp(rt);
//   Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
//   solve_inverse_kinematics(Tt,x.data());
//   printf("xf_opt = %.3f,%.3f,%.3f\n",x(0),x(1),x(2));*/

//   // Go from initial to final position
//   double constexpr RAD = M_PI/180.;
//   double constexpr DEG = 1.0/RAD;
//   Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
//   Eigen::Vector3d const r0{l1+l2,-l3,-M_PI_2};
//   Eigen::Vector3d const rf{l1,-(l2+l3),-M_PI_2};
//   for(double c=0.; c<=1.0; c+=1e-2){
//     Eigen::Vector3d const rt = (1.0-c)*r0 + c*rf;
//     Eigen::Affine2d const Tt = exp(rt);
//     if(solve_inverse_kinematics(Tt,x.data())){
//       printf("xf_opt[%.2f] = %.3f,%.3f,%.3f\n",
//         c,x(0)*DEG,x(1)*DEG,x(2)*DEG);
//     } else {
//       printf("xf_opt[%.2f] = FAILED!\n",c);
//     }
//     continue;
//   }

//   return EXIT_SUCCESS;
// }

// // ************************************************************************************************