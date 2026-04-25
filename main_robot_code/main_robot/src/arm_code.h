#ifndef ARM_IK_H
#define ARM_IK_H

  #include <eigen3/Eigen/Dense>

  // // ************************************************************************************************

  // double constexpr l1 = 85.5e-3;   // [m]
  // double constexpr l2 = 95.5e-3;   // [m]
  // double constexpr l3 = 61.0e-3;   // [m]

  // // ************************************************************************************************

  // namespace Eigen {
  //   template<typename T> using Vector2 = Eigen::Matrix<T,2,1>;
  //   template<typename T> using Vector3 = Eigen::Matrix<T,3,1>;
  //   template<typename T> using Affine2 = Eigen::Transform<T,2,Eigen::Affine>;
  //   template<typename T> using Translation2 = Eigen::Translation<T,2>;
  //   template<typename T> using Matrix2 = Eigen::Matrix<T,2,2>;
  // }

  // template<typename T>
  // Eigen::Affine2<T> exp(Eigen::Vector3<T> const&);

  // template<typename T>
  // Eigen::Affine2<T> exp(
  //   T const& tx,      // [m]
  //   T const& ty,      // [m]
  //   T const& theta);  // [rad]

  // template<typename T>
  // Eigen::Vector3<T> log(Eigen::Affine2<T> const&);

  // template<int N, int M>
  // Eigen::Matrix<double,M,N> pinv(Eigen::Matrix<double,N,M> const&);

  // template<typename T>
  // Eigen::Affine2<T> get_arm_transform(T const* x);

  // template<typename T>
  // T mod(T const theta_rad);

  // bool solve_inverse_kinematics(
  //   Eigen::Affine2d const& pt,
  //   double* x);

#endif
