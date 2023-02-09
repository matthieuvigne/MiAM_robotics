#ifndef DH_TRANSFORM_HPP
#define DH_TRANSFORM_HPP

#include <eigen3/Eigen/Dense>

namespace kinematics {

//--------------------------------------------------------------------------------------------------

class DHTransform {

public:

  enum class Parameter {d1 = 0, a1 = 1, d2 = 2, a2 = 3}; // enum class parameter

public:

  // Constructor
  DHTransform(double d1, double a1, double d2, double a2);
  DHTransform(Eigen::Vector4d const& parameters);
  DHTransform();

  // Destructor
  virtual ~DHTransform(){};

public:
  
  // Get member values
  Eigen::Affine3d const& get_transform() const;
  Eigen::Vector4d const& get_parameters() const;
  Eigen::Matrix<double,6,2> const& get_rotation_jacobian_matrix() const;

  // Set and get parameters
  double get_parameter(Parameter name);
  void set_parameters(double a1, double a2);
  void boxplus(double delta_a1, double delta_a2);

private:

  void compute_transform_and_jacobian();

private:

  // The order of the parameters matches the order of the operations:
  // 1) Translation of length d1 along the x axis;
  // 2) Rotation of angle a1 around the x axis
  // 1) Translation of length d2 along the z axis;
  // 2) Rotation of angle a2 around the z axis
  enum {kd1, ka1, kd2, ka2};
  enum {kwx, kwy, kwz, ktx, kty, ktz};
  typedef Eigen::Matrix<double,6,2> Matrix62d;
  Eigen::Vector4d parameters_;
  Eigen::Affine3d transform_;
  Matrix62d rotation_jacobian_;

}; // class DHTransform

//--------------------------------------------------------------------------------------------------

class DHTransformVector : public std::vector<DHTransform> {
  
public:
  DHTransformVector() = default;

public:

  Eigen::Affine3d get_global_transform() const;
  int optimize_parameters(Eigen::Affine3d const& T_target);
  
private:

  typedef Eigen::Matrix<double,6,1> Vector6d;
  typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6Xd;
  typedef Eigen::Matrix<double,Eigen::Dynamic,6> MatrixX6d;
  Matrix6Xd get_jacobian_wrt_free_parameters() const;
  
}; // class DHTransformVector

//--------------------------------------------------------------------------------------------------

} // namespace kinematics

#endif // DH_TRANSFORM_HPP
