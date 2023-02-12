#ifndef DH_TRANSFORM_HPP
#define DH_TRANSFORM_HPP

#include <set>

#include <eigen3/Eigen/Dense>

namespace kinematics {

//--------------------------------------------------------------------------------------------------

class DHTransform {

public:

  enum class Parameter {d1=0, a1=1, d2=2, a2=3};
  typedef std::set<Parameter> Parameters;
  typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6Xd;

public:

  // Constructor
  // d1,a1/x_im1 and d2,d2/z_i
  DHTransform(double d1, double a1, double d2, double a2);
  DHTransform(Eigen::Vector4d const& parameters);
  DHTransform();

  // Destructor
  virtual ~DHTransform(){};

public:
  
  // Get member values
  Parameters get_free_parameters() const;
  int get_num_free_parameters() const;
  Eigen::Vector4d const& get_parameters() const;
  Eigen::Affine3d const& get_transform() const;
  Eigen::Matrix<double,6,4> const& get_full_jacobian_matrix() const;

  // Set and get parameters
  double get_parameter(Parameter name);
  void set_parameter(Parameter name, double value);
  void set_parameters(Eigen::Vector4d const& params);
  void free_parameter(Parameter name, bool is_free);
  void update_parameters(double d1, double a1, double d2, double a2);
  void update_parameters(Eigen::Vector4d const& delta_params);

  // Display
  std::string print() const;

private:

  void compute_transform_and_jacobian();

private:

  // The order of the parameters matches the order of the operations:
  // 1) Translation of length d1 along the x axis;
  // 2) Rotation of angle a1 around the x axis;
  // 1) Translation of length d2 along the z axis;
  // 2) Rotation of angle a2 around the z axis;

  enum {kd1, ka1, kd2, ka2};
  enum {kwx, kwy, kwz, ktx, kty, ktz};
  typedef Eigen::Matrix<double,6,4> Matrix64d;

  Eigen::Vector4d parameters_;
  std::set<Parameter> free_parameters_;
  Eigen::Affine3d transform_;
  Matrix64d jacobian_;

}; // class DHTransform

//--------------------------------------------------------------------------------------------------

class DHTransformVector : public std::vector<DHTransform> {
  
public:
  DHTransformVector() = default;
  DHTransformVector(std::initializer_list<DHTransform> const& transforms);

public:

  Eigen::Affine3d get_global_transform() const;
  int optimize_parameters(Eigen::Affine3d const& T_target);
  std::string print() const;
  
private:

  typedef Eigen::Matrix<double,6,1> Vector6d;
  typedef Eigen::Matrix<double,6,4> Matrix64d;
  typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6Xd;
  typedef Eigen::Matrix<double,Eigen::Dynamic,6> MatrixX6d;
  
private:
  
  int get_num_free_parameters() const;
  Matrix6Xd get_free_jacobian_matrix() const;
  
}; // class DHTransformVector

//--------------------------------------------------------------------------------------------------

} // namespace kinematics

#endif // DH_TRANSFORM_HPP
