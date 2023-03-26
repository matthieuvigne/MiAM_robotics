#ifndef DH_TRANSFORM_HPP
#define DH_TRANSFORM_HPP

#include <set>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace kinematics {

//--------------------------------------------------------------------------------------------------
// Typedefs
//--------------------------------------------------------------------------------------------------

enum class Parameter {d1=0, a1=1, d2=2, a2=3};
typedef std::set<Parameter> Parameters;
typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6Xd;
enum class ProblemType {FullPose, PositionDirection};
struct OptimizationResult {bool success = false; int num_iters = 0;};

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class DHTransform {

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
  void set_parameter_free(Parameter name, bool is_free);
  void set_parameter_lower_bound(Parameter name, double value);
  void set_parameter_upper_bound(Parameter name, double value);
  void update_parameters(double d1, double a1, double d2, double a2);
  void update_parameters(Eigen::Vector4d const& delta_params);

  // Freeze parameters which violate constraints
  bool get_parameters_back_within_bounds();

  // Display
  std::string print() const;

private:

  int get_index_from_parameter(Parameter name) const;
  Parameter get_parameter_from_idx(int idx) const;
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
  Eigen::Vector4d lower_bounds_;
  Eigen::Vector4d upper_bounds_;
  std::set<Parameter> free_parameters_;
  Eigen::Affine3d transform_;
  Matrix64d jacobian_;

}; // class DHTransform

//--------------------------------------------------------------------------------------------------

class DHTransformVector : public std::vector<DHTransform> {

public:
  DHTransformVector() = default;
  DHTransformVector(std::initializer_list<DHTransform> const& transforms);
  DHTransformVector(DHTransformVector const& other) = default;

public:

  Eigen::Affine3d get_global_transform() const;
  OptimizationResult optimize_full_pose(Eigen::Affine3d const& T);
  OptimizationResult optimize_position_x_direction(
    Eigen::Vector3d const& target_position,
    Eigen::Vector3d const& target_x);
  std::string print() const;

private:

  // Single-pose jacobian wrt all parameters
  typedef Eigen::Matrix<double,6,4> Matrix64d;

  // Full pose problem
  typedef Eigen::Matrix<double,6,1> Vector6d;
  typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6Xd;
  typedef Eigen::Matrix<double,Eigen::Dynamic,6> MatrixX6d;

  // Position-direction problem
  typedef Eigen::Matrix<double,4,1> Vector4d;
  typedef Eigen::Matrix<double,4,Eigen::Dynamic> Matrix4Xd;
  typedef Eigen::Matrix<double,Eigen::Dynamic,4> MatrixX4d;

  // Position-direction problem (alt)
  typedef Eigen::Matrix<double,5,1> Vector5d;
  typedef Eigen::Matrix<double,5,Eigen::Dynamic> Matrix5Xd;
  typedef Eigen::Matrix<double,Eigen::Dynamic,5> MatrixX5d;

  template <int Nr> using ResidualVector = Eigen::Matrix<double,Nr,1>;
  template<int Nr> using ResidualJacobian = Eigen::Matrix<double,Nr,Eigen::Dynamic>;

private:

  // Free parameters
  int get_num_free_parameters() const;
  Matrix6Xd get_free_jacobian_matrix() const;

  // Check that all parameters are within bounds
  bool get_parameters_back_within_bounds();

  // Generic problem resolution function
  template <int Nr>
  OptimizationResult optimize_parameters(
    std::function<ResidualVector<Nr>(Eigen::Affine3d const&)> const& get_residual,
    std::function<ResidualJacobian<Nr>(Eigen::Affine3d const&, Matrix6Xd const&)> const& get_jacobian);
  OptimizationResult optimize_parameters(ProblemType type, double const* const* args);

}; // class DHTransformVector

//--------------------------------------------------------------------------------------------------
// Static functios
//--------------------------------------------------------------------------------------------------

std::string get_parameter_name(Parameter parameter);
DHTransformVector create_main_robot_arm();

//--------------------------------------------------------------------------------------------------
// Templates functions
//--------------------------------------------------------------------------------------------------

template <int Nr>
OptimizationResult DHTransformVector::optimize_parameters(
  std::function<ResidualVector<Nr>(Eigen::Affine3d const&)> const& get_residual_vector,
  std::function<ResidualJacobian<Nr>(Eigen::Affine3d const&, Matrix6Xd const&)> const& get_residual_jacobian)
{
  // -----------------------------------------------------------------------------------------------
  // Pseudo-inverse method documentation
  // https://homes.cs.washington.edu/~todorov/courses/cseP590/06_JacobianMethods.pdf
  // -----------------------------------------------------------------------------------------------

  // Optimization parameters
  int constexpr max_iters = 500;
  OptimizationResult results;
  double constexpr max_tolerance = 1e-5;
  typedef Eigen::Matrix<double,Nr,1> ResidualVector;
  typedef Eigen::Matrix<double,Nr,Eigen::Dynamic> ResidualJacobian;

  // Run algorithm
  int iter_idx = 1;
  for(; iter_idx<max_iters; iter_idx+=1)
  {
    // Get the global transform and check
    Eigen::Affine3d const T = this->get_global_transform();
    ResidualVector const dT = get_residual_vector(T);
    if(dT.norm() < max_tolerance)
    {
      results.success = true;
      break;
    }

    // Get the pose jacobian wrt optimized parameters
    Matrix6Xd const J_T_p = this->get_free_jacobian_matrix();
    ResidualJacobian J = get_residual_jacobian(T, J_T_p);
    Eigen::MatrixXd const Jp = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXd const dp = Jp * dT;

    // Update each pose individually
    int col_idx=0;
    int const num_poses = static_cast<int>(this->size());
    for(int pose_idx=0; pose_idx<num_poses; pose_idx+=1)
    {
      Eigen::Vector4d delta_params = Eigen::Vector4d::Zero();
      Parameters const& pose_free_params = (*this)[pose_idx].get_free_parameters();
      for(Parameter const& param : pose_free_params)
      {
        delta_params(static_cast<int>(param)) = dp(col_idx);
        col_idx += 1;
      }
      (*this)[pose_idx].update_parameters(delta_params);
    }
  }

  // Return the results
  results.num_iters = iter_idx;
  return results;
}

//--------------------------------------------------------------------------------------------------

} // namespace kinematics

#endif // DH_TRANSFORM_HPP
