#include <iostream>
#include <limits>

#include <DH_transform.hpp>

namespace kinematics {

//--------------------------------------------------------------------------------------------------
// Standalone maths functions
//--------------------------------------------------------------------------------------------------

Eigen::Matrix3d get_skew_symmetric_matrix(
  Eigen::Vector3d const& w)
{
  Eigen::Matrix3d S;
  S <<    0., -w.z(),  w.y(),
       w.z(),     0., -w.x(),
      -w.y(),  w.x(),     0.;
  return S;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> get_left_SE3_product_jacobian(
  Eigen::Affine3d const& T1,
  Eigen::Affine3d const& T2)
{
  Eigen::Matrix3d const& R1 = T1.rotation();
  Eigen::Vector3d const& t2 = T2.translation();
  Eigen::Matrix<double,6,6> J = Eigen::Matrix<double,6,6>::Identity();
  J.block<3,3>(3,0) = - get_skew_symmetric_matrix(R1*t2);
  return J;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> get_right_SE3_product_jacobian(
    Eigen::Affine3d const& T1,
    Eigen::Affine3d const& T2)
{
  Eigen::Matrix3d const& R1 = T1.rotation();
  Eigen::Matrix<double,6,6> J = Eigen::Matrix<double,6,6>::Identity();
  J.block<3,3>(0,0) = R1;
  J.block<3,3>(3,3) = R1;
  return J;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix3d get_left_SO3_jacobian(
  Eigen::Vector3d const& theta)
{
  double angle_rad = theta.norm();
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  if (angle_rad==0) return J;
  Eigen::Matrix3d const S = get_skew_symmetric_matrix(theta);
  J += ((1-std::cos(angle_rad))/std::pow(angle_rad,2))*S;
  J += ((angle_rad-std::sin(angle_rad))/std::pow(angle_rad,3))*S*S;
  return J;
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d exp_map(Eigen::Vector3d const& rotation, Eigen::Vector3d const& translation)
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  Eigen::Vector3d const axis = rotation.normalized();
  double const angle = rotation.norm();
  Eigen::AngleAxisd const angle_axis(angle,axis);
  T.rotate(angle_axis);
  T.translation() = translation;
  return T;
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d exp_map(Eigen::Matrix<double,6,1> const& tau)
{
  Eigen::Vector3d const rotation = tau.head<3>();
  Eigen::Vector3d const translation = tau.tail<3>();
  return exp_map(rotation, translation);
}

//--------------------------------------------------------------------------------------------------

Eigen::Vector3d log_map(Eigen::Quaterniond const& q)
{
  double q_norm = q.norm();
  if( q_norm < 1-1e-3 || q_norm > 1+1e-3)
    throw("Unormalized quaternion");
  Eigen::AngleAxisd angle_axis(q);
  return angle_axis.angle() * angle_axis.axis();
}

//--------------------------------------------------------------------------------------------------

Eigen::Vector3d log_map(Eigen::Matrix3d const& R)
{
  Eigen::Quaterniond const q(R);
  return log_map(q);
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,1> log_map(Eigen::Affine3d const& T)
{
  Eigen::Matrix<double,6,1> tau = Eigen::Matrix<double,6,1>::Zero();
  tau.tail<3>() = T.translation();
  tau.head<3>() = log_map(T.rotation());

  return tau;
}

//--------------------------------------------------------------------------------------------------
// Constructors and destructor
//--------------------------------------------------------------------------------------------------

DHTransform::DHTransform(double d1, double a1, double d2, double a2)
: parameters_ {d1, a1, d2, a2},
  lower_bounds_ {-std::numeric_limits<double>::infinity()*Eigen::Vector4d::Ones()},
  upper_bounds_ { std::numeric_limits<double>::infinity()*Eigen::Vector4d::Ones()}
{
  this->compute_transform_and_jacobian();
}

//--------------------------------------------------------------------------------------------------

DHTransform::DHTransform(Eigen::Vector4d const& parameters)
: DHTransform(parameters(kd1), parameters(ka1), parameters(kd2), parameters(ka2))
{}

//--------------------------------------------------------------------------------------------------

DHTransform::DHTransform()
: DHTransform(0,0,0,0)
{}

//--------------------------------------------------------------------------------------------------

DHTransformVector::DHTransformVector(std::initializer_list<DHTransform> const& transforms)
: std::vector<DHTransform>(transforms)
{}

//--------------------------------------------------------------------------------------------------
// Public functions
//--------------------------------------------------------------------------------------------------

Eigen::Affine3d const& DHTransform::get_transform() const
{
  return this->transform_;
}

//--------------------------------------------------------------------------------------------------

Eigen::Vector4d const& DHTransform::get_parameters() const
{
  return this->parameters_;
}

//--------------------------------------------------------------------------------------------------

DHTransform::Parameters DHTransform::get_free_parameters() const
{
  return free_parameters_;
}

//--------------------------------------------------------------------------------------------------

int DHTransformVector::get_num_free_parameters() const
{
  int number = 0;
  for(DHTransform const& T : *this)
    number += T.get_num_free_parameters();
  return number;
}

//--------------------------------------------------------------------------------------------------

int DHTransform::get_num_free_parameters() const
{
  return static_cast<int>(this->free_parameters_.size());
}

//--------------------------------------------------------------------------------------------------

void DHTransform::update_parameters(double dd1, double da1, double dd2, double da2)
{
  // Update parameters (with modulo pi)
  this->parameters_[kd1] += dd1;
  this->parameters_[ka1] = std::fmod(this->parameters_[ka1] + da1, 2 * M_PI);
  this->parameters_[kd2] += dd2;
  this->parameters_[ka2] = std::fmod(this->parameters_[ka2] + da2, 2 * M_PI);

  // Compute the new global transformation
  this->compute_transform_and_jacobian();
}

//--------------------------------------------------------------------------------------------------

void DHTransform::update_parameters(Eigen::Vector4d const& delta_params)
{
  return this->update_parameters(
    delta_params(kd1),
    delta_params(ka1),
    delta_params(kd2),
    delta_params(ka2));
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,4> const& DHTransform::get_full_jacobian_matrix() const
{
  return this->jacobian_;
}

//--------------------------------------------------------------------------------------------------

void DHTransform::set_parameter(Parameter name, double value)
{
  // Update the parameter
  int idx = this->get_index_from_parameter(name);
  switch(name)
  {
    // If angle value, apply modulo pi
    case Parameter::a1:
    case Parameter::a2:
      this->parameters_[idx] = std::fmod(value, 2 * M_PI);
      break;
    // Otherwise, don't worry
    case Parameter::d1:
    case Parameter::d2:
      this->parameters_[idx] = value;
      break;
  }

  // Recompute the global transform
  this->compute_transform_and_jacobian();
}

//--------------------------------------------------------------------------------------------------

void DHTransform::set_parameter_lower_bound(Parameter name, double value)
{
  int const idx = this->get_index_from_parameter(name);
  this->lower_bounds_[idx] = value;
  if(this->lower_bounds_[idx] > this->upper_bounds_[idx])
    throw std::runtime_error("Inconsistent lower and upper bound");
}

//--------------------------------------------------------------------------------------------------

void DHTransform::set_parameter_upper_bound(Parameter name, double value)
{
  int const idx = this->get_index_from_parameter(name);
  this->upper_bounds_[idx] = value;
  if(this->lower_bounds_[idx] > this->upper_bounds_[idx])
    throw std::runtime_error("Inconsistent lower and upper bound");
}

//--------------------------------------------------------------------------------------------------

bool DHTransform::get_parameters_back_within_bounds()
{
  bool all_parameters_are_within_bounds = true;
  for(Parameter parameter_name : this->free_parameters_)
  {
    int const idx = this->get_index_from_parameter(parameter_name);

    // CHeck the lower bound
    if(this->parameters_[idx] <= this->lower_bounds_[idx])
    {
      this->parameters_[idx] = this->lower_bounds_[idx];
      this->set_parameter_free(parameter_name, false);
      all_parameters_are_within_bounds = false;
    }

    // Check the upper bound
    else if(this->parameters_[idx] >= this->upper_bounds_[idx])
    {
      this->parameters_[idx] = this->upper_bounds_[idx];
      this->set_parameter_free(parameter_name, false);
      all_parameters_are_within_bounds = false;
    }
  }

  return all_parameters_are_within_bounds;
}

//--------------------------------------------------------------------------------------------------

bool DHTransformVector::get_parameters_back_within_bounds()
{
  bool all_parameters_are_within_bounds = true;
  for(DHTransform& T : *this)
    all_parameters_are_within_bounds &= T.get_parameters_back_within_bounds();
  return all_parameters_are_within_bounds;
}

//--------------------------------------------------------------------------------------------------

void DHTransform::set_parameters(Eigen::Vector4d const& params)
{
  this->parameters_ = params;
  this->compute_transform_and_jacobian();
}

//--------------------------------------------------------------------------------------------------

double DHTransform::get_parameter(Parameter name)
{
  int const idx = this->get_index_from_parameter(name);
  return this->parameters_[idx];
}

//--------------------------------------------------------------------------------------------------

void DHTransform::set_parameter_free(Parameter name, bool is_free)
{
  if(is_free)
    this->free_parameters_.insert(name);
  else
    this->free_parameters_.erase(name);
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d DHTransformVector::get_global_transform() const
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  for(DHTransform const& Ti : *this)
    T = T * Ti.get_transform();
  return T;
}

//--------------------------------------------------------------------------------------------------

int DHTransformVector::optimize_parameters(
  ProblemType problem_type, double const* const* args,
  bool use_contraints)
{
  // Set the problem
  int num_iters = 0;
  switch(problem_type)
  {

    case ProblemType::FullPose:
    {
      // Initialize default problem functions
      std::function<Vector6d(Eigen::Affine3d const&)> get_residual_vector;
      std::function<Matrix6Xd(Eigen::Affine3d const&, Matrix6Xd const&)> get_residual_jacobian;

      // Get the problem parameters
      if(args[0] == nullptr) throw std::runtime_error("Missing data");
      Eigen::Map<Eigen::Matrix4d const> T_matrix(args[0]);
      Eigen::Affine3d const T_target(T_matrix);

      // Build the problem functions
      get_residual_vector = [&](Eigen::Affine3d const& T){
          return log_map(T_target * T.inverse());};
      get_residual_jacobian = [&](Eigen::Affine3d const&, Matrix6Xd const& J_T_p){
          return J_T_p;};

      // Solve the problem (without inequality constraints for now)
      bool all_parameters_within_bounds = false;
      while(!all_parameters_within_bounds)
      {
        num_iters += this->optimize_parameters(get_residual_vector, get_residual_jacobian);
        all_parameters_within_bounds = this->get_parameters_back_within_bounds();
        std::cout << "All parameters within bounds: " << all_parameters_within_bounds << std::endl;
      }
      break;
    }

    case ProblemType::PositionDirection:
    {
      // Initialize default problem functions
      std::function<Vector4d(Eigen::Affine3d const&)> get_residual_vector;
      std::function<Matrix4Xd(Eigen::Affine3d const&, Matrix6Xd const&)> get_residual_jacobian;

      // Get the problem parameters
      if(args[0]==nullptr || args[1]==nullptr)
        throw std::runtime_error("Missing data");
      Eigen::Map<Eigen::Vector3d const> t_target(args[0]);
      Eigen::Map<Eigen::Vector3d const> ux_target(args[1]);

      // Build the problem functions

      get_residual_vector = [&](Eigen::Affine3d const& T){
        Eigen::Vector4d residual = Eigen::Vector4d::Zero();
        Eigen::Vector3d const t = T.translation();
        residual.head<3>() = t - t_target;
        Eigen::Vector3d const ux = T.rotation().col(0);
        residual(3) = ux.dot(ux_target) - 1.0;
        return residual;
      };

      get_residual_jacobian = [&](Eigen::Affine3d const& T, Matrix6Xd const& J_T_p){
        int const num_free_parameters = J_T_p.cols();
        Eigen::Vector3d const ux = T.rotation().col(0);
        Matrix4Xd J_g_p = Matrix4Xd::Zero(4,num_free_parameters);
        J_g_p.block(0,0,3,num_free_parameters) = J_T_p.block(3,0,3,num_free_parameters);
        J_g_p.block(3,0,1,num_free_parameters) =
          - ux_target.transpose()
          * get_skew_symmetric_matrix(ux)
          * J_T_p.block(0,0,3,num_free_parameters);
         return J_g_p;
      };

      // Solve the problem (without inequality constraints for now)
      bool all_parameters_within_bounds = false;
      while(!all_parameters_within_bounds)
      {
        num_iters += this->optimize_parameters(get_residual_vector, get_residual_jacobian);
        all_parameters_within_bounds = this->get_parameters_back_within_bounds();
      }
      break;
    }

    default:
      std::runtime_error("Unknown problem type.");
  }

  return num_iters;
}

//--------------------------------------------------------------------------------------------------

std::string DHTransform::print() const
{
  std::stringstream out;
  out << "Parameters: ("
      << "d1=" << this->parameters_[kd1] << " ("
      << (this->free_parameters_.count(Parameter::d1)>0 ? "free" : "fixed") << "), "
      << "a1=" << this->parameters_[ka1] << " ("
      << (this->free_parameters_.count(Parameter::a1)>0 ? "free" : "fixed") << "), "
      << "d2=" << this->parameters_[kd2] << " ("
      << (this->free_parameters_.count(Parameter::d2)>0 ? "free" : "fixed") << "), "
      << "a2=" << this->parameters_[ka2] << " ("
      << (this->free_parameters_.count(Parameter::a2)>0 ? "free" : "fixed") << ")\n";
  out << "Lower and upper bounds: "
        << "d1: lb=" << this->lower_bounds_[kd1]
        << "<=" << this->parameters_[kd1]
        << "<=" << this->upper_bounds_[kd1]
        << ", a1: lb=" << this->lower_bounds_[ka1]
        << "<=" << this->parameters_[ka1]
        << "<=" << this->upper_bounds_[ka1]
        << ", d2: lb=" << this->lower_bounds_[kd2]
        << "<=" << this->parameters_[kd2]
        << "<=" << this->upper_bounds_[kd2]
        << ", a2: lb=" << this->lower_bounds_[ka2]
        << "<=" << this->parameters_[ka2]
        << "<=" << this->upper_bounds_[ka2] << "\n";
  out << "Transform:\n" << this->transform_.matrix();
  return out.str();
}

//--------------------------------------------------------------------------------------------------

std::string DHTransformVector::print() const
{
  std::stringstream out;
  out << "Robotical arm:" << std::endl;
  out << "Successive transforms:\n\n";
  for(DHTransform const& T : *this)
    out << std::endl << T.print() << std::endl;
  out << "\n\nGlobal transform:\n" << this->get_global_transform().matrix() << std::endl;
  return out.str();
}

//--------------------------------------------------------------------------------------------------
// Private functions
//--------------------------------------------------------------------------------------------------

int DHTransform::get_index_from_parameter(Parameter name) const
{
  int idx = -1;
  switch(name)
  {
    case Parameter::d1:
      idx = kd1;
      break;
    case Parameter::a1:
      idx = ka1;
      break;
    case Parameter::d2:
      idx = kd2;
      break;
    case Parameter::a2:
      idx = ka2;
      break;
    default:
      throw std::runtime_error("Unknown parameter name");
  }
  if(idx<0) throw std::runtime_error("Invalid index");
  return idx;
}

//--------------------------------------------------------------------------------------------------

DHTransform::Parameter DHTransform::get_parameter_from_idx(int idx) const
{
  switch(idx)
  {
    case 0:
      return Parameter::d1;
    case 1:
      return Parameter::a1;
    case 2:
      return Parameter::d2;
    case 3:
      return Parameter::a2;
    default:
      throw std::runtime_error("Invalid parameter index");
  }
}

//--------------------------------------------------------------------------------------------------

void DHTransform::compute_transform_and_jacobian()
{
  // Get the current parameters
  double const d1 = this->parameters_(kd1);
  double const a1 = this->parameters_(ka1);
  double const d2 = this->parameters_(kd2);
  double const a2 = this->parameters_(ka2);

  // Compute the first transform
  Eigen::Affine3d const T1 =
      Eigen::Translation3d(d1*Eigen::Vector3d::UnitX())
    * Eigen::Quaterniond(Eigen::AngleAxisd(a1, Eigen::Vector3d::UnitX()));

  // Compute the second transform
  Eigen::Affine3d const T2 =
      Eigen::Translation3d(d2*Eigen::Vector3d::UnitZ())
    * Eigen::Quaterniond(Eigen::AngleAxisd(a2, Eigen::Vector3d::UnitZ()));

  // Compute the global transform
  this->transform_ = T1 * T2;

  // Jacobian wrt. first set of params
  Eigen::Matrix<double,6,6> const J_T_T1 = get_left_SE3_product_jacobian(T1,T2);
  Eigen::Matrix<double,6,1> J_T1_a1 = Eigen::Matrix<double,6,1>::Zero();
  J_T1_a1.segment<3>(0) = get_left_SO3_jacobian(a1*Eigen::Vector3d::UnitX()).col(kwx);
  this->jacobian_.col(kd1) = J_T_T1.col(ktx);
  this->jacobian_.col(ka1) = J_T_T1 * J_T1_a1;

  // Jacobian wrt. second set of params
  Eigen::Matrix<double,6,6> const J_T_T2 = get_right_SE3_product_jacobian(T1,T2);
  Eigen::Matrix<double,6,1> J_T2_a2 = Eigen::Matrix<double,6,1>::Zero();
  J_T2_a2.segment<3>(0) = get_left_SO3_jacobian(a2*Eigen::Vector3d::UnitZ()).col(kwz);
  this->jacobian_.col(kd2) = J_T_T2.col(ktz);
  this->jacobian_.col(ka2) = J_T_T2 * J_T2_a2;
}

//--------------------------------------------------------------------------------------------------

DHTransformVector::Matrix6Xd DHTransformVector::get_free_jacobian_matrix() const
{
  // Principle of the computation of the jacobian of the global pose wrt. angle parameters
  // -------------------------------------------------------------------------------------
  // |  J_T1_p1   ||  J_T2_p2   ||  J_T3_p3   || J_T4_p4   | -> step 1
  // | *J_T12_T1  || *J_T12_T2  ||            ||           | -> step 2
  // | *J_T13_T12 || *J_T13_T12 || *J_T13_T3  ||           | -> step 3
  // | *J_T14_T13 || *J_T14_T13 || *J_T14_T13 || *J_T14_T4 | -> step 4

  // Get and check dimensions
  int const num_free_parameters = this->get_num_free_parameters();
  int const num_poses = static_cast<int>(this->size());

  // Initialize jacobian matrices
  int col_idx=0;
  Matrix6Xd J_T_a = Matrix6Xd::Zero(6,num_free_parameters);
  for(int pose_idx=0; pose_idx<num_poses; pose_idx+=1)
  {
    DHTransform const& T = (*this)[pose_idx];
    Matrix64d const& J_T_p = T.get_full_jacobian_matrix();
    for(DHTransform::Parameter parameter : T.get_free_parameters())
    {
      J_T_a.col(col_idx) = J_T_p.col(static_cast<int>(parameter));
      col_idx += 1;
    }
  }
  if(col_idx != num_free_parameters)
    throw std::runtime_error("1) Inconsistent indexes in building the free jacobian "
      + std::to_string(col_idx) + " != " + std::to_string(num_free_parameters));

  // Make forward pass

  Eigen::Affine3d T0im1 = Eigen::Affine3d::Identity();
  for(int pose_idx=0; pose_idx<num_poses; pose_idx+=1)
  {
    // Compute the jacobians
    Eigen::Affine3d const& Ti = (*this)[pose_idx].get_transform();
    Eigen::Matrix<double,6,6> const J_T_T0im1 = get_left_SE3_product_jacobian(T0im1,Ti);
    Eigen::Matrix<double,6,6> const J_T_Ti = get_right_SE3_product_jacobian(T0im1,Ti);

    // Update all the jacobians
    col_idx=0;
    Eigen::Matrix<double,6,6> J = J_T_T0im1;
    for(int pose_jdx=0; pose_jdx<=pose_idx; pose_jdx+=1)
    {
      if(pose_jdx==pose_idx) J = J_T_Ti;
      int const num_params = (*this)[pose_jdx].get_num_free_parameters();
      J_T_a.block(0,col_idx,6,num_params) = J * J_T_a.block(0,col_idx,6,num_params);
      col_idx += num_params;
    }

    // Update the pose
    T0im1 = T0im1 * Ti;
  }

  return J_T_a;
}

//--------------------------------------------------------------------------------------------------
// Other functions
//--------------------------------------------------------------------------------------------------

DHTransformVector create_main_robot_arm()
{
  // Robotical arm configuration
  double constexpr d01x = 70.6e-3;
  double constexpr d12x = 76.5e-3;
  double constexpr a12x = M_PI_2;
  double constexpr d23x = 70.6e-3;
  double constexpr d34x = 90.6e-3;
  double constexpr d45x = 29.1e-3;
  double constexpr a45x = -M_PI_2;
  double constexpr d45z = 45.8e-3;

  // Build the robotical arm
  kinematics::DHTransform T01(d01x,0,0,0);
  kinematics::DHTransform T12(d12x,a12x,0,0);
  kinematics::DHTransform T23(d23x,0,0,0);
  kinematics::DHTransform T34(d34x,0,0,0);
  kinematics::DHTransform T45(d45x,a45x,d45z,0);
  T01.set_parameter_free(DHTransform::Parameter::a2, true);
  T01.set_parameter_lower_bound(DHTransform::Parameter::a2, -M_PI_2);
  T01.set_parameter_upper_bound(DHTransform::Parameter::a2, M_PI_2);
  std::cout << T01.print() << std::endl;

  T12.set_parameter_free(DHTransform::Parameter::a2, true);
  T12.set_parameter_lower_bound(DHTransform::Parameter::a2, -M_PI_2);
  T12.set_parameter_upper_bound(DHTransform::Parameter::a2, M_PI_2);
  std::cout << T12.print() << std::endl;

  T23.set_parameter_free(DHTransform::Parameter::a2, true);
  T23.set_parameter_lower_bound(DHTransform::Parameter::a2, -M_PI_2);
  T23.set_parameter_upper_bound(DHTransform::Parameter::a2, 0);
  std::cout << T23.print() << std::endl;

  T34.set_parameter_free(DHTransform::Parameter::a2, true);
  T34.set_parameter_lower_bound(DHTransform::Parameter::a2, -M_PI_2);
  T34.set_parameter_upper_bound(DHTransform::Parameter::a2, M_PI_2);
  std::cout << T34.print() << std::endl;

  kinematics::DHTransformVector arm{T01,T12,T23,T34,T45};
  return arm;
}

//--------------------------------------------------------------------------------------------------

} // namespace kinematics
