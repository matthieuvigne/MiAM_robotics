#include <iostream>

#include <kinematics/DH_transform.hpp>

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

Eigen::Matrix<double,6,1> log_map(Eigen::Affine3d const& T)
{
  // Get the translational part
  Eigen::Matrix<double,6,1> tau = Eigen::Matrix<double,6,1>::Zero();
  tau.tail<3>() = T.translation();

  // Get the quaternion and check it
  Eigen::Quaterniond q(T.rotation());
  double q_norm = q.norm();
  if( q_norm < 1-1e-3 || q_norm > 1+1e-3)
    throw("Unormalized quaternion");
  Eigen::AngleAxisd angle_axis(q);
  tau.head<3>() = angle_axis.angle() * angle_axis.axis();

  return tau;
}

//--------------------------------------------------------------------------------------------------
// Constructors and destructor
//--------------------------------------------------------------------------------------------------

DHTransform::DHTransform(double d1, double a1, double d2, double a2)
: parameters_ {d1, a1, d2, a2}
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
  this->parameters_[kd1] += dd1;
  this->parameters_[ka1] += da1;
  this->parameters_[kd2] += dd2;
  this->parameters_[ka2] += da2;
  this->compute_transform_and_jacobian();
}

//--------------------------------------------------------------------------------------------------

void DHTransform::update_parameters(Eigen::Vector4d const& delta_params)
{
  this->parameters_ += delta_params;
  this->compute_transform_and_jacobian();
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,4> const& DHTransform::get_full_jacobian_matrix() const
{
  return this->jacobian_;
}

//--------------------------------------------------------------------------------------------------

void DHTransform::set_parameter(Parameter name, double value)
{
  switch(name)
  {
    case Parameter::d1:
      this->parameters_[kd1] = value;
      break;
    case Parameter::a1:
      this->parameters_[ka1] = value;
      break;
    case Parameter::d2:
      this->parameters_[kd2] = value;
      break;
    case Parameter::a2:
      this->parameters_[ka2] = value;
      break;
    default:
      throw std::runtime_error("Unknown parameter name");
  }
  this->compute_transform_and_jacobian();
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
  switch(name)
  {
    case Parameter::d1:
      return this->parameters_[kd1];
      break;
    case Parameter::a1:
      return this->parameters_[ka1];
      break;
    case Parameter::d2:
      return this->parameters_[kd2];
      break;
    case Parameter::a2:
      return this->parameters_[ka2];
      break;
    default:
      throw std::runtime_error("Unknown parameter name");
  }
}

//--------------------------------------------------------------------------------------------------

void DHTransform::free_parameter(Parameter name, bool is_free)
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

int DHTransformVector::optimize_parameters(Eigen::Affine3d const& T_target)
{
  // Pseudo-inverse method
  // ---------------------
  // https://homes.cs.washington.edu/~todorov/courses/cseP590/06_JacobianMethods.pdf
  
  // Optimization parameters
  int constexpr max_iters = 10;
  double constexpr max_tolerance = 1e-5;
  
  // Run algorithm
  int iter_idx = 0;
  for(; iter_idx<max_iters; iter_idx+=1)
  {
    std::cout << "Iteration " << iter_idx << std::endl;
    
    // Get the global transform and check
    Eigen::Affine3d const T = this->get_global_transform();
    Vector6d const dT = log_map(T_target * T.inverse());
    if(dT.norm() < max_tolerance) break;
    
    // Get the pose jacobian wrt optimized parameters
    Matrix6Xd const J = this->get_free_jacobian_matrix();
    std::cout << "- J = \n" << J << std::endl;
    Eigen::MatrixXd const Jp = J.completeOrthogonalDecomposition().pseudoInverse();
    std::cout << "- Jp = \n" << Jp << std::endl;
    Eigen::VectorXd const dp = Jp*dT;
        
    // Update each pose individually
    int col_idx=0;
    int const num_poses = static_cast<int>(this->size());
    for(int pose_idx=0; pose_idx<num_poses; pose_idx+=1)
    {
      Eigen::Vector4d delta_params = Eigen::Vector4d::Zero();
      DHTransform::Parameters const& pose_free_params = (*this)[pose_idx].get_free_parameters();
      for(DHTransform::Parameter const& param : pose_free_params)
      {
        delta_params(static_cast<int>(param)) = dp(col_idx);
        col_idx += 1;
      }
      (*this)[pose_idx].update_parameters(delta_params);
      std::cout << "- dp = " << delta_params.transpose() << std::endl;
    }
  }
  
  return iter_idx;
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
      J_T_a.block(0,col_idx,6,num_params) = J_T_T0im1 * J_T_a.block(0,col_idx,6,num_params);
      col_idx += num_params;
    }

    // Update the pose
    T0im1 = T0im1 * Ti;
  }

  return J_T_a;
}

//--------------------------------------------------------------------------------------------------

} // namespace kinematics
