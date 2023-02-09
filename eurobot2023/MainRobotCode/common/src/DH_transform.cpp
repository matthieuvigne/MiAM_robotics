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
: DHTransform(parameters(0), parameters(1), parameters(2), parameters(3))
{}

//--------------------------------------------------------------------------------------------------

DHTransform::DHTransform()
: DHTransform(0,0,0,0)
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

void DHTransform::boxplus(double delta_a1, double delta_a2)
{
  this->parameters_[ka1] += delta_a1;
  this->parameters_[ka2] += delta_a2;
  this->compute_transform_and_jacobian();
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,2> const& DHTransform::get_rotation_jacobian_matrix() const
{
  return this->rotation_jacobian_;
}

//--------------------------------------------------------------------------------------------------

void DHTransform::set_parameters(double a1, double a2)
{
  this->parameters_[ka1] = a1; 
  this->parameters_[ka2] = a2; 
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
  int constexpr max_iters = 200;
  double constexpr max_tolerance = 1e-5;
  
  // Run algorithm
  int iter_idx = 0;
  for(; iter_idx<max_iters; iter_idx+=1)
  {
    // Get the global transform and check
    Eigen::Affine3d const T = this->get_global_transform();
    Vector6d const dT = log_map(T_target * T.inverse());
    if(dT.norm() < max_tolerance) break;
    
    // Get the pose jacobian wrt optimized parameters
    Matrix6Xd const J = this->get_jacobian_wrt_free_parameters();
    MatrixX6d const Jp = J.transpose() * (J*J.transpose()).inverse();
    if(Jp.rows()/2 != this->size()) throw std::runtime_error("Inconsistent size.");
    
    // Compute the parameter step
    Eigen::VectorXd const da = Jp*dT;
    for(int i=0; i<da.size(); i+=2)
    {
      double const da1 = da(i);
      double const da2 = da(i+1);
      (*this)[i].boxplus(da1,da2);
    }
  }
  
  return iter_idx;
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
  this->rotation_jacobian_.col(0) = J_T_T1 * J_T1_a1;

  // Jacobian wrt. second set of params
  Eigen::Matrix<double,6,6> const J_T_T2 = get_right_SE3_product_jacobian(T1,T2);
  Eigen::Matrix<double,6,1> J_T2_a2 = Eigen::Matrix<double,6,1>::Zero();
  J_T2_a2.segment<3>(0) = get_left_SO3_jacobian(a2*Eigen::Vector3d::UnitZ()).col(kwz); 
  this->rotation_jacobian_.col(1) = J_T_T2 * J_T2_a2;
}

//--------------------------------------------------------------------------------------------------

DHTransformVector::Matrix6Xd DHTransformVector::get_jacobian_wrt_free_parameters() const
{
  // Principle of the computation of the jacobian of the global pose wrt. angle parameters
  // -------------------------------------------------------------------------------------
  // |  J_T1_p1   ||  J_T2_p2   ||  J_T3_p3   || J_T4_p4   | -> step 1
  // | *J_T12_T1  || *J_T12_T2  ||            ||           | -> step 2
  // | *J_T13_T12 || *J_T13_T12 || *J_T13_T3  ||           | -> step 3
  // | *J_T14_T13 || *J_T14_T13 || *J_T14_T13 || *J_T14_T4 | -> step 4
  
  // Initialize jacobian matrix
  int const num_poses = static_cast<int>(this->size());
  Matrix6Xd J_T_a = Matrix6Xd::Zero(6,2*num_poses);
  for(int idx=0; idx<num_poses; idx+=1)
    J_T_a.block<6,2>(0,2*idx) = (*this)[idx].get_rotation_jacobian_matrix();
    
  // Make forward pass
  Eigen::Affine3d T0im1 = Eigen::Affine3d::Identity();
  for(int idx=0; idx<num_poses; idx+=1)
  {
    // Compute the jacobians
    Eigen::Affine3d const& Ti = (*this)[idx].get_transform();
    Eigen::Matrix<double,6,6> J_T_T0im1 = get_left_SE3_product_jacobian(T0im1,Ti);
    Eigen::Matrix<double,6,6> J_T_Ti = get_right_SE3_product_jacobian(T0im1,Ti);
    
    // Update the jacobians
    for(int jdx=0; jdx<idx; jdx+=1)
      J_T_a.block<6,2>(0,2*jdx) = J_T_T0im1 * J_T_a.block<6,2>(0,2*jdx);
    J_T_a.block<6,2>(0,2*idx) = J_T_Ti * J_T_a.block<6,2>(0,2*idx);
    
    // Update the pose
    T0im1 = T0im1 * Ti;
  }

  return J_T_a;
}

//--------------------------------------------------------------------------------------------------

} // namesapce kinematics
