#include <iostream>

#include <kinematics/DH_transform.hpp>

// Parameters
bool const run_first_problem = false;
bool const run_second_problem = true;

int main(int argc, char* argv[])
{
  using namespace kinematics;

  // Reference arm
  DHTransformVector ref_arm = create_main_robot_arm();
  ref_arm[1].set_parameter(Parameter::a2, M_PI_2);
  Eigen::Affine3d const Tf = ref_arm.get_global_transform();
  
  // First problem type
  if(run_first_problem)
  {
    DHTransformVector arm = create_main_robot_arm();  
    std::cout << "Solving the first problem type" << std::endl;
    OptimizationResult results = arm.optimize_full_pose(Tf);
    std::cout << "Solved the first problem with " << results.num_iters << " iterations." << std::endl;
    std::cout << arm.print() << std::endl;
    std::cout << "The target transform was\n" << Tf.matrix() << std::endl;
  }
  
  if(run_second_problem)
  {
    // Initialize the robotical arm
    DHTransformVector arm = create_main_robot_arm();
    Eigen::Affine3d const Tg = arm.get_global_transform();
    std::cout << "The initial global transform is:\n" << Tg.matrix() << std::endl;
    
    // Get the target transform
    Eigen::Affine3d Tf =
      Eigen::Translation3d(Eigen::Vector3d(0.200, 0.000, -0.180)) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
    std::cout << "The first target transform is:\n" << Tf.matrix() << std::endl;

    // Optimize the parameters to get the desired pose   
    Eigen::Vector3d pf = Tf.translation();
    Eigen::Vector3d uf = Tf.rotation().col(0).normalized();
    std::cout << "Target position is: " << pf.transpose() << std::endl;
    std::cout << "Target x vector is: " << uf.transpose() << std::endl;
    
    // Solve the optimization problem without constraints
    OptimizationResult results = arm.optimize_position_x_direction(pf, uf);
    if(!results.success) std::cout << "Problem failed... " << std::endl;
    std::cout << "Problem 1 was solved with " << results.num_iters << " iterations." << std::endl;
    std::cout << arm.print() << std::endl;
    
    // Get the second target pose
    Tf.translation() = Eigen::Vector3d(0.200, 0.000, -0.140);
    std::cout << "The second target transform is:\n" << Tf.matrix() << std::endl;
    
    // Solve the second optimization problem
    pf = Tf.translation();
    uf = Tf.rotation().col(0).normalized();
    std::cout << "Target position is: " << pf.transpose() << std::endl;
    std::cout << "Target x vector is: " << uf.transpose() << std::endl;
    results = arm.optimize_position_x_direction(pf, uf);
    if(!results.success) std::cout << "Problem failed..." << std::endl;
    std::cout << "Problem 2 was solved with " << results.num_iters << " iterations." << std::endl;
    std::cout << arm.print() << std::endl;
  }
  
  return EXIT_SUCCESS;
}
