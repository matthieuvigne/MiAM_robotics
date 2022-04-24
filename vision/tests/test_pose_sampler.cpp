#include <common/maths.hpp>
#include <common/pose_gaussian_sampler.hpp>

int main(int argc, char* argv[])
{
  // Build the pose sampler
  Eigen::Affine3d const T_mean = Eigen::Affine3d::Identity();
  double sigma_orientation = common::convertDegreeToRadian(10.0);
  double sigma_position = 1e-1;
  common::PoseGaussianSampler sampler(T_mean, sigma_orientation, sigma_position);
  sampler.setMaxOrientationDeviation(common::convertDegreeToRadian(7.0));
  sampler.setMaxPositionDeviation(8e-2);

  common::PoseGaussianSampler::Type type = sampler.getType();
  Eigen::Affine3d pose = sampler.sample();

  // Sample a lot of poses
  size_t constexpr num_poses = 1e4;
  Eigen::Matrix<double,6,1> sum = Eigen::Matrix<double,6,1>::Zero();
  Eigen::Matrix<double,6,1> squared_sum = Eigen::Matrix<double,6,1>::Zero();
  Eigen::Matrix<double,6,1> max_value =
    - std::numeric_limits<double>::max() * Eigen::Matrix<double,6,1>::Ones();
  Eigen::Matrix<double,6,1> min_value =
      std::numeric_limits<double>::min() * Eigen::Matrix<double,6,1>::Ones();
  for(size_t i=0u; i<num_poses; ++i)
  {
    Eigen::Matrix<double,6,1> const new_log = common::so3r3::logMap(sampler.sample());
    sum += new_log;
    squared_sum += new_log.cwiseAbs2();
    max_value = max_value.cwiseMax(new_log);
    min_value = min_value.cwiseMin(new_log);
  }
  
  // Compute the mean pose and the associated standard deviations on the Lie's Algebra
  Eigen::Matrix<double,6,1> mean = sum / num_poses;
  Eigen::Matrix<double,6,1> deviation = (squared_sum / num_poses - mean.cwiseAbs2()).cwiseSqrt();
  mean.head<3>() *= 180./M_PI;
  deviation.head<3>() *= 180./M_PI;
  max_value.head<3>() *= 180./M_PI;
  min_value.head<3>() *= 180./M_PI;

  // Display the results
  std::cout << "Mean: " << mean.transpose() << std::endl;
  std::cout << "Deviation: " << deviation.transpose() << std::endl;
  std::cout << "Max values: " << max_value.transpose() << std::endl;
  std::cout << "Min values: " << min_value.transpose() << std::endl;
  
  return EXIT_SUCCESS;
}
