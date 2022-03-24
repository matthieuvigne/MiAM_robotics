/// \author Rodolphe Dubois
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "Robot.h"

/// Function OK for robot playing right
bool Robot::isLidarPointWithinTable(LidarPoint const& point)
{
  // 1 Get the robot current position
  miam::RobotPosition const robot_position = this->getCurrentPosition();
  double const T_x_R = robot_position.x;
  double const T_y_R = robot_position.y;
  double const theta_T_R = robot_position.theta;

  // 2. Project the Lidar point within the table
  double const T_x_fi = T_x_R + point.r * std::cos(theta_T_R + point.theta);
  double const T_y_fi = T_y_R + point.r * std::sin(theta_T_R + point.theta);


  // 3. Check if the lidar point falls within the table
  if(T_x_fi < table_dimensions::table_max_x and T_x_fi > table_dimensions::table_min_x
    and T_y_fi < table_dimensions::table_max_y and T_y_fi > table_dimensions::table_min_y )
  {
      // Remove ramp.
      if(T_y_fi < table_dimensions::ramp_max_y
        and T_x_fi > table_dimensions::ramp_min_x
        and T_x_fi < table_dimensions::ramp_max_x)
            return false;
    //~ std::cout <<  T_x_fi << " " << T_y_fi << "r" << point.r << " theta" << point.theta << "np" << lastNumberOfPoints << std::endl;
    return true;
  }

  return false;
}

double Robot::avoidOtherRobots()
{
  // Handle robot stops
  static int num_stop_iters = 0.;
  constexpr int min_stop_iters = 12; // Minimum number of iterations to stop, i.e 10ms.
  constexpr int min_restart_iter = 20; // Minimum number of iterations to restart, i.e 10ms.

  double coeff = 1.0;
  bool is_robot_stopped = false;

  LidarPoint detected_point;
  detected_point.r = 1e6;
  detected_point.theta = 0.;

  // Update trajectory.
  bool forward = true;
  if (!currentTrajectories_.empty())
  {
    miam::trajectory::TrajectoryPoint trajectoryPoint = currentTrajectories_.at(0)->getCurrentPoint(curvilinearAbscissa_);
    forward = (trajectoryPoint.linearVelocity >= 0);
  }

  for(const DetectedRobot& robot : lidar_.detectedRobots_)
  {
    // Get the Lidar Point, symeterize it if needed and check its projection
    LidarPoint const point = this->isPlayingRightSide_
      ? LidarPoint(robot.point.r, -robot.point.theta)
      : LidarPoint(robot.point.r, robot.point.theta);

    if(!this->isLidarPointWithinTable(point)) continue;

    double x = point.r * std::cos(point.theta + (forward ? 0: M_PI));
    double y = point.r * std::sin(point.theta + (forward ? 0: M_PI));

    if (x > 0)
    {
        if (x < detection::x_max)
        {
            if (std::abs(y) < detection::y_max)
            {
                // Stop robot.
                coeff = 0.0;
                motorSpeed_[0] = 0.0;
                motorSpeed_[1] = 0.0;
                is_robot_stopped = true;
                detected_point = point;
            }
        }
        else if (x < detection::xfar_max)
        {
            double maximumY = detection::y_max + (x - detection::x_max) / (detection::xfar_max - detection::x_max) * (detection::yfar_max - detection::y_max);
            if (std::abs(y) < maximumY)
            {
                const double current_coeff = std::min(1.0, std::max(0.2, (point.r - detection::r1)
                    / (detection::r2 - detection::r1)));
                if(current_coeff < coeff)
                {
                    coeff = current_coeff;
                    detected_point = point;
                }
            }
        }
    }
  }

  // Before match: just return coeff, don't trigger memory.
  if (!hasMatchStarted_ || currentTrajectories_.size() == 0)
  {
    num_stop_iters = 0;
    return coeff;
  }

    if (num_stop_iters >= min_stop_iters &&  num_stop_iters < min_restart_iter)
    {
        num_stop_iters++;
        // Not ready to restart, just stop
        return 0.0;
    }

  if (is_robot_stopped)
  {
    num_stop_iters++;
    if (num_stop_iters < min_stop_iters)
    {
        // Not enough time spend, just slow down.
        coeff_ = 0.2;
    }
    else if(num_stop_iters > avoidanceTimeout_)
    {
      // Proceed avoidance
      // Search for the closest point along trajectory out of red zone
      // and far enough from the other robots and ask the MPC to set a new
      // trajectory to this point avoiding the fixed robot.
      // Set the new trajectory
      //~ std::cout << num_stop_iters << std::endl;
      num_stop_iters = 0;

      // Failed to perform avoidance.
      // Raise flag and end trajectory following.
      wasTrajectoryFollowingSuccessful_ = false;
      currentTrajectories_.clear();
      std::cout << "Obstacle still present, canceling trajectory" << std::endl;
    }
  }
  else
  {
      if (num_stop_iters >= min_restart_iter)
      {
          // Robot was stopped and is ready to start again.
          // Replan and retry trajectory.
          if (!currentTrajectories_.empty())
          {
              currentTrajectories_.at(0)->replanify(curvilinearAbscissa_);
              curvilinearAbscissa_ = 0;
          }
      }
      num_stop_iters = 0;
  }

  return coeff;
}
