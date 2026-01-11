#ifndef MOTION_PARAMETERS_H
    #define MOTION_PARAMETERS_H

    // Detection arameters
    namespace detection {

      /////////////////////////////////////////////////////
      // Parameters used by computeObstacleAvoidanceSlowdown only
      // Zone radius
      double constexpr r1 = 400;
      double constexpr r2 = 700;

      // Zone angular width
      double constexpr theta1 = M_PI_2;
      double constexpr theta2 = 0.70;

      double const x_max = 430;
      double const y_max = 400;
      double const xfar_max = 700;
      double const yfar_max = 500;

      // during avoidance...
      double const x_max_avoidance = 430;
      double const y_max_avoidance = 400;
      double const xfar_max_avoidance = 700;
      double const yfar_max_avoidance = 500;
      /////////////////////////////////////////////////////

      // Radius used to represent a detected robot in PathPlaner grid.
      // This is also the value used by computeObstacleAvoidanceSlowdownAnticipateTrajectory.
      double const mpc_obstacle_size = 475;
      // add this amount to obstacle radius in PathPlanner to get some slack
      double const mpc_obstacle_margin = 75;
    }

    // Dimensions of the table
    namespace table_dimensions {
      int constexpr table_size_x = 3000;
      int constexpr table_size_y = 2000;

      double constexpr table_margin = 150;  // How much margin to leave: this should be the robot's radius.

      double constexpr table_max_x = (double) table_size_x - table_margin;
      double constexpr table_max_y = (double) table_size_y - table_margin;
      double constexpr table_min_x = table_margin;
      double constexpr table_min_y = table_margin;
    } // namespace table dimensions


    inline bool isPositionInTable(RobotPosition const& pos)
    {
        return (pos.x < table_dimensions::table_max_x && pos.x > table_dimensions::table_min_x
            && pos.y < table_dimensions::table_max_y && pos.y > table_dimensions::table_min_y);
    }

#endif