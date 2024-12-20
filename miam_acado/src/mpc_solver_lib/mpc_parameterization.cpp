#include <mpc_parameterization.hpp>

TrajectoryConfig getMPCTrajectoryConfig() {
    TrajectoryConfig c;
    c.maxWheelVelocity = 500;
    c.maxWheelAcceleration = 600;
    c.robotWheelSpacing = 100.5;
    return c;
};

TrajectoryVector computeTrajectoryBasicPath(
    TrajectoryConfig const& config,
    std::vector<RobotPosition> p,
    double initialSpeed)
{
    TrajectoryVector vector;

    for (int i = 0; i < p.size() - 1; i++) {
        RobotPosition p1 = p.at(i);
        RobotPosition p2 = p.at(i+1);

        double starting_speed;
        if (i == 0) {
            starting_speed = initialSpeed;
        } else {
            // starting speed is the ending speed of the preceding trajectory
            // do not take the endpoint or else the speed is null
            starting_speed = vector.back()->getCurrentPoint(vector.back()->getDuration() - 0.001).linearVelocity;
        }

        std::shared_ptr<StraightLine> line(new StraightLine(
            config, p1, p2, 
            starting_speed, 
            config.maxWheelVelocity, false));

        // Go from point to point.
        vector.push_back(std::shared_ptr<Trajectory>(line));
    }

    return vector;
};

double getDurationBasicPath(TrajectoryVector bp) {
    double sum_time = 0;
    for (int i = 0; i < bp.size(); i++) {
        sum_time += bp.at(i)->getDuration();
    }
    return sum_time;
}

TrajectoryPoint getCurrentPointBasicPath(TrajectoryVector bp, double current_time) {
    double sum_time = 0;
    for (int i = 0; i < bp.size(); i++) {
        double traj_time = bp.at(i)->getDuration();
        if (sum_time + traj_time > current_time) {
            return bp.at(i)->getCurrentPoint(current_time - sum_time);
        }
        sum_time += traj_time;
    }
    return bp.back()->getEndPoint();
};