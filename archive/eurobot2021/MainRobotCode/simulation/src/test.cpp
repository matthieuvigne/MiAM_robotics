class AbstractAction {

    double nPointsToGain;
    double timeItTakes;
    RobotPosition actionStart;


    void performAction() { robot.score += nPointsToGain; robot.wait(timeItTakes);}
}

struct RobotState{
    // Est-ce que j'ai des choses dans mes ventouses...
    bool ventouseX;
}
class Map {
    // Carte du terrain
    bool canIGoThere(Point target);
}
class MotionPlanning{


    std::vector<points> planif_trajectory_highLevel(RobotPosition start, RobotPosition end, Map m)
    {
        // Compute traj
    }

    miam::Trajectory smoothAndSpeedTrajectory(std::vector<points>)
    {
        return smoothTraj;
    }

    miam::Trajectory computeTraj(start, end)
    {
        return smoothAndSpeedTrajectory(planif_trajectory_highLevel(start, end, m ))
    }

    void updateMap()
    {

    }
}

void strategy()
{
    std::vector<AbstractActions> actions;
    RobotState state;
    MotionPlanning planner;
    while (!matchHasEnded)
    {
        AbstractAction a = chooseNextAction(actions, state, robot);

        miam::Trajectory traj = planner.computeTraj(robot.getCurrentPosition(), a.actionStart)

        robot.setTrajectoryToFollow(traj);
        bool success = robot.waitForTrajectoryFinish();
        if (success)
        {
            a.performAction(state);
        }
    }
}