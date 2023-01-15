#include <miam_utils/trajectory/Trajectory.h>
#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>

using namespace miam;

int main() {
    RobotPosition targetPosition;
    trajectory::TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;

    miam::trajectory::TrajectoryConfig c;
    c.maxWheelVelocity = 500;
    c.maxWheelAcceleration = 600;
    c.robotWheelSpacing = 100.5;

    targetPosition = RobotPosition(0, 0, 0);
    endPosition = RobotPosition(1000, 0, 0);

    traj = trajectory::computeTrajectoryStraightLineToPoint(c, targetPosition, endPosition);

    std::cout << "EndPoint : " << traj.getEndPoint().position << std::endl;
}