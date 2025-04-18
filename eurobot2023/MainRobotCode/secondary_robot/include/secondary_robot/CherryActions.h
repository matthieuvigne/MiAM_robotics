#ifndef CHERRY_ACTION_H
#define CHERRY_ACTION_H

#include "secondary_robot/SecondaryRobotAction.h"

using namespace miam;

namespace CherryActionConstants
{
    // Points of interest: points to go to grab the cherries (before the final move action)
    double const distributor_width = 30;
    double const cherryDistributorOffset = 151  + 60 + distributor_width / 2.0;
    RobotPosition const cherryDistributorBottom(1000 - cherryDistributorOffset, 165, 0);
    RobotPosition const cherryDistributorTop(1000 - cherryDistributorOffset, 3000 - 165, 0);
    RobotPosition const cherryDistributorLeft(cherryDistributorOffset, 1500, 0);
    RobotPosition const cherryDistributorRight(2000 - cherryDistributorOffset, 1500, 0);
}


class CherryAction : public SecondaryRobotAction
{
    public:

        CherryAction(AbstractStrategy* inStrategy) : SecondaryRobotAction(inStrategy) {};

        bool performAction();
        int getScore() { return 10; }
};


class GrabCherriesLeft : public CherryAction
{
    public:
        GrabCherriesLeft(AbstractStrategy* inStrategy) : CherryAction(inStrategy)
        {
            start_position = CherryActionConstants::cherryDistributorLeft;
            start_position.theta = M_PI;
            start_position.x += 20;

            // no end position
        }
};

class GrabCherriesRight : public CherryAction
{
    public:
        GrabCherriesRight(AbstractStrategy* inStrategy) : CherryAction(inStrategy)
        {
            start_position = CherryActionConstants::cherryDistributorRight;
            start_position.theta = 0;
            start_position.x -= 20;

            // no end position
        }
};

class PutCherriesInTheBasket : public SecondaryRobotAction
{
    public:

        PutCherriesInTheBasket(AbstractStrategy* inStrategy) : SecondaryRobotAction(inStrategy) {
            start_position.x = 180; //robotParameters.CHASSIS_WIDTH + 90.0;
            start_position.y = 3000 - 150 - 160;
            start_position.theta = M_PI_2;

            RobotPosition obstacle;
            obstacle.x = 733;
            obstacle.y = 1882;
            obstacles_on_the_road.push_back(std::make_tuple(obstacle, 200));
        };

        bool performAction();
        int getScore() { return 10; }

        bool needRailTop() { return true; }
};


#endif