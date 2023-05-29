#ifndef SECONDARY_ROBOT_ACTION_H
#define SECONDARY_ROBOT_ACTION_H

#include "miam_utils/trajectory/RobotPosition.h"
#include "miam_utils/trajectory/Trajectory.h"
#include "common/RobotInterface.h"
#include "common/AbstractStrategy.h"


class SecondaryRobotAction
{
    public:

        SecondaryRobotAction(AbstractStrategy* inStrategy) : activated(true), strategy(inStrategy) {};

        RobotPosition start_position;
        RobotPosition end_position;

        std::vector<Obstacle > obstacles_on_the_road;
        std::vector<Obstacle > obstacles_in_the_end;

        bool activated;

        AbstractStrategy* strategy;

        virtual bool performAction() = 0;
        virtual int getScore() = 0;
        bool needRailTop() { return false; }
};


#endif