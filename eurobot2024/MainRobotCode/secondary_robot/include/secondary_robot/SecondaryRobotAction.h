#ifndef SECONDARY_ROBOT_ACTION_H
#define SECONDARY_ROBOT_ACTION_H


#include "common/AbstractAction.h"


class SecondaryRobotAction : public Action
{
    public:

        SecondaryRobotAction(AbstractStrategy* inStrategy) : Action(strategy) {};

        bool needRailTop() { return false; }
};


#endif