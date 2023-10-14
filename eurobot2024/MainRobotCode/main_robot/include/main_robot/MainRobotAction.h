#ifndef MAIN_ROBOT_ACTION_H
#define MAIN_ROBOT_ACTION_H


#include "common/AbstractAction.h"


class GetPlantsAction : public Action
{
    public:

        GetPlantsAction(AbstractStrategy* inStrategy) : Action(strategy) {};

        bool performAction() { return true; /* TODO */ };
        int getScore() { return 3; }
};


#endif