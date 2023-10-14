#ifndef ABSTRACT_ACTION_H
#define ABSTRACT_ACTION_H

#include "common/RobotInterface.h"
#include "miam_utils/trajectory/RobotPosition.h"
#include "miam_utils/trajectory/Trajectory.h"
#include "common/AbstractStrategy.h"
#include <iostream>
class Action {

public:

    Action(AbstractStrategy* inStrategy) : activated(true), strategy(inStrategy) {};

    RobotPosition start_position;
    RobotPosition end_position;

    std::vector<Obstacle > obstacles_on_the_road;
    std::vector<Obstacle > obstacles_in_the_end;

    bool activated;

    AbstractStrategy* strategy;

    virtual bool performAction() = 0;
    virtual int getScore() = 0;

    // bool isFeasible()
    // {
    //     return true;
    // }

    // double nPointsToGain_;
    // double timeItTakes_;
    // RobotPosition startPosition_;
    // bool isActivated_;

    // void performAction(RobotInterface* robot) {
    //     robot->updateScore(nPointsToGain_);
    //     robot->wait(timeItTakes_);
    // }

    // Action(double nPointsToGain, double timeItTakes, RobotPosition startPosition)
    // {
    //     nPointsToGain_ = nPointsToGain;
    //     timeItTakes_ = timeItTakes;
    //     startPosition_ = startPosition;
    //     isActivated_ = true;
    // }

    // bool operator==(Action const& other)
    // {
    //     bool ok = (nPointsToGain_ == other.nPointsToGain_);
    //     ok &= (timeItTakes_ == other.timeItTakes_);
    //     // ok &= (startPosition_ == other.startPosition_);
    //     return ok;
    // }

    friend std::ostream& operator<<(std::ostream& os, const Action& dt);
};

inline std::ostream& operator<<(std::ostream& os, const Action& dt)
{
    os << dt.start_position;
    return os;
}

#endif