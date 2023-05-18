#ifndef ABSTRACT_ACTION_H
#define ABSTRACT_ACTION_H

#include "RobotInterface.h"
#include <iostream>
class Action {

public:
    bool isFeasible()
    {
        return true;
    }

    double nPointsToGain_;
    double timeItTakes_;
    RobotPosition startPosition_;
    bool isActivated_;

    void performAction(RobotInterface* robot) {
        robot->updateScore(nPointsToGain_);
        robot->wait(timeItTakes_);
    }

    Action(double nPointsToGain, double timeItTakes, RobotPosition startPosition)
    {
        nPointsToGain_ = nPointsToGain;
        timeItTakes_ = timeItTakes;
        startPosition_ = startPosition;
        isActivated_ = true;
    }

    bool operator==(Action const& other)
    {
        bool ok = (nPointsToGain_ == other.nPointsToGain_);
        ok &= (timeItTakes_ == other.timeItTakes_);
        // ok &= (startPosition_ == other.startPosition_);
        return ok;
    }

    friend std::ostream& operator<<(std::ostream& os, const Action& dt);
};

inline std::ostream& operator<<(std::ostream& os, const Action& dt)
{
    os << dt.startPosition_;
    return os;
}

#endif