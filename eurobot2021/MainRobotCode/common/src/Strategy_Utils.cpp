/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <unistd.h>
#include <math.h>
#include <thread>

#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>
#include "Parameters.h"
#include "Strategy.h"

// Drop everything from the suction system
void Strategy::dropElements()
{
    servo->activatePump(false);
    servo->openValve() ;
    servo->openTube(0);
    servo->openTube(1);
    servo->openTube(2);
    robot->wait(0.5);
}


bool Strategy::shouldPushExcavationSite(ExcavationSquareColor color)
{
    bool shouldPush = false;
    if (robot->isPlayingRightSide())
    {
        shouldPush = color ==  ExcavationSquareColor::PURPLE;
    }
    else
    {
        shouldPush = color ==  ExcavationSquareColor::YELLOW;
    }
    return(shouldPush);
}

void Strategy::pushExcavationSite()
{
    servo->moveFinger(robot->isPlayingRightSide(), finger::PUSH);
    servo->moveArm(robot->isPlayingRightSide(), arm::RAISE);
    robot->wait(0.4);
    servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);
}

// Test an excavation site, pushing it if necessary.
ExcavationSquareColor Strategy::testExcavationSite()
{
    // Take measurement
    servo->moveArm(robot->isPlayingRightSide(), arm::MEASURE);
    servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);
    robot->wait(0.8);
    ExcavationSquareColor const color = robot->getExcavationReadings(robot->isPlayingRightSide());

    if (shouldPushExcavationSite(color))
    {
        pushExcavationSite();
        robot->updateScore(5);
    }
    servo->moveArm(robot->isPlayingRightSide(), arm::RAISE);

    return color;
}
