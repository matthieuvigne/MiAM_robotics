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

// // Drop everything from the suction system
// void Strategy::dropElements()
// {
//     servo->activatePump(false);
//     servo->openValve() ;
//     servo->openTube(0);
//     servo->openTube(1);
//     servo->openTube(2);
//     robot->wait(0.5);
// }


// bool Strategy::shouldPushExcavationSite(ExcavationSquareColor color)
// {
//     bool shouldPush = false;
//     if (robot->isPlayingRightSide())
//     {
//         shouldPush = color ==  ExcavationSquareColor::PURPLE;
//     }
//     else
//     {
//         shouldPush = color ==  ExcavationSquareColor::YELLOW;
//     }
//     return(shouldPush);
// }

// void Strategy::pushExcavationSite()
// {
//     std::cout << "Pushing sample" << std::endl;

//     servo->moveArm(robot->isPlayingRightSide(), arm::MEASURE);
//     servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);
//     robot->wait(0.2);
//     servo->moveFinger(robot->isPlayingRightSide(), finger::PUSH);
//     servo->moveArm(robot->isPlayingRightSide(), arm::RAISE);
//     robot->wait(0.2);
//     servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);
//     robot->updateScore(5);
    
//     if (!is_bonus_already_counted)
//     {
//         std::cout << "Counting first bonus" << std::endl;
//     	is_bonus_already_counted = true;
//     	robot->updateScore(5);
//     }
    	
// }

// // Test an excavation site, pushing it if necessary.
// ExcavationSquareColor Strategy::testExcavationSite()
// {
//     // Take measurement
//     servo->moveArm(robot->isPlayingRightSide(), arm::MEASURE);
//     servo->moveFinger(robot->isPlayingRightSide(), finger::MEASURE);
//     robot->wait(0.4);
//     ExcavationSquareColor const color = robot->getExcavationReadings(robot->isPlayingRightSide());

//     std::cout << "Mesured: " << static_cast<int>(color) << std::endl;

//     if (shouldPushExcavationSite(color))
//     {
//         std::cout << "Detected right color : push!" << std::endl;
//         pushExcavationSite();
        
//     } else {
//         std::cout << "Wrong color" << std::endl;
//     }
//     servo->moveArm(robot->isPlayingRightSide(), arm::RAISE);

//     return color;
// }

// void Strategy::stopEverything()
// {
//     // release magnet and suction
//     servo->activateMagnet(false);
//     servo->activatePump(false);
//     // servo->moveSuction(0, suction::VERTICAL);
//     // servo->moveSuction(1, suction::VERTICAL);
//     // servo->moveSuction(2, suction::VERTICAL);
//     servo->openValve() ;
//     servo->openTube(0);
//     servo->openTube(1);
//     servo->openTube(2);

//     // // raise rail and fold suction
//     // robot->moveRail(0.7);
//     // servo->moveSuction(0, suction::FOLD);
//     // servo->moveSuction(1, suction::FOLD);
//     // servo->moveSuction(2, suction::FOLD);

//     // fold arms
//     servo->moveStatue(statue::FOLD);
//     servo->moveArm(true, arm::FOLD);
//     servo->moveFinger(true, finger::FOLD);
//     servo->moveArm(false, arm::FOLD);
//     servo->moveFinger(false, finger::FOLD);
//     servo->moveClaw(claw::FOLD);
// }
