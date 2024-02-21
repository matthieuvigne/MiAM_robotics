/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#include <cstdlib>
#include <iostream>
#include <miam_utils/Logger.h>



#include "common/MotionController.h"
#include "main_robot/Parameters.h"


int main (int argc, char *argv[])
{
    Logger logger;
    logger.start("motionControllerTest.hdf5");

    MotionController motionController(
        main_robot::generateParams(),
        &logger);

    miam::RobotPosition startPosition(310.0, 1690.0, 0.0);
    miam::RobotPosition targetPosition(600.0, 1300.0, -0.5);

    motionController.resetPosition(startPosition);
    motionController.computeMPCTrajectory(
        targetPosition,
        motionController.getDetectedObstacles(),
        true);

    return 0;
}


