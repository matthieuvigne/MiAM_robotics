/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <unistd.h>
#include <math.h>

#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>
#include "Parameters.h"
#include "Strategy.h"

//reste à faire: symetrie booléens pour servos & doigts avec is Right dans Robot.h
//dans initialisation des 3 minutes : initialiser rail au mileu, servos figurine en haut, statuette bas
//definir hauteurs de rails (depose figurine)
//ajuster l'ordre : recupérer statue en position milieu puis depose figurine
//ajuster hauteur servos (milieu legerement haut) pour pousser avec doigt (position bas) à l'endroit du distributeur, mesures (servos milieu legerement bas)


using namespace miam::trajectory;
using miam::RobotPosition;

void setupRobot(RobotInterface *robot, ServoHandler *servo)
{
    servo->figurineArmLow();
    servo->turnOffPump();
}


void matchStrategy(RobotInterface *robot, ServoHandler *servo)
{
    std::cout << "Strategy thread started." << std::endl;
    while(true) {


	    miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
		                                            robotdimensions::maxWheelAccelerationTrajectory,
		                                            robotdimensions::wheelSpacing);

	    // Create required variables.
	    RobotPosition targetPosition;
	    TrajectoryVector traj;
	    RobotPosition endPosition;
	    std::vector<RobotPosition> positions;
	    bool wasMoveSuccessful = true;
	    robot->updateScore(2);
	    servo->initsectionmiddle();


	    servo->openTube(2);
        servo->turnOnPump();
        robot->wait(5000000);
        servo->closeTube(2);
        servo->closeTube(1);
        servo->closeTube(3);
        robot->wait(10000000);


	    //initialisation servos
	    //servo->moveSuction(true);
	    //servo->moveSuctionUnitary(2,1800);
	    //servo->reglageouvrirledoigtdroitbas();
	    //robot->wait(10000000);
	    //servo->moveSuctionUnitary(1,1800);
	    //robot->wait(1000000);
	    //servo->moveSuctionUnitary(2,1800);
	    //robot->wait(1000000);
	    //servo->ouvrirlebrasmilieu(robot->isPlayingRightSide(),false);
	    //robot->wait(1000000);


	    //servo->reglageouvrirlebrasdroitbas();
	    //robot->wait(10000000);
	    //servo->electroMagnetOn();
	    //servo->reglageouvrirlebrasgauchebas();
	    //robot->wait(5000000);

	    //if (robot->getExcavationReadings(true)== YELLOW)
	    	//servo->reglageouvrirlebrasgauchemilieu();
	        //robot->wait(5000000);
	        //servo->reglageouvrirlebrasgauchehaut();
	    	//servo->reglageouvrirledoigtdroithaut();



	     /*
	    // Set initial position
	    targetPosition.x = robotdimensions::CHASSIS_BACK;
	    targetPosition.y = 1200;
	    targetPosition.theta = 0;
	    robot->resetPosition(targetPosition, true, true, true);

	    servo->openTube(2);
	    //servo->turnOnPump();
	    servo->closeTube(2);
	    servo->closeTube(1);
	    servo->closeTube(3);
	    robot->wait(1000000);
	    // Move rail to top.
	    //robot->moveRail(1.0);
	    //robot->wait(100000); // Wait 100ms - works both on the robot and in simulation.
            */

	    /*
	    //**********************************************************
	    // Go get the statue
	    //**********************************************************
	    targetPosition = robot->getCurrentPosition();
	    positions.push_back(targetPosition);
	    targetPosition.x = 450;
	    positions.push_back(targetPosition);
	    targetPosition.y = 450;
	    positions.push_back(targetPosition);
	    // Move at 45degree angle toward the statue
	    targetPosition.x += 80;
	    targetPosition.y += 80;
	    positions.push_back(targetPosition);
	    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.2);
	    robot->setTrajectoryToFollow(traj);
	    wasMoveSuccessful = robot->waitForTrajectoryFinished();
	    targetPosition = robot->getCurrentPosition();

	    //retrive statue and put the figurine

	    servo->electroMagnetOn();
	    servo->figurineArmLow();
             */


    }


    //while(true) {
        //std::cout << "waiting" << std::endl;
        //robot->wait(100000);
    //}

    // Update config.
    miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                    robotdimensions::maxWheelAccelerationTrajectory,
                                                    robotdimensions::wheelSpacing);

    // Create required variables.
    RobotPosition targetPosition;
    TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;
    bool wasMoveSuccessful = true;
    robot->updateScore(2);

    servo->moveSuction(true);
    robot->wait(100000);

    // Set initial position
    targetPosition.x = robotdimensions::CHASSIS_BACK;
    targetPosition.y = 1200;
    targetPosition.theta = 0;
    robot->resetPosition(targetPosition, true, true, true);

    servo->openTube(2);
    servo->turnOnPump();
    servo->closeTube(2);
    servo->closeTube(1);
    servo->closeTube(3);
    robot->wait(100000);
    // Move rail to top.
    //robot->moveRail(1.0);
    //robot->wait(100000); // Wait 100ms - works both on the robot and in simulation.

    //**********************************************************
    // Go get the statue
    //**********************************************************
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 450;
    positions.push_back(targetPosition);
    targetPosition.y = 450;
    positions.push_back(targetPosition);
    // Move at 45degree angle toward the statue
    targetPosition.x += 80;
    targetPosition.y += 80;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.2);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    targetPosition = robot->getCurrentPosition();

    //retrive statue and put the figurine

    servo->electroMagnetOn();
    servo->figurineArmLow();

    //Go back
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition,-131.0);
    robot->setTrajectoryToFollow(traj);
    robot->updateScore(15);
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    //Rotate
    targetPosition.x = 500;
    targetPosition.y = 500;
    positions.push_back(targetPosition);
    targetPosition.x = 450;
    targetPosition.y = 450;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.2);
    robot->setTrajectoryToFollow(traj);
    servo->moveSuction(true);
    servo->turnOffPump();
    servo->openTube(2);
    servo->openTube(1);
    servo->openTube(3);
    robot->updateScore(15);
    robot->updateScore(15);

    //**********************************************************
    // Go back to the side distributor
    //**********************************************************
    targetPosition = robot->getCurrentPosition();


    std::cout << targetPosition << std::endl;

    // Go back
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition,-250.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    //**********************************************************
    // Round to the side distributor
    //**********************************************************

    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    //Rotate
    targetPosition.x = 102+40+robotdimensions::CHASSIS_WIDTH+50;
    targetPosition.y = 500;
    positions.push_back(targetPosition);
    targetPosition.y = 750;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 150.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    robot->updateScore(3);
    servo->ouvrirlebrasmilieu(robot->isPlayingRightSide(),false);
    servo->ouvrirledoigtbas(robot->isPlayingRightSide(),false);

    //**********************************************************
    // Go to the display
    //**********************************************************


    // go forward
    targetPosition = robot->getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition, 500.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    servo->ouvrirlebrashaut(robot->isPlayingRightSide(),false);

    // then a little to the left
    targetPosition = robot->getCurrentPosition();
    endPosition = targetPosition;
    endPosition.x = 100 + robotdimensions::CHASSIS_WIDTH;
    endPosition.y += 100;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(targetPosition,endPosition,0.0,false);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();


    // finally go straight
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    endPosition.x = targetPosition.x ;
    endPosition.y = 2000-40-robotdimensions::CHASSIS_FRONT - 40;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(targetPosition,endPosition,0.0,false);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    servo->electroMagnetOff();
    robot->updateScore(15);

    // go back a little
    targetPosition = robot->getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition,-100.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    //**********************************************************
    // Go to the side distributor
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1700 ;
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_FRONT + 40 + 20 ;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    servo->turnOnPump();
    servo->moveSuction(false);
    robot->updateScore(1);

    //**********************************************************
    // Go back to the gallery & side distributor
    //**********************************************************

    servo->ouvrirlebrashaut(robot->isPlayingRightSide(),true);
    //Go back
    targetPosition = robot->getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition,-1100.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    robot->updateScore(3);
    servo->ouvrirlebrashaut(robot->isPlayingRightSide(),true);

    //**********************************************************
    // Rotate to the gallery & stop to put the first tresor
    //**********************************************************

    double y_front_of_the_gallery = 2000- robotdimensions::CHASSIS_FRONT - 100 - 60;

    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 930;
    targetPosition.y -= 150;
    positions.push_back(targetPosition);
    targetPosition.y = y_front_of_the_gallery ;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    servo->turnOffPump();
    servo->moveSuction(true);
    robot->updateScore(8);

    // go back a little
    targetPosition = robot->getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition,-100.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();



    //**********************************************************
    // Go to the side distributor
    //**********************************************************
    //positions.clear();
    //targetPosition = robot->getCurrentPosition();
    //positions.push_back(targetPosition);
    //targetPosition.y = 2000- robotdimensions::CHASSIS_FRONT - 100 - 40 ;
    //positions.push_back(targetPosition);
    //targetPosition.x = 1350;
    //positions.push_back(targetPosition);
    //traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    //robot->setTrajectoryToFollow(traj);
    //wasMoveSuccessful = robot->waitForTrajectoryFinished();
    //robot->updateScore(15);

     //**********************************************************
    // Go to the central zone
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 500;
    positions.push_back(targetPosition);
    targetPosition.y = 1325 ;
    positions.push_back(targetPosition);
     targetPosition.x = 675;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    servo->turnOnPump();
    servo->moveSuction(false);
    robot->updateScore(15);

     //**********************************************************
    // Rotate to the edge
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = y_front_of_the_gallery ;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    robot->updateScore(9);
    servo->turnOffPump();
    servo->moveSuction(true);

    // go back a little
    targetPosition = robot->getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition,-100.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    //**********************************************************
    // Rotate to the zone de fouille
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 1200;
    targetPosition.y = 800 ;
    positions.push_back(targetPosition);
    targetPosition.x = 800;
    targetPosition.y = 450 ;
    positions.push_back(targetPosition);
    targetPosition.x = 450 ;
    targetPosition.y = 220 ;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    robot->updateScore(15);


    // go back a big little
    targetPosition = robot->getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition,-150.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    //**********************************************************
    // Rotate to measure (with several stops to add with finger to command),
    //**********************************************************


    // servo->baisserledoigtdroit();

    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 450;
    targetPosition.y = robotdimensions::CHASSIS_WIDTH + 40 +20;
    positions.push_back(targetPosition);
    targetPosition.x = 667.5;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    //test des 4 configurations

    //1ere mesure du carré : soit notre équipe (Yellow ou Violet) soit RED
    if (robot->getExcavationReadings(true)== RED)  //dans configuration
    {
        servo->reglageouvrirlebrasgauchemilieu();
        robot->wait(5000000);
        servo->reglageouvrirlebrasgauchehaut();
        //servo->reglageouvrirledoigtdroithaut();
    }

    servo->ouvrirlebrasmilieu(robot->isPlayingRightSide(),true);
    servo->ouvrirledoigtbas(robot->isPlayingRightSide(),true);
    robot->updateScore(8);

    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = robotdimensions::CHASSIS_WIDTH + 40 +20;
    targetPosition.x = 852.5;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    servo->ouvrirlebrasmilieu(robot->isPlayingRightSide(),true);
    robot->updateScore(8);

    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = robotdimensions::CHASSIS_WIDTH + 40 +20;
    targetPosition.x = 1037.5;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    servo->ouvrirlebrasmilieu(robot->isPlayingRightSide(),true);
    robot->updateScore(8);

    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = robotdimensions::CHASSIS_WIDTH + 40 +20;
    targetPosition.x = 1222.5;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    servo->ouvrirlebrasmilieu(robot->isPlayingRightSide(),true);
    robot->updateScore(8);

    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = robotdimensions::CHASSIS_WIDTH + 40 +20;
    targetPosition.x = 1407.5;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    servo->ouvrirlebrasmilieu(robot->isPlayingRightSide(),true);
    robot->updateScore(8);
    servo->ouvrirlebrashaut(robot->isPlayingRightSide(),true);

    //**********************************************************
    // Rotate to come back to the campment
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 900;
    targetPosition.y = 600;
    positions.push_back(targetPosition);
    targetPosition.x = 975;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    robot->updateScore(20);

    std::cout << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}
