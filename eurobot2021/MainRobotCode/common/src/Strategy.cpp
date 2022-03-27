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

    // Set initial position
    targetPosition.x = robotdimensions::CHASSIS_BACK;
    targetPosition.y = 1200;
    targetPosition.theta = 0;
    robot->resetPosition(targetPosition, true, true, true);
    
    //init ventouse & rail
    servo->moveSuction(true);
    servo->initsectionmiddle();
    robot->wait(1e6);
    robot->moveRail(0.7);
    robot->wait(5e6);
    robot->moveRail(0.97);

	    
   //init pompe
    servo->turnOffPump();
    servo->openValve();
    servo->openTube(0);
    servo->openTube(1);
    servo->openTube(2);
    robot->wait(1e6);
    

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

    // begin to move arm some time after beginning to follow traj
    robot->wait(3e6);
    servo->figurineArmTransport();

    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    std::cout << "ETAPE 1 : juste avant de reculer vers la statuette" << std::endl;

    targetPosition = robot->getCurrentPosition();


    //Go back
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition,-261.0);
    robot->setTrajectoryToFollow(traj);
    robot->updateScore(15);


    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    std::cout << "ETAPE 2 : devant la statuette" << std::endl;


    std::cout << "ETAPE 3 : baisser les bras prendre la statuette et lever le bras" << std::endl;

    servo->figurineArmCatch();
    robot->wait(5e5);
    servo->electroMagnetOn();
    robot->wait(1e6);
    servo->figurineArmTransport();


    targetPosition = robot->getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition,140.0);
    robot->setTrajectoryToFollow(traj);

    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    std::cout << "ETAPE 4 : le robot est cense avoir avance avec la statuette" << std::endl;


    //**********************************************************
    // Go drop the figurine
    //**********************************************************    

    // rotate 180 deg
    targetPosition = robot->getCurrentPosition();
    traj.clear();
    traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(targetPosition, targetPosition.theta + M_PI)));
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    // go forward and drop figurine
    targetPosition = robot->getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition,30.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    // TODO : placer figurine sur le piedestal ici
    //lancer pompe
    //servo->openValve();
    servo->closeTube(0);
    servo->closeTube(2);
    robot->wait(1e6);
    //servo->openTube(2);
    servo->turnOnPump();
    //robot->wait(5000000);
    servo->closeValve();
    //robot->wait(2000000);
    //servo->closeTube(1);
    //robot->wait(3000000);
    //servo->turnOffPump();
    //aspirer pour toutes les ventouses
    robot->wait(3e6);
    robot->moveRail(0.15);
    servo->transportfigurine();
    //lacher milieu
    robot->wait(1e6);
    servo->turnOffPump();
    servo->openValve() ;
    servo->openTube(0);
    servo->openTube(1);
    servo->openTube(2);
    robot->wait(5e6);

    // reculer
    targetPosition = robot->getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition,-30.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();


    //**********************************************************
    // Round to the side distributor
    //**********************************************************

    // go in front of the side distributor
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
    robot->moveRail(0.5);

    // TODO : unfold finger

    // We should have knocked down the hexagons
    robot->updateScore(3);


    //**********************************************************
    // Go to the display
    //**********************************************************

    // continue trajectory
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y += 500;
    positions.push_back(targetPosition);
    targetPosition.x = 100 + robotdimensions::CHASSIS_WIDTH;
    targetPosition.y += 100; 
    targetPosition.theta += M_PI;
    positions.push_back(targetPosition);
    traj = miam::trajectory::computeTrajectoryRoundedCorner(positions, 150.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();



    std::cout << "ETAPE 5 : le robot fait marche arriere" << std::endl;


    // finally go straight
    targetPosition = robot->getCurrentPosition();
    endPosition.x = targetPosition.x ;
    endPosition.y = 2000-20-robotdimensions::CHASSIS_FRONT;
    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(targetPosition,endPosition,0.0,true);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    std::cout << "ETAPE 6 : lacher la statuette" << std::endl;
    
    servo->electroMagnetOff();
    robot->wait(5e5);
    robot->updateScore(15);

 

    // go forward a little
    targetPosition = robot->getCurrentPosition();
    traj = miam::trajectory::computeTrajectoryStraightLine(targetPosition,90.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    // while(true) {
    //     robot->wait(1e6);
    // }


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
    //servo->moveSuction(false);
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
    //servo->moveSuction(true);
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
    //servo->moveSuction(false);
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
    //servo->moveSuction(true);

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

