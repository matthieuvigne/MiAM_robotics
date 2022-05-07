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
    servo->moveStatue(statue::FOLD);
    servo->activateMagnet(false);
    servo->activatePump(false);

    servo->moveArm(true, arm::FOLD);
    servo->moveFinger(true, finger::FOLD);
    servo->moveArm(false, arm::FOLD);
    servo->moveFinger(false, finger::FOLD);

    //init ventouse & rail
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::FOLD);
    servo->moveSuction(1, suction::HOLD_FAKE_STATUE);
    robot->moveRail(0.7);
    robot->wait(5.0);
    robot->moveRail(0.97);
}

// Test an excavation site, pushing it if necessary.
void testExcavationSite(RobotInterface *robot, ServoHandler *servo)
{
    // Take measurement
    servo->moveArm(!robot->isPlayingRightSide(), arm::MEASURE);
    servo->moveFinger(!robot->isPlayingRightSide(), finger::MEASURE);
    robot->wait(0.2);
    ExcavationSquareColor const color = robot->getExcavationReadings(!robot->isPlayingRightSide());
    bool shouldDrop = false;
    const lastMeasuredColor = robot->getExcavationReadings(!robot->isPlayingRightSide());

    if (robot->isPlayingRightSide())
    {
        shouldDrop = color ==  ExcavationSquareColor::PURPLE;
    }
    else
    {
        shouldDrop = color ==  ExcavationSquareColor::YELLOW;
    }
    if (shouldDrop)
    {
        servo->moveFinger(!robot->isPlayingRightSide(), finger::PUSH);
        robot->updateScore(5);
        robot->wait(0.2);
        servo->moveFinger(!robot->isPlayingRightSide(), finger::MEASURE);
    }
    servo->moveArm(!robot->isPlayingRightSide(), arm::RAISE);
    
    return lastMeasuredColor;
}


void matchStrategy(RobotInterface *robot, ServoHandler *servo)
{
    std::cout << "Strategy thread started." << std::endl;

    // Update config.
    setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
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

   //init pompe
    servo->activatePump(false);
    servo->openValve();
    servo->openTube(0);
    servo->openTube(1);
    servo->openTube(2);
    robot->wait(1.0);
    
        //**********************************************************
    // Rotate to measure
    //**********************************************************

    servo->moveArm(!robot->isPlayingRightSide(), arm::RAISE);
    servo->moveFinger(!robot->isPlayingRightSide(), finger::MEASURE);

    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.x = 450;
    targetPosition.y = robotdimensions::CHASSIS_WIDTH + 40 +20;
    positions.push_back(targetPosition);
    
    
    targetPosition.x = 667.5+7 * 185;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    firstColor = testExcavationSite(robot, servo);
    
    if (robot->isPlayingRightSide())
        {
        	if (firstColor== YELLOW)
        	{
			//bascule 5e et 6e
			i = 5;
			targetPosition.x = 667.5 + i * 185;
	    		traj = computeTrajectoryStraightLineToPoint(robot->getCurrentPosition(), targetPosition);
	    		robot->setTrajectoryToFollow(traj);
	    		wasMoveSuccessful = robot->waitForTrajectoryFinished();
			fifthColor = testExcavationSite(robot, servo);
			
			i = 4;
			targetPosition.x = 667.5 + i * 185;
	    		traj = computeTrajectoryStraightLineToPoint(robot->getCurrentPosition(), targetPosition);
	    		robot->setTrajectoryToFollow(traj);
	    		wasMoveSuccessful = robot->waitForTrajectoryFinished();
			fourthColor = testExcavationSite(robot, servo);
	
        	}
      
      
               else if (firstColor== PURPLE)
    	       {
	    	       //bascule 4e
	    	       i = 4;
		       targetPosition.x = 667.5 + i * 185;
	    	       traj = computeTrajectoryStraightLineToPoint(robot->getCurrentPosition(), targetPosition);
	    	       robot->setTrajectoryToFollow(traj);
	    	       wasMoveSuccessful = robot->waitForTrajectoryFinished();
		       fourthColor = testExcavationSite(robot, servo);
		}
    	

	i = 3;
    	targetPosition.x = 667.5 + i * 185;
    	traj = computeTrajectoryStraightLineToPoint(robot->getCurrentPosition(), targetPosition);
    	robot->setTrajectoryToFollow(traj);
    	wasMoveSuccessful = robot->waitForTrajectoryFinished();
	thirdColor = testExcavationSite(robot, servo);
	
    
    if (thirdColor== RED)  //dans configuration 2 ou 4
    {
       //bascule du 2e carré et 1e carré puis va tester le 4e carré
        i = 2;
    	targetPosition.x = 667.5 + i * 185;
    	traj = computeTrajectoryStraightLineToPoint(robot->getCurrentPosition(), targetPosition);
    	robot->setTrajectoryToFollow(traj);
    	wasMoveSuccessful = robot->waitForTrajectoryFinished();
	secondColor = testExcavationSite(robot, servo);
	

	i = 1;
    	targetPosition.x = 667.5 + i * 185;
    	traj = computeTrajectoryStraightLineToPoint(robot->getCurrentPosition(), targetPosition);
    	robot->setTrajectoryToFollow(traj);
    	wasMoveSuccessful = robot->waitForTrajectoryFinished();
	firstColor = testExcavationSite(robot, servo);

        
    else 
    {
    	if (thirdColor== (YELLOW or PURPLE))
    	{
		//bascule du 2e carré
		i = 2;
	    	targetPosition.x = 667.5 + i * 185;
	    	traj = computeTrajectoryStraightLineToPoint(robot->getCurrentPosition(), targetPosition);
	    	robot->setTrajectoryToFollow(traj);
	    	wasMoveSuccessful = robot->waitForTrajectoryFinished();
		secondColor = testExcavationSite(robot, servo);

    }

 
    robot->updateScore(25);
    
    //TO DO : adapter la trajectoire


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
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.2);
    robot->setTrajectoryToFollow(traj);

    // begin to move arm some time after beginning to follow traj
    robot->wait(3.0);
    servo->moveStatue(statue::TRANSPORT);

    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    std::cout << "ETAPE 1 : juste avant de reculer vers la statuette" << std::endl;

    targetPosition = robot->getCurrentPosition();


    //Go back
    traj = computeTrajectoryStraightLine(targetPosition,-261.0);
    robot->setTrajectoryToFollow(traj);
    robot->updateScore(15);

    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    std::cout << "ETAPE 2 : devant la statuette" << std::endl;

    std::cout << "ETAPE 3 : baisser les bras prendre la statuette et lever le bras" << std::endl;

    servo->moveStatue(statue::CATCH);
    servo->activateMagnet(true);
    robot->wait(0.5);
    servo->moveStatue(statue::TRANSPORT);

    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition,140.0);
    robot->setTrajectoryToFollow(traj);

    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    std::cout << "ETAPE 4 : le robot est cense avoir avance avec la statuette" << std::endl;
    
    //TO DO : recupérer éléments avec ventouse à côté de la statue


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
    traj = computeTrajectoryStraightLine(targetPosition,30.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    // TODO : placer figurine sur le piedestal ici
    servo->closeTube(0);
    servo->closeTube(2);
    robot->wait(1.0);
    servo->activatePump(true);
    servo->closeValve();
    robot->wait(3.0);
    robot->moveRail(0.15);
    servo->moveSuction(1, suction::DROP_FAKE_STATUE);
    //lacher la figurine au milieu
    robot->wait(1.0);
    servo->activatePump(false);
    servo->openValve() ;
    servo->openTube(0);
    servo->openTube(1);
    servo->openTube(2);
    robot->wait(5.0);

    // reculer
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition,-30.0);
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

    traj = computeTrajectoryRoundedCorner(positions, 150.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    robot->moveRail(0.5);

    // // TODO : unfold finger
    // servo->bougerlebrasgauchebasculedistributeur();
    // servo->bougerledoigtgauchemilieubasculedistributeur();

    // // We should have knocked down the hexagons
    // robot->updateScore(3);


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
    traj = computeTrajectoryRoundedCorner(positions, 150.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();



    std::cout << "ETAPE 5 : le robot fait marche arriere" << std::endl;


    // finally go straight
    targetPosition = robot->getCurrentPosition();
    endPosition.x = targetPosition.x ;
    endPosition.y = 2000-20-robotdimensions::CHASSIS_FRONT;
    traj = computeTrajectoryStraightLineToPoint(targetPosition,endPosition,0.0,true);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    std::cout << "ETAPE 6 : lacher la statuette" << std::endl;

    servo->activateMagnet(false);
    robot->wait(5.0);
    robot->updateScore(15);



    // go forward a little
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition,90.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();


    //**********************************************************
    // Go to the side distributor
    //**********************************************************

    //TO DO : reglage hauteur rail pour pompe
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = 1700 ;
    positions.push_back(targetPosition);
    targetPosition.x = robotdimensions::CHASSIS_FRONT + 40 + 20 ;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    robot->moveRail(0.4);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::HORIZONTAL);

    servo->closeTube(0);
    servo->closeTube(2);
    robot->wait(1.0);
    servo->activatePump(true);
    servo->closeValve();

    robot->updateScore(1);

    //**********************************************************
    // Go back to the gallery & side distributor
    //**********************************************************

    //Go back
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition,-1100.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    // robot->updateScore(3);
    // servo->bougerlebrasdroitbasculedistributeur();
    // servo->bougerledoigtdroitmilieubasculedistributeur();

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
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();

    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::VERTICAL);
    //TO DO : regler la hauteur des rails
    robot->moveRail(0.6);

    servo->activatePump(false);
    servo->openValve() ;
    servo->openTube(0);
    servo->openTube(1);
    servo->openTube(2);
    robot->wait(2.0);

    robot->updateScore(8);

    // go back a little
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition,-100.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();


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
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    //TO DO : régler la hauteur des rails
    robot->moveRail(0);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::HORIZONTAL);

    servo->closeTube(0);
    servo->closeTube(1);
    servo->closeTube(2);
    robot->wait(1.0);
    servo->activatePump(true);
    servo->closeValve();

    robot->updateScore(15);

     //**********************************************************
    // Rotate to the edge
    //**********************************************************
    positions.clear();
    targetPosition = robot->getCurrentPosition();
    positions.push_back(targetPosition);
    targetPosition.y = y_front_of_the_gallery ;
    positions.push_back(targetPosition);
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    //TO DO : régler la hauteur des rails
    robot->moveRail(0.5);
    for (int i = 0; i < 3; i++)
        servo->moveSuction(i, suction::VERTICAL);

    servo->openValve();
    servo->openTube(0);
    servo->openTube(1);
    servo->openTube(2);
    robot->wait(2.0);

    servo->activatePump(false);
    robot->updateScore(9);

    // go back a little
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition,-100.0);
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
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    robot->updateScore(15);


    // go back a big little
    targetPosition = robot->getCurrentPosition();
    traj = computeTrajectoryStraightLine(targetPosition,-150.0);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();


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
    traj = computeTrajectoryRoundedCorner(positions, 200.0, 0.5);
    robot->setTrajectoryToFollow(traj);
    wasMoveSuccessful = robot->waitForTrajectoryFinished();
    robot->updateScore(20);

    std::cout << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}
