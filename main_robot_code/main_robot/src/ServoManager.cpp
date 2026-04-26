#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"
#include "arm_code.h"

#include <miam_utils/raspberry_pi/RaspberryPi.h>

// Servo map:
#define ID_RAIL_X 10
#define ID_RAIL_Y 11
#define ID_ARM_1 12
#define ID_ARM_2 13
#define ID_ARM_3 14
#define ID_HAND_ROT 15
#define ID_HAND_TRIGHT 17
#define ID_HAND_TLEFT 16
#define ID_BED 18
#define ID_FINGER_R 19
#define ID_FINGER_L 20
#define ID_CURSOR 21

////////////////////////////////////////////////////////////////////////////
// Arm-related functions
///////////////////////////////////////////////////////////////////////////


int fromRad(double rad)
{
    return 2048 + 2048 / M_PI * rad;
}
void moveArmServos(STSScheduler *servos_, Eigen::Vector3d q)
{
    servos_->setTargetPosition(ID_ARM_1, fromRad(-q[0]), true);
    servos_->setTargetPosition(ID_ARM_2, fromRad(-q[1] - M_PI_2), true);
    servos_->setTargetPosition(ID_ARM_3, fromRad(-q[2]), true);
}

// Precomputed arm poses
Eigen::Vector3d qRaised, qGrab, qGrabMid;
Eigen::Vector3d qCalib;

Eigen::Vector3d qFold, qFoldMid;

Eigen::Vector3d qBedUnfold;

void precomputeArmIK()
{
    Eigen::Vector3d const xRaised{l1 + 0.030, -(l2 + l3 - 0.04),-M_PI_2};
    Eigen::Vector3d const xGrab{l1 + 0.020, -(l2 + l3 + 0.005),-M_PI_2};
    // Eigen::Vector3d const xGrab{l1 + 0.020, -(l2 + l3 + 0.01),-M_PI_2};
    Eigen::Vector3d const xCalib{l1 + 0.05, -(l2 + l3 - 0.02),-M_PI_2};

    Eigen::Vector3d qRef = Eigen::Vector3d::Zero();
    qRef[1] = -M_PI_2;

    qRaised = solveArmPosition(xRaised, qRef);
    qGrab = solveArmPosition(xGrab, qRaised);
    double ratio = 0.5;
    qGrabMid = solveArmPosition((1 - ratio) * xRaised + ratio * xGrab, qRaised);

    qCalib = solveArmPosition(xCalib, qRaised);

    Eigen::Vector3d const xFold{l1 - 0.08, -(l2 + l3 - 0.045),-M_PI_2};
    Eigen::Vector3d xFoldMid = (xFold + xRaised) / 2.0;
    xFoldMid[1] += 0.01;
    qFoldMid = solveArmPosition(xFoldMid, qRaised);
    qFold = solveArmPosition(xFold, qFoldMid);

    Eigen::Vector3d const xBed{l1 + 0.14, -(l2 + l3 - 0.14),-0.5};

    Eigen::Vector3d const qInt = solveArmPosition((xBed + xRaised) / 2.0, qRaised);
    qBedUnfold = solveArmPosition(xBed, qInt);
}
////////////////////////////////////////////////////////////////////////////
// End arm-related functions
///////////////////////////////////////////////////////////////////////////


ServoManager::ServoManager()
{
}

ServoManager::~ServoManager()
{
    shutdown();
}

void ServoManager::shutdown()
{
    pumpOff(Side::RIGHT);
    pumpOff(Side::LEFT);
    valveOff(Side::RIGHT);
    valveOff(Side::LEFT);
}

void ServoManager::init(RobotInterface *robot)
{
    robot_ = robot;
    servos_ = robot->getServos();
    precomputeArmIK();

    // Configure servos
    servos_->setMaxVelocity(ID_ARM_1, 2000);
    servos_->setMaxVelocity(ID_ARM_2, 2500);
    servos_->setMaxVelocity(ID_ARM_3, 2500);
    servos_->setPIDGains(ID_HAND_ROT, 20, 15, 0);


    servos_->setMode(ID_ARM_1,      STS::Mode::POSITION);
    servos_->setMode(ID_ARM_2,      STS::Mode::POSITION);
    servos_->setMode(ID_ARM_3,      STS::Mode::POSITION);
    servos_->setMode(ID_HAND_ROT,   STS::Mode::POSITION);
    servos_->setMode(ID_HAND_TRIGHT,STS::Mode::POSITION);
    servos_->setMode(ID_HAND_TLEFT, STS::Mode::POSITION);
    servos_->setMode(ID_BED,        STS::Mode::POSITION);
    // servos_->setMode(ID_FINGER_R,   STS::Mode::POSITION);
    // servos_->setMode(ID_FINGER_L,   STS::Mode::POSITION);
    servos_->setMode(ID_CURSOR,     STS::Mode::POSITION);


    // Setup rails
    railX_ = servos_->createRail(ID_RAIL_X, 6, 5500, true);
    railY_ = servos_->createRail(ID_RAIL_Y, 25, 4400, false);

    cursorFold();
    bedFold();
    servos_->setTargetPosition(ID_ARM_1, 2048);
    robot_->wait(0.5);
    moveArm(ArmPosition::CALIBRATE);
    translateSuction(Side::RIGHT, 0);
    translateSuction(Side::LEFT, 0);
    RPi_setupGPIO(12, PI_GPIO_OUTPUT);
    RPi_setupGPIO(13, PI_GPIO_OUTPUT);
    RPi_setupGPIO(23, PI_GPIO_OUTPUT);
    RPi_setupGPIO(24, PI_GPIO_OUTPUT);
    pumpOff(Side::RIGHT);
    pumpOff(Side::LEFT);
    valveOff(Side::RIGHT);
    valveOff(Side::LEFT);
    // Start calib
    servos_->startRailCalibration();
}

void ServoManager::testArm()
{
    std::string input;
    while (true)
    {
        std::getline(std::cin, input);
        std::cout << "raise" << std::endl;
        moveArm(ArmPosition::RAISE);
        std::getline(std::cin, input);
        // Bed unfold
        moveArmServos(servos_, qBedUnfold);
        robot_->wait(0.5);
        bedUnfold();
        robot_->wait(0.5);
        bedFold();
        moveArm(ArmPosition::RAISE);
        robot_->wait(0.5);
        // bedFold();
        // std::getline(std::cin, input);

        // std::getline(std::cin, input);
        // std::cout << "raise" << std::endl;
        // moveArm(ArmPosition::RAISE);
        // std::getline(std::cin, input);
        // std::cout << "doGrab" << std::endl;
        // pumpOn(Side::RIGHT);
        // pumpOn(Side::LEFT);
        // doGrab();
        // std::getline(std::cin, input);
        // std::cout << "raise" << std::endl;
        // moveArm(ArmPosition::RAISE);
        // std::getline(std::cin, input);
        // std::cout << "folding" << std::endl;
        // moveArmServos(servos_, qFoldMid);
        // robot_->wait(1.0);
        // moveArmServos(servos_, qFold);
        // robot_->wait(1.0);
        // std::getline(std::cin, input);
        // releaseSuction();


    }
}

void ServoManager::moveRails(RailPosition const& position)
{
    switch(position)
    {
        case RailPosition::FORWARD:
            railX_->move(0.0);
            railY_->move(0.5);
            break;
        case RailPosition::INTERNAL:
            railX_->move(1.0);
            railY_->move(0.5);
            break;
        default: break;
    }
}

bool ServoManager::areRailsMoving()
{
    return railX_->isMoving() || railY_->isMoving();
}

void ServoManager::cursorFold()
{
    servos_->setTargetPosition(ID_CURSOR, 2048);
}
void ServoManager::cursorUnfold()
{
    servos_->setTargetPosition(ID_CURSOR, 3350);
}

void ServoManager::bedFold()
{
    servos_->setTargetPosition(ID_BED, 2048);
}
void ServoManager::bedUnfold()
{
    servos_->setTargetPosition(ID_BED, 4080);
}

void ServoManager::moveArm(ArmPosition const& position)
{
    currentArmPosition = position;
    switch(position)
    {
        case ArmPosition::CALIBRATE:
            moveArmServos(servos_, qCalib);
            servos_->setTargetPosition(ID_HAND_ROT, 2048);
            break;
        case ArmPosition::GRAB:
            moveArmServos(servos_, qGrab);
            break;
        case ArmPosition::RAISE:
            moveArmServos(servos_, qRaised);
            break;
        case ArmPosition::FOLD_MID:
            moveArmServos(servos_, qFoldMid);
            break;
        case ArmPosition::FOLD:
            moveArmServos(servos_, qFold);
            break;
        case ArmPosition::CAMERA_POSE:
            servos_->setTargetPosition(ID_ARM_1, 2900, true);
            servos_->setTargetPosition(ID_ARM_2, 2600, true);
            servos_->setTargetPosition(ID_ARM_3, 1450, true);
            break;
        default: break;
    }
}

void ServoManager::doGrab()
{
    // moveArmServos(servos_, qGrabMid);
    // robot_->wait(0.5);
    moveArmServos(servos_, qGrab);
    robot_->wait(1.0);
    servos_->disable(ID_ARM_1);
    servos_->disable(ID_ARM_2);
    servos_->disable(ID_ARM_3);
    robot_->wait(0.8);
    servos_->enable(ID_ARM_1);
    servos_->enable(ID_ARM_2);
    servos_->enable(ID_ARM_3);
}

void ServoManager::hideArm()
{
    translateSuction(Side::RIGHT, 0.0);
    translateSuction(Side::LEFT, 0.0);
    releaseSuction();
    if (currentArmPosition != ArmPosition::CAMERA_POSE)
    {
        moveArm(ArmPosition::FOLD_MID);
        robot_->wait(0.2);
        moveArm(ArmPosition::CAMERA_POSE);
    }
}

void ServoManager::unhideArm()
{
    moveArm(ArmPosition::FOLD);
    robot_->wait(0.5);
    // moveArm(ArmPosition::FOLD_MID);
    // robot_->wait(0.2);
    moveArm(ArmPosition::RAISE);
}

#define SUCTION_RANGE 150


void ServoManager::translateSuction(Side const side, double const ratio)
{
    int const servoIds[2] = {ID_HAND_TRIGHT, ID_HAND_TLEFT};
    int const sign[2] = {1, 1};
    int const closePosition[2] = {500, 610};

    int const idx = static_cast<int>(side);
    servos_->setTargetPosition(servoIds[idx], closePosition[idx] + sign[idx] * ratio * SUCTION_RANGE);
}


void ServoManager::pumpOn(Side const side)
{
    int const idx = static_cast<int>(side);
    RPi_writeGPIO(12 + idx, HIGH);
    valveOff(side);
}

void ServoManager::pumpOff(Side const side)
{
    int const idx = static_cast<int>(side);
    RPi_writeGPIO(12 + idx, LOW);
}
void ServoManager::valveOn(Side const side)
{
    int const idx = static_cast<int>(side);
    RPi_writeGPIO(23 + idx, HIGH);
}

void ServoManager::valveOff(Side const side)
{
    int const idx = static_cast<int>(side);
    RPi_writeGPIO(23 + idx, LOW);
}

void ServoManager::grabCrates()
{
    // Look for tags in the image
    std::vector<Tag> tags;
    for (int i = 0; i < 5; i++)
    {
        tags = visionHandler_.getTags();
        if (tags.size() == 4)
            break;
        robot_->wait(0.050); // Wait, with enough time for the camera to get a new frame.
    }
    // Analyze: what did we see
    if (tags.size() == 0)
    {
        robot_->logger_ << "[ServoManager] Grab crates: no tag seen, exiting." << std::endl;
        return;
    }

    robot_->logger_ << "[ServoManager] Tags:";
    for (auto const& t : tags)
    {
        if (t.markerId == BLUE)
            robot_->logger_ << "blue ";
        if (t.markerId == YELLOW)
            robot_->logger_<< "yellow ";
    }
    robot_->logger_ << std::endl;
    robot_->logger_ << "[ServoManager] Tags Y pos:";
    for (auto const& t : tags)
    {
        robot_->logger_ << t.position.y() << " ";
    }
    robot_->logger_ << std::endl;


    if (tags.size() != 4)
    {
        robot_->logger_ << "[ServoManager::grabCrates] Incorrect number of crates, exiting." << std::endl;
        return;
    }

    unhideArm();
    robot_->wait(1.0);

    int const myColor = (robot_->isPlayingRightSide() ? BLUE : YELLOW);
    int const opponentColor = (robot_->isPlayingRightSide() ? YELLOW : BLUE);

    std::vector<int> myTags;
    std::vector<int> opponentTags;
    for (unsigned int i = 0; i < tags.size(); i++)
    {
        if (tags.at(i).markerId == myColor)
            myTags.push_back(i);
        if (tags.at(i).markerId == opponentColor)
            opponentTags.push_back(i);
    }

    // Do we need to put something in the bed?
    // if (opponentTags.size() > 0)
    // {
    //     grabTags(tags, opponentTags);
    //     moveCratesInBed();
    //     robot_->getGameState()->isBedFull = true;
    // }
    if (myTags.size() > 0)
    {
        grabTags(tags, myTags);
        robot_->getGameState()->isClawFull = true;
    }
}

void ServoManager::grabTags(std::vector<Tag> const& tags, std::vector<int> tagsToGrab)
{
    if (tagsToGrab.size() == 0)
        return;

    // TODO
    if (tagsToGrab.size() == 1)
        return;

    int leftTagIdx = tagsToGrab[0];
    int rightTagIdx = tagsToGrab[1];

    robot_->logger_ << "[ServoManager::grabTags] Grabbing tags indexed " << leftTagIdx << " " << rightTagIdx << std::endl;

    double suctionRight = 0.0, suctionLeft = 0.0, rail = 0.0;
    // Do all 5 cases
    if (leftTagIdx == 0 && rightTagIdx == 1)
    {
        suctionLeft = 0.0;
        suctionRight = 0.0;
        rail = 1.0;
    }
    else if (leftTagIdx == 0 && rightTagIdx == 2)
    {
        suctionLeft = 1.0;
        suctionRight = 0.0;
        rail = 0.5;
    }
    else if (leftTagIdx == 0 && rightTagIdx == 3)
    {
        suctionLeft = 1.0;
        suctionRight = 1.0;
        rail = 0.5;
    }
    else if (leftTagIdx == 1 && rightTagIdx == 2)
    {
        suctionLeft = 0.0;
        suctionRight = 0.0;
        rail = 0.5;
    }
    else if (leftTagIdx == 1 && rightTagIdx == 3)
    {
        suctionLeft = 0.0;
        suctionRight = 1.0;
        rail = 0.5;
    }
    else if (leftTagIdx == 2 && rightTagIdx == 3)
    {
        suctionLeft = 0.0;
        suctionRight = 0.0;
        rail = 0.0;
    }

    // Perform motion
    translateSuction(Side::LEFT, suctionLeft);
    translateSuction(Side::RIGHT, suctionRight);
    railY_->move(rail);
    while (areRailsMoving())
        robot_->wait(0.050);

    pumpOn(Side::RIGHT);
    pumpOn(Side::LEFT);
    doGrab();
    moveArm(ArmPosition::RAISE);
    robot_->wait(0.5);
    translateSuction(Side::LEFT, 0.0);
    translateSuction(Side::RIGHT, 0.0);
    railY_->move(0.5);
    while (areRailsMoving())
        robot_->wait(0.050);

    // //
    // // Fow now, we don't care about angles, but want to get spacing right.

    // // Enforce a 2-elemnt grab.
    // bool rightActive = true, leftActive = true;
    // // Drop elements if we have too many
    // int leftTag = 0, rightTag = 0;

    // if (tagsToGrab.size() > 2)
    // {
    //     robot_->logger_ << "[ServoManager::grabTags] Too many tags, keeping only 2" << std::endl;
    //     if (tagsToGrab.at(0) == 0)
    //         tagsToGrab.erase(tagsToGrab.begin());
    // }
    // if (tagsToGrab.size() == 2)
    // {
    //     leftTag = tagsToGrab.at(0);
    //     rightTag = tagsToGrab.at(1);
    // }
    // else
    // {
    //     int singleTag = tagsToGrab.at(0);
    //     if (singleTag < 2)
    //     {
    //         leftTag = singleTag;
    //         rightActive = false;
    //     }
    //     else
    //     {
    //         rightTag = singleTag;
    //         leftActive = false;
    //     }
    // }
    // robot_->logger_ << "[ServoManager::grabTags] Grabbing tags indexed " << (leftActive ? std::to_string(leftTag) + " " : "[no left] ");
    // robot_->logger_ << (rightActive ? std::to_string(rightTag) + " " : "[no right] ") << std::endl;

    // // Position suction according to grabbing pattern.
    // double leftPos = (leftTag == 0 ? 1.0: 0.0);
    // double rightPos = (rightTag == 3 ? 1.0: 0.0);
    // if (!leftActive)
    //     leftPos = 0.0;
    // if (!rightActive)
    //     rightPos = 0.0;

    // translateSuction(Side::LEFT, leftPos);
    // translateSuction(Side::RIGHT, rightPos);
    // double const BLOCK_WIDTH = 0.050;

    // double leftY = tags[leftTag].position.y();
    // double rightY = tags[rightTag].position.y();
    // if (!leftActive)
    //     leftY = rightY + BLOCK_WIDTH * (1 + rightPos);
    // if (!rightActive)
    //     rightY = leftY - BLOCK_WIDTH * (1 + leftPos);


    // robot_->logger_ << "[ServoManager::grabTags] Suction position: " << leftPos << " " << rightPos << std::endl;
    // robot_->logger_ << "[ServoManager::grabTags] Marker positions: " << leftY << " " << rightY << std::endl;

    // double yOffset = (rightY + leftY) / 2.0;
    // robot_->logger_ << "[ServoManager::grabTags] Y offset" << yOffset << std::endl;
    // // Offset this based on how wide the claw is.
    // yOffset += BLOCK_WIDTH / 2 * (-leftPos + rightPos);
    // robot_->logger_ << "[ServoManager::grabTags] Y offset with claw comp" << yOffset << std::endl;

    // double const yRailScaling = (0.5 / 0.04);
    // // Rail is backward: 0 = going to left
    // double const yRailPosition = 0.5 - std::clamp(yRailScaling * yOffset, -0.5, 0.5);
    // robot_->logger_ << "[ServoManager::grabTags] Y rail position" << yRailPosition << std::endl;

    // // Move rail accordingly.
    // // if (yRailPosition)

    // // Grab
    // if (rightActive)
    //     pumpOn(Side::RIGHT);
    // if (leftActive)
    //     pumpOn(Side::LEFT);
    // doGrab();

    // // Lift and fold suctions
    // moveArm(ArmPosition::RAISE);
}

void ServoManager::moveCratesInBed()
{
    translateSuction(Side::LEFT, 0.0);
    translateSuction(Side::RIGHT, 0.0);

    moveArm(ArmPosition::FOLD_MID);
    robot_->wait(0.5);
    moveArm(ArmPosition::FOLD);
    robot_->wait(0.2);
    moveRails(RailPosition::INTERNAL);
    while (areRailsMoving())
        robot_->wait(0.1);
    releaseSuction();
    moveRails(RailPosition::FORWARD);
    while (areRailsMoving())
        robot_->wait(0.1);
    moveArm(ArmPosition::RAISE);
}

void ServoManager::dropCrates()
{
    if (robot_->getGameState()->isBedFull)
    {
        emptyBed();
        robot_->getGameState()->isBedFull = false;
    }

    if (robot_->getGameState()->isClawFull)
    {
        moveArm(ArmPosition::GRAB);
        releaseSuction();
        robot_->wait(0.25);
        moveArm(ArmPosition::RAISE);
        robot_->wait(0.25);
        robot_->getGameState()->isClawFull = false;
    }
}

void ServoManager::emptyBed()
{
    moveArmServos(servos_, qBedUnfold);
    robot_->wait(0.5);
    bedUnfold();
    robot_->wait(0.5);
    bedFold();
    moveArm(ArmPosition::RAISE);
    robot_->wait(0.5);
}

void ServoManager::releaseSuction()
{
    pumpOff(Side::RIGHT);
    pumpOff(Side::LEFT);
    valveOn(Side::RIGHT);
    valveOn(Side::LEFT);
    robot_->wait(0.3);
    valveOff(Side::RIGHT);
    valveOff(Side::LEFT);
}