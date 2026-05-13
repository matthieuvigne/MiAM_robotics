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
// #define ID_HAND_ROT 15
#define ID_HAND_TRIGHT 17
#define ID_HAND_TLEFT 16
#define ID_BED 18
#define ID_FINGER_R 19
#define ID_FINGER_L 20
#define ID_CURSOR 21
#define ID_CURSOR_RIGHT 22

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
Eigen::Vector3d qCursor;


void precomputeArmIK()
{
    Eigen::Vector3d const xRaised{l1 + 0.040, -(l2 + l3 - 0.04),-M_PI_2};
    Eigen::Vector3d const xGrab{l1 + 0.030, -(l2 + l3 + 0.005),-M_PI_2};
    // Eigen::Vector3d const xGrab{l1 + 0.020, -(l2 + l3 + 0.01),-M_PI_2};
    Eigen::Vector3d const xCalib{l1 + 0.05, -(l2 + l3 - 0.02),-M_PI_2};

    Eigen::Vector3d qRef = Eigen::Vector3d::Zero();
    qRef[1] = -M_PI_2;

    qRaised = solveArmPosition(xRaised, qRef);
    qGrab = solveArmPosition(xGrab, qRaised);
    double ratio = 0.7;
    qGrabMid = solveArmPosition((1 - ratio) * xRaised + ratio * xGrab, qRaised);

    qCalib = solveArmPosition(xCalib, qRaised);

    Eigen::Vector3d const xFold{l1 - 0.08, -(l2 + l3 - 0.045),-M_PI_2};
    Eigen::Vector3d xFoldMid = (xFold + xRaised) / 2.0;
    // xFoldMid[1] += 0.005;
    qFoldMid = solveArmPosition(xFoldMid, qRaised);
    qFoldMid[2] -= 0.25;
    qFold = solveArmPosition(xFold, qFoldMid);

    Eigen::Vector3d const xBed{l1 + 0.13, -(l2 + l3 - 0.13),-0.1};

    Eigen::Vector3d const qInt = solveArmPosition((xBed + xRaised) / 2.0, qRaised);
    qBedUnfold = solveArmPosition(xBed, qInt);

    qCursor = qRaised;
    qCursor[1] -= 0.5;
    qCursor[2] += 1.5;
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
    visionHandler_ = robot->getVisionHandler();
    precomputeArmIK();

    // Configure servos
    servos_->setMaxVelocity(ID_ARM_1, 2000);
    servos_->setMaxVelocity(ID_ARM_2, 2500);
    servos_->setMaxVelocity(ID_ARM_3, 2500);
    // servos_->setPIDGains(ID_HAND_ROT, 20, 15, 0);
    servos_->writeTwoBytesRegister(ID_ARM_1, STS::registers::CURRENT_PROTECTION_TH, 350);
    servos_->writeRegister(ID_ARM_1, STS::registers::TORQUE_PROTECTION_TH, 40);


    servos_->setMode(ID_ARM_1,      STS::Mode::POSITION);
    servos_->setMode(ID_ARM_2,      STS::Mode::POSITION);
    servos_->setMode(ID_ARM_3,      STS::Mode::POSITION);
    servos_->setMode(ID_HAND_TRIGHT, STS::Mode::POSITION);
    servos_->setMode(ID_HAND_TLEFT,  STS::Mode::POSITION);
    servos_->setMode(ID_BED,         STS::Mode::POSITION);
    servos_->setMode(ID_FINGER_R,    STS::Mode::POSITION);
    servos_->setMode(ID_FINGER_L,    STS::Mode::POSITION);
    servos_->setMode(ID_CURSOR,      STS::Mode::POSITION);
    servos_->setMode(ID_CURSOR_RIGHT,STS::Mode::POSITION);


    // Setup rails
    railX_ = servos_->createRail(ID_RAIL_X, 6, 5500, true);
    railY_ = servos_->createRail(ID_RAIL_Y, 25, 4575, false);

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
    fingerOpen();
    // Start calib
    servos_->startRailCalibration();
}

void ServoManager::testArm()
{
    std::string input;
    pumpOn(Side::RIGHT);
    pumpOn(Side::LEFT);
    while (true)
    {
        std::getline(std::cin, input);
        std::cout << "raise" << std::endl;
        moveArm(ArmPosition::RAISE);
        std::getline(std::cin, input);

        servos_->setMaxVelocity(ID_ARM_1, 1000);
        servos_->setMaxVelocity(ID_ARM_2, 1600);
        servos_->setMaxVelocity(ID_ARM_3, 1800);
        moveArm(ArmPosition::FOLD_MID);
        robot_->wait(0.5);
        moveArm(ArmPosition::FOLD);
        robot_->wait(0.4);
        servos_->setMaxVelocity(ID_ARM_1, 2000);
        servos_->setMaxVelocity(ID_ARM_2, 2500);
        servos_->setMaxVelocity(ID_ARM_3, 2500);
        std::cout << "FOLD DONE" << std::endl;
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
        case RailPosition::DROP:
            railX_->move(0.0);
            if (robot_->isPlayingRightSide())
                railY_->move(0.0);
            else
                railY_->move(1.0);
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
    servos_->setTargetPosition(ID_CURSOR, 100);
    servos_->setTargetPosition(ID_CURSOR_RIGHT, 900);
}

void ServoManager::cursorUnfold()
{
    if (robot_->isPlayingRightSide())
        servos_->setTargetPosition(ID_CURSOR_RIGHT, 525);
    else
        servos_->setTargetPosition(ID_CURSOR, 475);
}

void ServoManager::bedFold()
{
    servos_->setTargetPosition(ID_BED, 2048);
}
void ServoManager::bedUnfold()
{
    servos_->setTargetPosition(ID_BED, 3600);
}
void ServoManager::bedMidUnfold()
{
    servos_->setTargetPosition(ID_BED, 2600);
}

void ServoManager::moveArm(ArmPosition const& position)
{
    currentArmPosition = position;
    switch(position)
    {
        case ArmPosition::CALIBRATE:
            moveArmServos(servos_, qCalib);
            // servos_->setTargetPosition(ID_HAND_ROT, 2048);
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
        case ArmPosition::BED_UNFOLD:
            moveArmServos(servos_, qBedUnfold);
            break;
        case ArmPosition::DO_CURSOR:
            moveArmServos(servos_, qCursor);
            break;
        default: break;
    }
}

void ServoManager::doGrab()
{
    // moveArmServos(servos_, qGrabMid);
    // robot_->wait(0.5);
    moveArmServos(servos_, qGrab);
    robot_->wait(0.3);
    servos_->disable(ID_ARM_1);
    servos_->disable(ID_ARM_2);
    servos_->disable(ID_ARM_3);
    robot_->wait(0.4);
    servos_->enable(ID_ARM_1);
    servos_->enable(ID_ARM_2);
    servos_->enable(ID_ARM_3);
    moveArmServos(servos_, qGrab);
    robot_->wait(0.2);
    moveArmServos(servos_, qGrabMid);
    robot_->wait(0.1);
}

void ServoManager::hideArm()
{
    bedFold();
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
    robot_->wait(0.4);
    moveArm(ArmPosition::RAISE);
}

#define SUCTION_RANGE 220


void ServoManager::translateSuction(Side const side, double const ratio)
{
    int const servoIds[2] = {ID_HAND_TRIGHT, ID_HAND_TLEFT};
    int const sign[2] = {1, 1};
    int const closePosition[2] = {460, 600};

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

CameraResult ServoManager::cameraDetectCrates()
{
    // Look for tags in the image
    CameraResult result;
    result.cratesPresent = false;
    if (visionHandler_ != nullptr)
    {
        for (int i = 0; i < 3; i++)
        {
            result.tags = visionHandler_->getTags();
            if (result.tags.size() == 4)
                break;
            robot_->wait(0.120); // Wait, with enough time for the camera to get a new frame.
        }
    }
    // Analyze: what did we see
    if (result.tags.size() == 0)
    {
        robot_->logger_ << "[ServoManager] cameraDetectCrates: no tag seen, exiting." << std::endl;
        return result;
    }

    robot_->logger_ << "[ServoManager] Tags:";
    for (auto const& t : result.tags)
    {
        if (t.markerId == BLUE)
            robot_->logger_ << "blue ";
        if (t.markerId == YELLOW)
            robot_->logger_<< "yellow ";
    }
    robot_->logger_ << std::endl;
    result.lateralOffset = 0.0;
    robot_->logger_ << "[ServoManager] Tags Y pos:";
    for (auto const& t : result.tags)
    {
        robot_->logger_ << t.position.y() << " ";
        result.lateralOffset += t.position.y();
    }
    robot_->logger_ << std::endl;
    robot_->logger_ << "[ServoManager] Tags X pos:";
    for (auto const& t : result.tags)
    {
        robot_->logger_ << t.position.x() << " ";
    }
    robot_->logger_ << std::endl;
    result.lateralOffset /= result.tags.size();
    result.lateralOffset *= 1000.0; // Motion planning is done in mm.

    if (result.tags.size() != 4)
    {
        robot_->logger_ << "[ServoManager::grabCrates] Incorrect number of crates, exiting." << std::endl;
        return result;
    }
    result.cratesPresent = true;
    return result;
}

void ServoManager::grabCrates(CameraResult const& cameraResult)
{
    if (!cameraResult.cratesPresent)
        return;

    // unhideArm();
    // robot_->wait(0.2);

    int const myColor = (robot_->isPlayingRightSide() ? BLUE : YELLOW);
    int const opponentColor = (robot_->isPlayingRightSide() ? YELLOW : BLUE);

    std::vector<int> myTags;
    std::vector<int> opponentTags;
    for (unsigned int i = 0; i < cameraResult.tags.size(); i++)
    {
        if (cameraResult.tags.at(i).markerId == myColor)
            myTags.push_back(i);
        if (cameraResult.tags.at(i).markerId == opponentColor)
            opponentTags.push_back(i);
    }

    // Do we need to put something in the bed?
    if (opponentTags.size() > 0 && robot_->getMatchTime() < 75)
    {
        if (robot_->getMatchTime() > 75)
        {
            robot_->logger_ << "[ServoManager::grabTags] Abort oponent grab, not enough time left" << std::endl;
        }
        else
        {
            grabTags(cameraResult.tags, opponentTags);
            moveCratesInBed();
            fingerClose();
            robot_->getGameState()->isBedFull = true;
        }
    }
    if (myTags.size() > 0)
    {
        grabTags(cameraResult.tags, myTags, true);
        robot_->getGameState()->isClawFull = true;
    }
}

void ServoManager::grabTags(std::vector<Tag> const& tags, std::vector<int> tagsToGrab, bool secondGrab)
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
    bool asymRight = false;
    bool asymLeft = false;
    double lateralAmount = secondGrab ? 0.6: 0.4;

    // Do all 5 cases
    if (leftTagIdx == 0 && rightTagIdx == 1)
    {
        suctionLeft = lateralAmount;
        suctionRight = 0.0;
        asymLeft = true;
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
        suctionRight = lateralAmount;
        asymRight = true;
        rail = 0.0;
    }

    // Perform motion
    translateSuction(Side::LEFT, suctionLeft);
    translateSuction(Side::RIGHT, suctionRight);
    railY_->move(rail);
    while (areRailsMoving())
    {
        robot_->wait(0.050);
    }

    if (!asymLeft)
        pumpOn(Side::RIGHT);
    if (!asymRight)
        pumpOn(Side::LEFT);
    doGrab();
    if (asymLeft || asymRight)
    {
        Side asymSide = (asymLeft ? Side::LEFT : Side::RIGHT);
        while (lateralAmount >= 0)
        {
            translateSuction(asymSide, lateralAmount);
            lateralAmount -= 0.10;
            robot_->wait(0.050);
        }
        pumpOn(Side::RIGHT);
        pumpOn(Side::LEFT);
        doGrab();
    }
    moveArm(ArmPosition::RAISE);
    robot_->wait(0.4);
    translateSuction(Side::LEFT, 0.0);
    translateSuction(Side::RIGHT, 0.0);
    railY_->move(0.5);
}

void ServoManager::moveCratesInBed()
{
    translateSuction(Side::LEFT, 0.0);
    translateSuction(Side::RIGHT, 0.0);

    servos_->setMaxVelocity(ID_ARM_1, 1000);
    servos_->setMaxVelocity(ID_ARM_2, 1600);
    servos_->setMaxVelocity(ID_ARM_3, 1800);
    moveArm(ArmPosition::FOLD_MID);
    robot_->wait(0.4);
    moveArm(ArmPosition::FOLD);
    robot_->wait(0.3);
    moveRails(RailPosition::INTERNAL);
    while (areRailsMoving())
        robot_->wait(0.050);
    releaseSuction();
    servos_->setMaxVelocity(ID_ARM_1, 2000);
    servos_->setMaxVelocity(ID_ARM_2, 2500);
    servos_->setMaxVelocity(ID_ARM_3, 2500);
    moveRails(RailPosition::FORWARD);
    while (areRailsMoving())
    {
        if (railX_->getCurrentPosition() < 0.2)
            moveArm(ArmPosition::RAISE);
        robot_->wait(0.050);
    }
    moveArm(ArmPosition::RAISE);
}

void ServoManager::dropCrates()
{
    moveArm(ArmPosition::GRAB);
    robot_->wait(0.35);
    releaseSuction();
    moveArm(ArmPosition::RAISE);
    robot_->getGameState()->isClawFull = false;
}

void ServoManager::emptyBed()
{
    moveArm(ArmPosition::BED_UNFOLD);
    bedUnfold();
    robot_->wait(1.10);
    fingerOpen();
    bedFold();
    robot_->wait(0.5);
    moveArm(ArmPosition::RAISE);
    robot_->getGameState()->isBedFull = false;
}

void ServoManager::releaseSuction()
{
    pumpOff(Side::RIGHT);
    pumpOff(Side::LEFT);
    valveOn(Side::RIGHT);
    valveOn(Side::LEFT);
    robot_->wait(0.2);
    valveOff(Side::RIGHT);
    valveOff(Side::LEFT);
}

#define FINGER_RIGHT_OPEN 480
#define FINGER_LEFT_OPEN 500

void ServoManager::fingerOpen()
{
    servos_->setTargetPosition(ID_FINGER_R, FINGER_RIGHT_OPEN);
    servos_->setTargetPosition(ID_FINGER_L, FINGER_LEFT_OPEN);
}

void ServoManager::fingerClose()
{
    int closeAmount = 250;
    servos_->setTargetPosition(ID_FINGER_R, FINGER_RIGHT_OPEN + closeAmount);
    servos_->setTargetPosition(ID_FINGER_L, FINGER_LEFT_OPEN - closeAmount);
}


std::string ServoManager::updateInfoString()
{
    if (visionHandler_ == nullptr)
        return "No tags seen!";
    std::vector<Tag> tags = visionHandler_->getTags();
    std::string result;
    if (tags.size() == 0)
        result = "No tags seen!";
    else
    {
        result = "Tags: ";
        for (auto const& t : tags)
        {
            if (t.markerId == BLUE)
                result += "blue ";
            if (t.markerId == YELLOW)
                result += "yellow ";
        }

    }
    return result;
}