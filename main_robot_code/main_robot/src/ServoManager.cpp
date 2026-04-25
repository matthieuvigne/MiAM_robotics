#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"

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
    RPi_writeGPIO(23, LOW);
    RPi_writeGPIO(24, LOW);
}

void ServoManager::init(RobotInterface *robot)
{
    robot_ = robot;
    servos_ = robot->getServos();

    servos_->setMode(ID_ARM_1,      STS::Mode::POSITION);
    servos_->setMode(ID_ARM_2,      STS::Mode::POSITION);
    servos_->setMode(ID_ARM_3,      STS::Mode::POSITION);
    servos_->setMode(ID_HAND_ROT,   STS::Mode::POSITION);
    servos_->setMode(ID_HAND_TRIGHT,STS::Mode::POSITION);
    servos_->setMode(ID_HAND_TLEFT, STS::Mode::POSITION);
    servos_->setMode(ID_BED,        STS::Mode::POSITION);
    // servos_->setMode(ID_FINGER_R,   STS::Mode::POSITION);
    // servos_->setMode(ID_FINGER_L,   STS::Mode::POSITION);
    // servos_->setMode(ID_CURSOR,     STS::Mode::POSITION);

    // Configure servos
    servos_->setMaxVelocity(ID_ARM_1, 1300);
    servos_->setMaxVelocity(ID_ARM_2, 2000);
    servos_->setMaxVelocity(ID_ARM_3, 3000);
    servos_->setPIDGains(ID_HAND_ROT, 20, 15, 0);

    // Setup rails
    railX_ = servos_->createRail(ID_RAIL_X, 6, 5500, true);
    railY_ = servos_->createRail(ID_RAIL_Y, 25, 4400, false);

    cursorFold();
    bedFold();
    moveArm(ArmPosition::CALIBRATE);
    translateSuction(Side::RIGHT, 0);
    translateSuction(Side::LEFT, 0);
    RPi_setupGPIO(12, PI_GPIO_OUTPUT);
    RPi_setupGPIO(13, PI_GPIO_OUTPUT);
    RPi_setupGPIO(23, PI_GPIO_OUTPUT);
    RPi_setupGPIO(24, PI_GPIO_OUTPUT);
    pumpOff(Side::RIGHT);
    pumpOff(Side::LEFT);
    // Start calib
    // servos_->startRailCalibration();
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
    servos_->setTargetPosition(ID_CURSOR, 3200);
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
            servos_->setTargetPosition(ID_ARM_1, 2048);
            servos_->setTargetPosition(ID_ARM_2, 1500);
            servos_->setTargetPosition(ID_ARM_3, 2048);
            servos_->setTargetPosition(ID_HAND_ROT, 2048);
            break;
        case ArmPosition::GRAB:
            servos_->setTargetPosition(ID_ARM_1, 2200);
            servos_->setTargetPosition(ID_ARM_2, 1650);
            servos_->setTargetPosition(ID_ARM_3, 2300);
            break;
        case ArmPosition::RAISE:
            servos_->setTargetPosition(ID_ARM_1, 1948);
            servos_->setTargetPosition(ID_ARM_2, 2000);
            servos_->setTargetPosition(ID_ARM_3, 2200);
            // servos_->setTargetPosition(ID_ARM_1, 1648);
            // servos_->setTargetPosition(ID_ARM_2, 2200);
            // servos_->setTargetPosition(ID_ARM_3, 2300);
            break;
        case ArmPosition::FOLD_MID:
            servos_->setTargetPosition(ID_ARM_1, 1750);
            servos_->setTargetPosition(ID_ARM_2, 2580);
            servos_->setTargetPosition(ID_ARM_3, 1550);
            break;
        case ArmPosition::FOLD:
            servos_->setTargetPosition(ID_ARM_1, 2000);
            servos_->setTargetPosition(ID_ARM_2, 2780);
            servos_->setTargetPosition(ID_ARM_3, 1450);
            break;
        case ArmPosition::CAMERA_POSE:
            servos_->setTargetPosition(ID_ARM_1, 2900);
            servos_->setTargetPosition(ID_ARM_2, 2700);
            servos_->setTargetPosition(ID_ARM_3, 1450);
            break;
        default: break;
    }
}

void ServoManager::hideArm()
{
    translateSuction(Side::RIGHT, 0.0);
    translateSuction(Side::LEFT, 0.0);
    if (currentArmPosition != ArmPosition::CAMERA_POSE)
    {
        moveArm(ArmPosition::FOLD_MID);
        robot_->wait(0.5);
        moveArm(ArmPosition::CAMERA_POSE);
    }
}

void ServoManager::unhideArm()
{
    moveArm(ArmPosition::FOLD);
    robot_->wait(0.5);
    moveArm(ArmPosition::FOLD_MID);
    robot_->wait(0.5);
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
    RPi_writeGPIO(23 + idx, LOW);
}

void ServoManager::pumpOff(Side const side)
{
    int const idx = static_cast<int>(side);
    RPi_writeGPIO(12 + idx, LOW);
    RPi_writeGPIO(23 + idx, HIGH);
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
    unhideArm();

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
    if (opponentTags.size() > 0)
    {
        grabTags(tags, opponentTags);
        // moveCratesInBed();
        // robot_->getGameState()->isBedFull = true;
    }
    // if (myTags.size() > 0)
    // {
    //     grabTags(tags, myTags);
    //     robot_->getGameState()->isClawFull = true;
    // }
}

void ServoManager::grabTags(std::vector<Tag> const& tags, std::vector<int> tagsToGrab)
{
    if (tagsToGrab.size() == 0)
        return;
    // Fow now, we don't care about angles, but want to get spacing right.

    // Enforce a 2-elemnt grab.
    bool rightActive = true, leftActive = true;
    // Drop elements if we have too many
    int leftTag = 1, rightTag = 2;

    if (tagsToGrab.size() > 2)
    {
        robot_->logger_ << "[ServoManager::grabTags] Too many tags, keeping only 2" << std::endl;
        if (tagsToGrab.at(0) == 0)
            tagsToGrab.erase(tagsToGrab.begin());
    }
    if (tagsToGrab.size() == 2)
    {
        leftTag = tagsToGrab.at(0);
        rightTag = tagsToGrab.at(1);
    }
    else
    {
        int singleTag = tagsToGrab.at(0);
        if (singleTag < 2)
        {
            leftTag = singleTag;
            rightActive = false;
        }
        else
        {
            rightTag = singleTag;
            leftActive = false;
        }
    }
    robot_->logger_ << "[ServoManager::grabTags] Grabbing tags " << leftTag << (leftActive ? " " : "[no] ");
    robot_->logger_  << rightTag << (rightActive ? " " : "[no] ") << std::endl;

    // Position suction according to grabbing pattern.
    int leftPos = (leftTag == 0 ? 1.0: 0.0);
    int rightPos = (rightTag == 3 ? 1.0: 0.0);

    translateSuction(Side::LEFT, leftPos);
    translateSuction(Side::RIGHT, rightPos);
    std::cout << "left pos" << leftPos << std::endl;
    std::cout << "right pos" << rightPos << std::endl;

    // Move rail to barycenter

    // Figure out spacing.
    // double const baricenterBias = 0;
    // int const blockDist = abs(tagsToGrab[1] - tagsToGrab[0]);

    // // Grab
    // if (rightActive)
    //     pumpOn(Side::RIGHT);
    // if (leftActive)
    //     pumpOn(Side::LEFT);


    // moveArm(ArmPosition::GRAB);
    // robot_->wait(1.0);
    // moveArm(ArmPosition::RAISE);
    // robot_->wait(0.3);
}

void ServoManager::moveCratesInBed()
{
    translateSuction(Side::LEFT, 0.0);
    translateSuction(Side::RIGHT, 0.0);

    moveArm(ArmPosition::FOLD_MID);
    robot_->wait(0.5);
    moveArm(ArmPosition::FOLD);
    robot_->wait(0.5);
    moveRails(RailPosition::INTERNAL);
    while (areRailsMoving())
        robot_->wait(0.1);
    pumpOff(Side::RIGHT);
    pumpOff(Side::LEFT);

    moveRails(RailPosition::FORWARD);
    while (areRailsMoving())
        robot_->wait(0.1);
    moveArm(ArmPosition::RAISE);
}

void ServoManager::dropCrates()
{
    if (robot_->getGameState()->isBedFull)
    {
        moveArm(ArmPosition::CALIBRATE);
        bedUnfold();
        translateSuction(Side::LEFT, 1.0);
        translateSuction(Side::RIGHT, 1.0);
        robot_->wait(0.5);
        bedFold();
        robot_->getGameState()->isBedFull = false;
    }

    if (robot_->getGameState()->isClawFull)
    {
        moveArm(ArmPosition::GRAB);
        pumpOff(Side::RIGHT);
        pumpOff(Side::LEFT);
        robot_->wait(0.5);
        moveArm(ArmPosition::RAISE);
        robot_->getGameState()->isClawFull = false;
    }
}