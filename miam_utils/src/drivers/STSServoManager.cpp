#include <unistd.h>
#include "miam_utils/STSServoManager.h"


STSServoManager::STSServoManager(double const& readTimeout):
    driver_(readTimeout)
{
}

STSServoManager::~STSServoManager()
{
    shutdown();
}

void STSServoManager::shutdown()
{
    askedForShutdown_ = true;
    if (bgThread_.joinable())
        bgThread_.join();
}

bool STSServoManager::init(std::string const& portName, int const& dirPin, int const& baudRate)
{
    bool success = driver_.init(portName, dirPin, baudRate);

    if (success)
    {
        askedForShutdown_ = false;
        bgThread_ = std::thread(&STSServoManager::backgroundThread, this);
    }
    return success;
}

void STSServoManager::setMode(unsigned char const& servoId, STS::Mode const& mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[servoId].state = State::ENABLING;
    commands_[servoId].mode = mode;
}

int16_t STSServoManager::getCurrentPosition(unsigned char const& servoId)
{
    return driver_.getCurrentPosition(servoId);
}

int16_t STSServoManager::getCurrentVelocity(unsigned char const& servoId)
{
    return driver_.getCurrentVelocity(servoId);
}

bool STSServoManager::isMoving(unsigned char const& servoId)
{
    return driver_.isMoving(servoId);
}

void STSServoManager::setTargetPosition(unsigned char const& servoId, int16_t const& position)
{
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[servoId].value = position;
}

void STSServoManager::setTargetVelocity(unsigned char const& servoId, int16_t const& velocity)
{
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[servoId].value = velocity;
}

void STSServoManager::disable(unsigned char const& servoId)
{
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[servoId].state = State::DISABLING;
}

void STSServoManager::enable(unsigned char const& servoId)
{
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[servoId].state = State::ENABLING;
}


void STSServoManager::backgroundThread()
{
    pthread_setname_np(pthread_self(), "STSServoMangaer");

    while (!askedForShutdown_)
    {
        ServoCommand command;

        unsigned char i = 0;
        while (i < 0xFE)
        {
            // Look for next servo
            {
                std::lock_guard<std::mutex> lock(mutex_);
                while (i < 0xFE && commands_[i].state == State::UNMANAGED)
                {
                    i++;
                }
                command = commands_[i];

                // Update state machine - do this in the lock to avoid race conditions
                if (command.state == State::ENABLING)
                {
                    commands_[i].state = State::ON;
                    commands_[i].resetCounter_ = 10;
                }
                else if (command.state == State::DISABLING)
                    commands_[i].state = State::UNMANAGED;


                // Decrement reset counter and resent enable if needed
                if (commands_[i].state == State::ON)
                {
                    commands_[i].resetCounter_--;
                    if (commands_[i].resetCounter_ == 0)
                    {
                        commands_[i].resetCounter_ = 10;
                    }
                }

            }
            // Process command
            if (command.state == State::ENABLING)
            {
                if (command.mode == STS::Mode::POSITION)
                    driver_.setTargetPosition(i, driver_.getCurrentPosition(i));
                driver_.setMode(i, command.mode);
            }
            else if (command.state == State::DISABLING)
            {
                driver_.disable(i);
            }
            else if (command.state == State::ON)
            {
                // Resend mode to reneable motor in case it got powered off
                if (command.resetCounter_ == 1)
                {
                    driver_.setMode(i, command.mode);
                }
                if (command.mode == STS::Mode::VELOCITY)
                    driver_.setTargetVelocity(i, command.value);
                else if (command.mode == STS::Mode::POSITION)
                {
                    driver_.setTargetPosition(i, command.value);
                }
            }
            usleep(10);
            i++;
        }
    }
    driver_.disable(0xFE);
}