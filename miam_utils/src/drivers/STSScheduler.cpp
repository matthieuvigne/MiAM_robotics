#include <unistd.h>
#include "miam_utils/STSScheduler.h"


STSScheduler::STSScheduler(double const& readTimeout):
    driver_(readTimeout)
{
}

STSScheduler::~STSScheduler()
{
    shutdown();
}

void STSScheduler::shutdown()
{
    askedForShutdown_ = true;
    if (bgThread_.joinable())
        bgThread_.join();
}

bool STSScheduler::init(std::string const& portName, int const& dirPin, int const& baudRate)
{
    bool success = driver_.init(portName, dirPin, baudRate);

    if (success)
    {
        askedForShutdown_ = false;
        bgThread_ = std::thread(&STSScheduler::backgroundThread, this);
    }
    return success;
}

std::shared_ptr<RailServo> STSScheduler::createRail(int const& servoId, int const& gpioId, int const& distance, bool inverted, bool calibrateBottom)
{
    std::shared_ptr<RailServo> rail = std::make_shared<RailServo>(&driver_, servoId, gpioId, distance, inverted, calibrateBottom);
    rails_.push_back(rail);
    return rail;
}


void STSScheduler::startRailCalibration()
{
    for (auto &r : rails_)
        r->startCalibration();
}

bool STSScheduler::areAllRailsCalibrated() const
{
    bool allCalib = true;
    for (auto const& r : rails_)
        allCalib &= r->isCalibrated();
    return allCalib;
}

void STSScheduler::setMode(unsigned char const& servoId, STS::Mode const& mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[servoId].state = State::ENABLING;
    commands_[servoId].mode = mode;
}

int16_t STSScheduler::getCurrentPosition(unsigned char const& servoId)
{
    return driver_.getCurrentPosition(servoId);
}

int16_t STSScheduler::getCurrentVelocity(unsigned char const& servoId)
{
    return driver_.getCurrentVelocity(servoId);
}

bool STSScheduler::isMoving(unsigned char const& servoId)
{
    return driver_.isMoving(servoId);
}

void STSScheduler::setTargetPosition(unsigned char const& servoId, int16_t const& position)
{
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[servoId].value = position;
}

void STSScheduler::setTargetVelocity(unsigned char const& servoId, int16_t const& velocity)
{
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[servoId].value = velocity;
}

void STSScheduler::disable(unsigned char const& servoId)
{
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[servoId].state = State::DISABLING;
}

void STSScheduler::enable(unsigned char const& servoId)
{
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[servoId].state = State::ENABLING;
}


void STSScheduler::backgroundThread()
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
                if (i == 0xFE)
                    continue;
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

        // Now handle the rails
        for (auto& r : rails_)
        {
            r->tick();
            usleep(5);
        }
    }
    driver_.disable(0xFE);
}