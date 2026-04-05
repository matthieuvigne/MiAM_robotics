#include <unistd.h>
#include <iostream>
#include <chrono>
#include "miam_utils/STSScheduler.h"


STSScheduler::STSScheduler(bool const disableServos, double const& readTimeout):
    driver_(readTimeout),
    keepServosDisabled_(disableServos)
{
}

STSScheduler::~STSScheduler()
{
    shutdown();
}


void STSScheduler::setLogger(Logger *logger)
{
    logger_ = logger;
    for (int i = 0; i < 256; i++)
        missingServoWarningSent_[i] = false;
}

void STSScheduler::shutdown()
{
    askedForShutdown_ = true;
    if (bgThread_.joinable())
        bgThread_.join();
}

bool STSScheduler::init(std::string const& portName, int const& dirPin, int const& baudRate)
{
    if (keepServosDisabled_)
        return true;

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
    std::shared_ptr<RailServo> rail = std::make_shared<RailServo>(&driver_, servoId, gpioId, distance, inverted, calibrateBottom, keepServosDisabled_);
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
    if (keepServosDisabled_)
        return true;
    bool allCalib = true;
    for (auto const& r : rails_)
        allCalib &= r->isCalibrated();
    return allCalib;
}

void STSScheduler::setPIDGains(unsigned char const& servoId, unsigned char const& Kp, unsigned char const& Kd, unsigned char const& Ki)
{
    if (!keepServosDisabled_)
        driver_.setPIDGains(servoId, Kp, Kd, Ki);
}

void STSScheduler::setMode(unsigned char const& servoId, STS::Mode const& mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[servoId].state = State::ENABLING;
    commands_[servoId].mode = mode;
    commands_[servoId].isInit_ = true;
}

int16_t STSScheduler::getCurrentPosition(unsigned char const& servoId)
{
    if (keepServosDisabled_)
        return 0;
    return driver_.getCurrentPosition(servoId);
}

int16_t STSScheduler::getCurrentVelocity(unsigned char const& servoId)
{
    if (keepServosDisabled_)
        return 0;
    return driver_.getCurrentVelocity(servoId);
}

bool STSScheduler::isMoving(unsigned char const& servoId)
{
    if (keepServosDisabled_)
        return false;
    return driver_.isMoving(servoId);
}

void STSScheduler::setTargetPosition(unsigned char const& servoId, int16_t const& position)
{
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[servoId].value = position;
    commands_[servoId].isInit_ = false;
}

void STSScheduler::setMaxVelocity(unsigned char const& servoId, int16_t const& maxVelocity)
{
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[servoId].maxVelocity = maxVelocity;
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
        auto const startTime = std::chrono::steady_clock::now();
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
                {
                    bool success = false;
                    int16_t pos = driver_.getCurrentPosition(i, success);
                    if (success)
                    {
                        driver_.setTargetPosition(i, pos);
                        driver_.setMode(i, command.mode);
                        command.value = pos;
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            if (commands_[i].isInit_)
                            {
                                commands_[i].isInit_ = false;
                                commands_[i].value = pos;
                            }
                        }
                    }
                    else
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        // Retry enabling
                        commands_[i].state = State::ENABLING;
                        // Warn user
                        if (logger_ != nullptr)
                        {
                            if (!missingServoWarningSent_[i])
                            {
                                missingServoWarningSent_[i] = true;
                                *logger_ << "[STSScheduler] Warning: servo " << static_cast<int>(i) << " failed to respond." << std::endl;
                            }
                        }
                    }
                }
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
                    if (command.maxVelocity > 0)
                    {
                        driver_.setTargetVelocity(i, command.maxVelocity);
                    }
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
        double const dt = (std::chrono::steady_clock::now() - startTime).count();
        // logger_->log("STSSchedulerElapsedTime", 0.0, dt);
    }
    driver_.disable(0xFE);
}