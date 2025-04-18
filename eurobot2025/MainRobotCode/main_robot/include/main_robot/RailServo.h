#ifndef RAIL_SERVO_H
#define RAIL_SERVO_H

#include <miam_utils/drivers/STSServoDriver.h>

class RailManager;

enum RailState
{
    MOVING,
    TARGET_REACHED,
    MOTION_FAILED
};


/// \brief Control for a continuous rotation servo mounted on a vertical rail
class RailServo
{
    friend RailManager;

    public:

        /// @brief Constructor
        /// @param servoId Id of the servo
        /// @param gpioId Id of the limit switch GPIO
        /// @param distance Total travel distance
        /// @param inverted Invert servo rotation
        RailServo(int const& servoId, int const& gpioId, int const& distance, bool inverted=false, bool calibrateBottom=false);

        void init(STSServoDriver *driver);

        void move(double const& targetPosition);
        // Abort current motion and move rail up
        void abort();

        double getCurrentPosition() const;

        bool isMoving() const
        {
            return currentState_ == RailState::MOVING;
        }

        bool isTargetReached() const
        {
            return currentState_ == RailState::TARGET_REACHED;
        }

        RailState getCurrentState() const
        {
            return currentState_;
        }

    protected:
        void tick(); // Function called periodically to perform servo control
        void calibration(); // Blocking function performing calibration

    private:
        STSServoDriver *driver_;
        int servoId_;
        int gpio_;
        int travelDistance_;
        int sign_;
        bool calibrateBottom_;

        double currentPosition_ = 0.0;
        double targetPosition_ = 0.0;
        int lastReadPosition_ = 0;

        RailState currentState_ = RailState::TARGET_REACHED;
};

class RailManager
{
public:
    RailManager() = default;
    void start(std::vector<RailServo*> rails);

    bool areCalibrated() const;
    void abort();

private:
    std::vector<RailServo*> rails_;
    bool calibDone_ = false;

    // Thread to control the rails
    void railControlThread();
};

#endif
