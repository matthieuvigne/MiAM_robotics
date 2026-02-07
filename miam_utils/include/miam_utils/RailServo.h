#ifndef RAIL_SERVO
    #define RAIL_SERVO

    #include "miam_utils/drivers/STSServoDriver.h"

    class STSScheduler;


    /// \brief Control for a continuous rotation servo mounted on a rail, with motion normalized between 0 (bottom) and 1 (top)
    class RailServo
    {
        enum RailState
        {
            INIT,
            CALIBRATING,
            MOVING,
            TARGET_REACHED,
            MOTION_FAILED
        };

        friend STSScheduler;

        public:

            /// @brief Constructor
            /// @param servoId Id of the servo
            /// @param gpioId Id of the limit switch GPIO
            /// @param distance Total travel distance
            /// @param inverted Invert servo rotation
            /// @param calibrateBottom If set, initial calibration is done at the bottom (0) instead of top (1)
            RailServo(STSServoDriver *driver, int const& servoId, int const& gpioId, int const& distance, bool inverted=false, bool calibrateBottom=false);

            void move(double const& targetPosition);

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

            void startCalibration()
            {
                currentState_ = RailState::CALIBRATING;
            }
            bool isCalibrated() const
            {
                return currentState_ != RailState::CALIBRATING && currentState_ != RailState::INIT;
            }

        protected:
            void tick(); // Function called periodically to perform servo control

        private:
            STSServoDriver *driver_;
            int servoId_;
            int gpio_;
            double travelDistance_;
            int sign_;
            bool calibrateBottom_;

            int calibStep_ = -1;

            double currentPosition_ = 0.0;
            double targetPosition_ = 0.0;
            int lastReadPosition_ = 0;

            RailState currentState_ = RailState::TARGET_REACHED;
    };
#endif
