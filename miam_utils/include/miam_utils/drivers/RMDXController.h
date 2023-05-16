/// \file RMDXController.h
/// \brief A PI-based velocity controller (with a low-pass) for RMDX motors


#ifndef RMDX_CONTROLLER
#define RMDX_CONTROLLER

#include "miam_utils/drivers/RMDX.h"
#include "miam_utils/LowPass.h"

class RMDXController{

    public:
        /// @brief Init controller
        /// @param driver Driver for communication
        /// @param motorId Motor number
        /// @param Kp Proportional gain (A / rad/s)
        /// @param Ki Integral gain (Hz)
        /// @param maxOutput Maximum output (A)
        /// @param filterCutoff Low-pass filter cutoff frequency (Hz).
        /// @param maxFeedforward Maximum feedfoward current (A)
        /// @param maxAcceleration Maximum wheel acceleration, rad/s (used to clamp target)
        RMDXController(RMDX *driver,
                       unsigned char const& motorId,
                       double const& Kp,
                       double const& Ki,
                       double const& maxOutput,
                       double const& filterCutoff,
                       double const& maxFeedforward,
                       double const& maxAcceleration);

        /// @brief Compute next output and send it to the motor.
        /// @param targetVelocity Target velocity (rad/s)
        /// @param dt Time since last call
        /// @return The target sent
        double sendTarget(double const& targetVelocity, double const& dt);

        /// @brief Stop the motor, locking it in place.
        void stop();

        double position_; ///< Motor position (rad)
        double velocity_; ///< Motor velocity (rad/s)
        double rawVelocity_; ///< Unfiltered velocity (rad/s)
        double targetCurrent_; ///< Motor target current (A)
        double current_; ///< Motor current (A)
        double clampedTargetVelocity_ = 0;
    private:
        RMDX *driver_;
        unsigned char motorId_;
        double Kp_;
        double Ki_;
        double integralValue_;
        double maxOutput_;
        bool isStopped_;
        miam::LowPass lowPass_;
        double maxFeedforward_;
        double maxAcceleration_;
};


#endif
