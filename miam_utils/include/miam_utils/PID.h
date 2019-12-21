/// \file PID.h
/// \brief Implementation of a PID controller
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_PID
#define MIAM_PID

    namespace miam{
        class PID
        {
            public:
                /// \brief Default constructor.
                PID();

                /// \brief Constructor.
                /// \param[in] Kp Proportional gain.
                /// \param[in] Kd Derivative gain.
                /// \param[in] Ki Integral gain.
                /// \param[in] maxIntegral Maximum value of Ki * integral_
                PID(double const& Kp, double const& Kd, double const& Ki, double const& maxIntegral);

                /// \brief Compute PID output.
                /// \param[in] error Value of current error (current - target).
                /// \param[in] dt Time since last call, used to compute integral and derivative.
                double computeValue(double const& error, double const& dt);

                /// \brief Compute PID output.
                /// \param[in] error Value of current error (current - target).
                /// \param[in] errorDerivative Derivative of the error term.
                /// \param[in] dt Time since last call, used to compute integral and derivative.
                double computeValue(double const& error, double const& errorDerivative, double const& dt);

                /// \brief Reset the integral to a specific value (default 0)
                void resetIntegral(double const& value = 0.0);

                double getCorrection();

            private:
                double Kp_; ///< Proportional gain.
                double Kd_; ///< Derivative gain.
                double Ki_; ///< Integral gain.
                double maxIntegral_; ///< Maximum integral value.
                double integral_; ///< Current integral value.
                double previousError_; ///< Previous error, used to compute derivative.
                double lastCorrection_; ///< Last correction value computed.
        };
    }
#endif
