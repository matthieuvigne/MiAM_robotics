/// \file LowPass.h
/// \brief A simple first-order low-pass
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_LOWPASS
#define MIAM_LOWPASS

    namespace miam{
        class LowPass
        {
            public:
                /// \brief Constructor
                /// \param[in] frequency Cutoff frequency (Hz
                LowPass(double const& frequency);

                /// @brief Reset the low-pass state
                /// @param[in] value Reset value
                void reset(double const& value);

                /// @brief Filter input
                /// @param[in] value Input value
                /// @param[in] dt Time since last sample
                /// @return Filtered value
                double filter(double const& value, double const& dt);

            private:
                double omega_;
                double state_;
                bool isInit_{false};
        };
    }
#endif
