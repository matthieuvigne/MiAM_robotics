/// \file Metronome.h
/// \brief Precise sleep for periodic event.
///
/// \details This file provides a way to execute code in a periodic manner, as precisesly as possible.
/// The idea is the same as g_timeout_add: we want some code to be executed every delta_t time.
/// However, g_timeout_add is no meant for precise timing, and doesn't catchup execution time.
/// Here, we use clock_nanosleep on TIMER_ABSTIME to be as close as possible to real time, while catching up
/// delays.
/// As a benchmark, I tried calling a funciton every 5ms for 100s:
///  - using g_timeout function call was accurate withing 10%, and we ended up doing only 18600 iterations (7% loss).
///  - using this metronome, function call was accurate withing 1%, and exactly 20000 iterations were performed.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef METRONOME
#define METRONOME
    #include <time.h>

    class Metronome{
        public:
            /// \brief Create a metronome, at the current time, with the given period.
            ///
            /// \param[in] period Metronome period, in nanoseconds.
            Metronome(int period);

            /// \brief Wait for one period.
            /// \details This funciton sleeps until the duration nPeriod has elapsed since the last call to metronome_wait or
            /// metronome_create. Possibly this will not sleep at all, if the CPU is running late; it will sleep no more than
            /// no more than nPeriod (granted no modification to targetTime is done by the user).
            void wait();

            /// \brief Get the time elapsed since startTime.
            ///
            /// \return Time since startTime, in seconds.
            double getElapsedTime();

            /// \brief Reset the time target to remove delay catchup.
            void resetLag();

        private:
            struct timespec startTime_; ///< The start time of the metronome (i.e. time when init was called).
            struct timespec targetTime_; ///< The target time to stop sleep: this is equal to startTime + n_iterations * nPeriod
            int nPeriod_; ///< Metronome period, in nanoseconds.
    };
#endif
