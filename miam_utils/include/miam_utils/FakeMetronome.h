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
#ifndef FAKE_METRONOME
#define FAKE_METRONOME
    #include <condition_variable>
    #include <miam_utils/Metronome.h>

    class FakeMetronome: public Metronome{
        public:
            /// \brief A fake metronome, to work with the simulation time.
            FakeMetronome(int period);

            void tick();

            void reset();

            /// \brief Wait for one period.
            /// \details This funciton sleeps until the duration nPeriod has elapsed since the last call to metronome_wait or
            /// metronome_create. Possibly this will not sleep at all, if the CPU is running late; it will sleep no more than
            /// no more than nPeriod (granted no modification to targetTime is done by the user).
            void wait() override;

            /// \brief Get the time elapsed since startTime.
            ///
            /// \return Time since startTime, in seconds.
            double getElapsedTime() override;

            /// \brief Reset the time target to remove delay catchup.
            void resetLag() override;

            bool hasReset_;
        private:
            double currentTime_;
            std::mutex mutex_;
            std::condition_variable cv_;
            bool hasTickBeenSent_{false};
    };
#endif
