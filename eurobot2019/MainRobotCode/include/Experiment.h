/// \file Experiment.h
/// \brief Bluetooth communciation with experiment.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef EXPERIMENT_H
     #define EXPERIMENT_H

    #include <mutex>
    #include <thread>

    /// \brief Class to communicate via bluetooth the the experiment.
    /// \details This class initializes the connection, checks that it is alive and that the arduino is responding,
    ///          and sends a start signal to the experiment.
    ///          All this is done in a background thread so as not to block the user.
    class Experiment
    {
        public:
            /// \param Constructor. This also starts a background thread to communicate with the object.
            Experiment();

            /// \brief Try to connect to the experiment. If successful, the background thread is started.
            ///
            /// \return True on success, false otherwise.
            bool startConnection();

            /// \brief Check the connection with the experiment.
            /// \return True if experiment is connected, false otherwise.
            bool isConnected();

            /// \brief Try to send a start message to the experiment.
            /// \details The background thread then tries to send the start message for as long as needed and then
            ///          terminates.
            void start();

            /// \brief True if the experiment has started (acknowledge recieved).
            bool hasStarted();

        private:

            void loop();    ///< Background thread that does the actual communication.

            /// \brief Check connection with the Arduino.
            ///
            /// \return True if connection is successful.
            bool checkConnection();

            /// \brief Send start signal to Arduino.
            ///
            /// \return True if acknowledge was recieved.
            bool sendStart();

            std::mutex mutex_; ///< Mutex variable.
            int port_;    ///< The port file descriptor.

            bool isConnected_;    ///< True if connection was established.
            bool isStarted_;    ///< True if start was sent and acknoledged by the Arduino.
            bool shouldStart_;    ///< True if a start should be sent to the Arduino.
    };
 #endif
