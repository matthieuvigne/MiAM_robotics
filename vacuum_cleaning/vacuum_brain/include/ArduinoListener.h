/// \file ArduinoListener.h
/// \brief Communication between the rapsberry and the arduino.
///
/// \details This class starts a background thread that monitors the status of the Arduino, sending targets and
///          receiving current information.
///
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef ARDUINOSLAVE_H
     #define ARDUINOSLAVE_H

    ///< Global includes
    #include <miam_utils/miam_utils.h>
    
    class ArduinoListener{
        public:
            /// \brief Constructor.
            ///
            /// \param[in] kinematics Kinematics of the current robot.
            /// \param[in] encoderResolution Resolution of encoders, for conversion from SI to encoder ticks.
            ArduinoListener(omni::ThreeWheelsKinematics const& kinematics, int const& encoderResolution);
            
            /// \brief Initialize communication with Arduino and start background thread on success.
            ///
            /// \param[in] portName Name of the port where the arduino is connect.
            ///
            /// \return True if connection with Arduino was successful, false otherwise.
            bool initialize( std::string const& portName);
            
            /// \brief Set target speed of motors.
            ///
            /// \param[in] targetBaseSpeed Target base speed (linear and angular, SI units).
            void setTarget(omni::BaseSpeed const& targetBaseSpeed);
            
            /// \brief Get current speed of the base.
            ///
            /// \return Current base speed (linear and angular, SI units).
            omni::BaseSpeed getCurrentSpeed();
            
            /// \brief Get kinematics of the robot.
            /// \return Pointer to the kinematics object.
            omni::ThreeWheelsKinematics *getKinematics();
    
      private:
        void communicationThread();  ///< Thread handling communication with Arduino.
        
        int port_; ///< Port on which the Arduino is connected.
        omni::ThreeWheelsKinematics kinematics_; ///< Kinematics of the robot.
        omni::WheelSpeed targetWheelSpeed_; ///< Target speed.
        omni::WheelSpeed currentWheelSpeed_; ///< Current angular
        double SI_TO_TICKS_; ///< Convertion from SI units (rad/s) to ticks/s
        double lastWriteTime_; ///< Sime since last write to Arduino, for controlling write frequency.
        
        std::mutex mutex_; ///< Mutex, for thread safety.
        
    };
 #endif
