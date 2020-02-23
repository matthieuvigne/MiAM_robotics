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
    #include <miam_utils/MiAMEurobot.h>
    
    class ArduinoListener{
        public:
            /// \brief Default constructor.
            ArduinoListener();
            
            /// \brief Initialize object and start background thread on the given port.
            ///
            /// \param[in] kinematics Kinematics of the current robot.
            /// \param[in] portName Name of the port where the arduino is connect.
            ///
            /// \return True if connection with Arduino was successful, false otherwise.
            bool intialize(omni::ThreeWheelsKinematics const& kinematics, std::string const& portName);
            
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
        omni::ThreeWheelsKinematics kinematics_;
        bool isInitialized_;
        
    };
 #endif
