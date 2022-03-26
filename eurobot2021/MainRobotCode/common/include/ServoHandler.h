/// \file ServoConfig.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef SERVO_CONFIG_H
    #define SERVO_CONFIG_H
        #include <miam_utils/drivers/MaestroServoDriver.h>

        /// \brief Helper class for controlling the robot servos.
        class ServoHandler
        {
            public:
                /// \brief Constructor
                ServoHandler(MaestroDriver *maestro);

                /// \brief Initialize communication with the Maestro servo driver.
                ///
                /// \param portName Serial port file name ("/dev/ttyOx")
                /// \returns   true on success, false otherwise.
                bool init(std::string const& portName);

                void ouvrirlebrasmilieu(bool isPlayingRightSide, bool right);

                void ouvrirlebrasbas(bool isPlayingRightSide, bool right);

                void ouvrirlebrashaut(bool isPlayingRightSide, bool right);
                
                void ouvrirledoigthaut(bool isPlayingRightSide, bool right);
                
                void ouvrirledoigtbas(bool isPlayingRightSide, bool right);
                
		void reglageouvrirlebrasdroithaut();
		void reglageouvrirlebrasdroitmilieu();
		void reglageouvrirlebrasdroitbas();

		void reglageouvrirlebrasgauchehaut() ;
		void reglageouvrirlebrasgauchemilieu() ;
		void reglageouvrirlebrasgauchebas() ;
		
		void reglageouvrirledoigtgauchehaut() ;
		void reglageouvrirledoigtgauchebas() ;
		
		void reglageouvrirledoigtdroithaut() ;
		void reglageouvrirledoigtdroitbas() ;


                

                // Valve control
                void openValve();
                void closeValve();

                void openTube(int tubeNumber); ///< Open suction air tube.
                void closeTube(int tubeNumber); ///< Close suction air tube. 
                void moveSuctionUnitary(int tubeNumber, int targetServo);

                void shutdownServos(); ///< Turn off all servos.

                void turnOnPump();
                void turnOffPump();

                // Electro magnet and arm control
                void electroMagnetOn();
                void electroMagnetOff();

                void figurineArmLow();
                void figurineArmMiddle();
                void figurineArmHigh();

                // Figurine speed
                void figurineArmSpeedLow();
                void figurineArmSpeedHigh();

                void moveSuction(bool high, bool moveMiddle = true);
                void moveMiddle();
                void moveMiddleSuctionForDrop(bool drop = false);

                void moveRail(int velocity);

                void foldArms();
                void unfoldArms(bool isPlayingRightSide);
                void raiseArms(bool isPlayingRightSide);
                void moveArmForDrop(bool isPlayingRightSide);
                void moveSuctionForGoldDrop();

                void setMaestro(MaestroDriver & maestro);
                bool isPumpOn_;
            private:
                MaestroDriver * maestro_;
        };
 #endif
