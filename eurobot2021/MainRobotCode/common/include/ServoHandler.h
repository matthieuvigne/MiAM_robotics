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
                void shutdownServos(); ///< Turn off all servos.

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
		void initsectionmiddle();




                // Valve control
                void openValve();
                void closeValve();

                void openTube(int tubeNumber); ///< Open suction air tube.
                void closeTube(int tubeNumber); ///< Close suction air tube.
                void moveSuctionUnitary(int tubeNumber, int targetServo);


                void turnOnPump();
                void turnOffPump();

                // Electro magnet and arm control
                void electroMagnetOn();
                void electroMagnetOff();

                // Figurine arm is high enough for transport and dropout
                void figurineArmTransport();
                // Figurine arm touches the top of the figurine
                void figurineArmCatch();
                // Figurine arm is folded inside the robot
                void figurineArmLow();

                // Figurine speed
                void figurineArmSpeedLow();
                void figurineArmSpeedHigh();

                void moveSuction(bool high);

                void moveRail(int velocity);


                void moveMiddle();
                void transportfigurine();
                

                void moveRail(int velocity);

                void foldArms();
                void unfoldArms(bool isPlayingRightSide);
                void raiseArms(bool isPlayingRightSide);
                
                

                void setMaestro(MaestroDriver & maestro);

                bool isPumpOn_;
            private:
                MaestroDriver * maestro_;
        };
 #endif
