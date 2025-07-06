#ifndef TYPES_H
     #define TYPES_H

    #include <string>
    #include <miam_utils/trajectory/RobotPosition.h>
    #include "miam_utils/trajectory/DrivetrainKinematics.h"
    #include "miam_utils/Types.h"
    #include "miam_utils/RPLidarHandler.h"



    ///< The various states the robot can be in
    enum robotstate{
        INIT                 = 0,
        STRATEGY_SETUP       = 1,
        WAITING_FOR_CABLE    = 2,
        WAITING_FOR_START    = 3,
        MATCH                = 4,
        MATCH_DONE           = 5,
        UNDERVOLTAGE         = 6,
        MATCH_QUIT           = 7
    };
    std::string const robotStateNames[] = {"Initializing", "Strategy setup", "Waiting for cable", "Waiting for start", "Match", "Match done", "Undervoltage", "Match quit"};


    struct RobotGUIData {
        robotstate state = robotstate::INIT;
        std::string debugStatus = "";
        double batteryVoltage = 0;
        int score = 0;
        double currentMatchTime = 0;
        miam::RobotPosition currentPosition;
        std::vector<miam::RobotPosition> detectedObstacles;
        std::string currentActionName = "";
    }; ///< Data forwarded by the robot to the GUI


    typedef struct {
        WheelSpeed motorSpeed; ///<< Target motor speed, in rad/s
    }DrivetrainTarget;

    struct DrivetrainMeasurements{
        WheelSpeed encoderPosition; ///< Encoder position, rad.
        WheelSpeed encoderPositionIncrement; ///< Speed, measured by the encoders
        WheelSpeed motorSpeed; ///<< Measured motor speed, in rad/s
        std::deque<DetectedRobot> lidarDetection; ///< Robots detected by the lidar.
        double matchTime; ///< Time in the match.
        double gyroscope;
    };

    struct RobotMeasurements{
        DrivetrainMeasurements drivetrainMeasurements;
        double batteryVoltage = 0.0;
    };

    typedef std::tuple<miam::RobotPosition, double> Obstacle;

 #endif
