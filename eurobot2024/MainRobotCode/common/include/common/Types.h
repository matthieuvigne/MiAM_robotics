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
        UNDERVOLTAGE         = 6
    };
    std::string const robotStateNames[] = {"Initializing", "Strategy setup", "Waiting for cable", "Waiting for start", "Match", "Match done", "Undervoltage"};


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
        Vector2 motorSpeed = Vector2::Zero(); ///<< Target motor speed, in rad/s
    }DrivetrainTarget;

    struct DrivetrainMeasurements{
        Vector2 encoderPosition = Vector2::Zero(); ///< Encoder position, rad.
        WheelSpeed encoderSpeed; ///< Speed, measured by the encoders
        Vector2 motorSpeed = Vector2::Zero(); ///<< Measured motor speed, in rad/s
        std::deque<DetectedRobot> lidarDetection; ///< Robots detected by the lidar.
    };

    struct RobotMeasurements{
        DrivetrainMeasurements drivetrainMeasurements;
        double batteryVoltage = 0.0;
    };

 #endif
