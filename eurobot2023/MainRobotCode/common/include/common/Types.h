#ifndef TYPES_H
     #define TYPES_H

    #include <string>
    #include <miam_utils/trajectory/RobotPosition.h>

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
    }; ///< Data forwarded by the robot to the GUI

    struct DetectedObstacle {
        miam::RobotPosition position;
        double diameter;
    };

 #endif
