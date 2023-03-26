#ifndef TYPES_H
     #define TYPES_H

    #include <string>

    ///< The various states the robot can be in
    enum robotstate{
        INIT                = 0,
        WAITING_FOR_CABLE   = 1,
        WAITING_FOR_START   = 2,
        MATCH               = 3,
        MATCH_DONE          = 4
    };
    std::string const robotStateNames[] = {"Initializing", "Waiting for cable", "Waiting for start", "Match", "Match done"};


    struct RobotGUIData {
        robotstate state = robotstate::INIT;
        std::string debugStatus = "";
        double batteryVoltage = 0;
        int score = 0;
        double currentMatchTime = 0;
    }; ///< Data forwarded by the robot to the GUI

 #endif
