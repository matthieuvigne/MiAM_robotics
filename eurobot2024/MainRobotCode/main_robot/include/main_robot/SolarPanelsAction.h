#ifndef MAIN_ROBOT_SOLAR_PANELS_ACTION_H
#define MAIN_ROBOT_SOLAR_PANELS_ACTION_H

#include <string>
#include "main_robot/AbstractAction.h"
#include "main_robot/ServoManager.h"

// #ifndef SIMULATION
// #include "common/solar_panel_camera.hpp"
// #endif

class SolarPanelsAction: public AbstractAction
{
public:
    /// @brief Action to pick up plants from a given zone.
    /// @param zoneCenter Center of the zone from which to pick-up the plants
    SolarPanelsAction(RobotInterface *robot, ServoManager *servoManager):
        AbstractAction("Solar panels", robot),
        servoManager_(servoManager)
    {}

    /// @brief This function is called before choosing the action in the list,
    ///        giving the opportunity for an action to update its start position and priority.
    void updateStartCondition() override;

    void actionStartTrigger() override;

    bool performAction() override;

private:
    ServoManager *servoManager_;
};


// class SolarPanelsWithCameraAction: public AbstractAction
// {
// public:
//     /// @brief Action to pick up plants from a given zone.
//     /// @param zoneCenter Center of the zone from which to pick-up the plants
//     SolarPanelsWithCameraAction(RobotInterface *robot, ServoManager *servoManager):
//         AbstractAction("Solar panels, camera", robot),
//         servoManager_(servoManager)
// #ifndef SIMULATION
//         ,camera("/dev/video0")
// #endif
//     {}

//     // /// @brief This function is called before choosing the action in the list,
//     // ///        giving the opportunity for an action to update its start position and priority.
//     // void updateStartCondition() override;

//     // void actionStartTrigger() override;

//     // bool performAction() override;

// private:
//     ServoManager *servoManager_;

// #ifndef SIMULATION
//     vision::SolarPanelCamera camera;
// #endif
// };


#endif
