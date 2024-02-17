#ifndef PARAMETERS_H
#define PARAMETERS_H

    #include <miam_utils/trajectory/Trajectory.h>
    #include <miam_utils/Types.h>

    #include "common/RobotParameters.h"
    #include "common/RobotInterface.h"
    #include "common/AbstractStrategy.h"
    #include "common/RobotGUI.h"


    #define RPM_TO_RAD_S(VALUE) (VALUE * 2.0f * M_PI / 60.0f)

    #define TABLE_WIDTH_MM 3000.0
    #define TABLE_HEIGHT_MM 2000.0
    #define TABLE_MARGIN_MM 0.0

    #define WHEEL_RADIUS_MM 30.0f
    #define WHEEL_SPACING_MM 31.0f

    // give 20% overhead
    #define MOTOR_RATED_RPM 140.0f //120.0f // 30.0f
    #define MAX_SPEED_RPM (MOTOR_RATED_RPM)
    #define MAX_SPEED_RAD_S (RPM_TO_RAD_S(MAX_SPEED_RPM))
    #define MAX_SPEED_STEP_S (rad_s_to_step_s(MAX_SPEED_RAD_S))

    #define MAX_WHEEL_SPEED_MM_S (MAX_SPEED_RAD_S * WHEEL_RADIUS_MM)
    #define MAX_WHEEL_ACCELERATION_MM_S 200.0f

#endif