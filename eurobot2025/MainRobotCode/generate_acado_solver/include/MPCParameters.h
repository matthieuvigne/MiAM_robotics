#ifndef MPC_PARAMETERS_H
#define MPC_PARAMETERS_H

    #define MPC_DELTA_T 0.1// 100 ms
    #define MPC_N_TIME_INTERVALS 27 // 20 discrete time intervals

    #define MPC_MU_TRAJ 5000 // weight of the trajectory (x, y) in the optimization algorithm
    #define MPC_MU_THETA 10 // weight of the trajectory (theta) in the optimization algorithm
    #define MPC_MU_VLIN 100 // weight of the trajectory (v) in the optimization algorithm
    #define MPC_MU_VANG 1 // weight of the trajectory (w) in the optimization algorithm

    #define MPC_PONDERATION_FINAL_STATE 10 // scaling factor to account more for the final state

    #define REFERENCE_WHEEL_SPACING 104.5 // reference wheel spacing in mm, from wheel to center of robot
    #define MAX_WHEEL_SPEED 450.0 // max wheel velocity in mm/s
    #define MAX_WHEEL_ACCELERATION 600.0 // max wheel acceleration in mm/s


#endif

