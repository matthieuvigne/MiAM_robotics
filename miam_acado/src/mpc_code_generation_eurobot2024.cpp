#include <iostream>
#include <fstream>

#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

#define MPC_DELTA_T 0.1// 100 ms
#define MPC_N_TIME_INTERVALS 20 // 20 discrete time intervals

#define MPC_MU_TRAJ 1000 // weight of the trajectory (x, y) in the optimization algorithm
#define MPC_MU_THETA 10 // weight of the trajectory (theta) in the optimization algorithm
#define MPC_MU_VLIN 100 // weight of the trajectory (v) in the optimization algorithm
#define MPC_MU_VANG 1 // weight of the trajectory (w) in the optimization algorithm

#define MPC_PONDERATION_FINAL_STATE 10 // scaling factor to account more for the final state

#define REFERENCE_WHEEL_SPACING 100.5 // reference wheel spacing in mm, from wheel to center of robot
#define MAX_WHEEL_SPEED 1000 // max wheel velocity in mm/s
#define MAX_WHEEL_ACCELERATION 1500 // max wheel acceleration in mm/s


int main() {

    USING_NAMESPACE_ACADO
    
    // Number of time intervals
    int N = MPC_N_TIME_INTERVALS;
    
    // Duration of the timestep
    // 10 ms
    double dt = MPC_DELTA_T;
    
    // Final time
    double T = N * dt;

    DifferentialState        x, y, theta, v, w    ;
    Control                  vu, wu     ;   
    DifferentialEquation     f( 0.0, T );

    f << dot(x) == v * cos(theta) ;
    f << dot(y) == v * sin(theta) ;
    f << dot(theta) == w;
    f << dot(v) == vu;
    f << dot(w) == wu;
    
    Function h, hN;
    h << x << y << theta << v << w ;
    hN << x << y << theta << v << w;

    DMatrix W ( h.getDim(), h.getDim() );
    W(0, 0) = MPC_MU_TRAJ;
    W(1, 1) = MPC_MU_TRAJ;
    W(2, 2) = MPC_MU_THETA;
    W(3, 3) = MPC_MU_VLIN;
    W(4, 4) = MPC_MU_VANG;
    
    DMatrix WN ( hN.getDim(), hN.getDim() );
    WN(0, 0) = MPC_PONDERATION_FINAL_STATE * MPC_MU_TRAJ;
    WN(1, 1) = MPC_PONDERATION_FINAL_STATE * MPC_MU_TRAJ;
    WN(2, 2) = MPC_PONDERATION_FINAL_STATE * MPC_MU_THETA;
    WN(3, 3) = MPC_PONDERATION_FINAL_STATE * MPC_MU_VLIN;
    WN(4, 4) = MPC_PONDERATION_FINAL_STATE * MPC_MU_VANG;

    //
    // Optimal Control Problem
    //
    OCP ocp(0.0, T, N);

    ocp.subjectTo( f );

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);
    
    
    // Relaxing some constraints
    ocp.subjectTo( -MAX_WHEEL_SPEED / 1000.0 <= v + (REFERENCE_WHEEL_SPACING / 1000.0) * w <= MAX_WHEEL_SPEED / 1000.0   );     // the control input u,
    ocp.subjectTo( -MAX_WHEEL_SPEED / 1000.0 <= v - (REFERENCE_WHEEL_SPACING / 1000.0) * w <= MAX_WHEEL_SPEED / 1000.0   );     // the control input u,
    ocp.subjectTo( -MAX_WHEEL_ACCELERATION / 1000.0 <= vu + (REFERENCE_WHEEL_SPACING / 1000.0) * wu <= MAX_WHEEL_ACCELERATION / 1000.0   );     // the control input u,
    ocp.subjectTo( -MAX_WHEEL_ACCELERATION / 1000.0 <= vu - (REFERENCE_WHEEL_SPACING / 1000.0) * wu <= MAX_WHEEL_ACCELERATION / 1000.0   );     // the control input u,

    // Export the code:
    OCPexport mpc( ocp );

    mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
    mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );
    mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
    // mpc.set( INTEGRATOR_TYPE,             INT_EX_EULER         );
    mpc.set( NUM_INTEGRATOR_STEPS,        N              );
    // mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS,        2              );
    mpc.set( USE_SINGLE_PRECISION,        BT_TRUE              );

    mpc.set( QP_SOLVER,                   QP_QPOASES      );
    mpc.set( GENERATE_TEST_FILE,          NO             );
    mpc.set( GENERATE_MAKE_FILE,          NO             );
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO             );
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO             );
    mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO             );


    // if (mpc.exportCode( "../src/generated_code" ) != SUCCESSFUL_RETURN)
    //     exit( EXIT_FAILURE );

    if (mpc.exportCode( "../../src/MiAM_robotics/eurobot2024/MainRobotCode/common/acado_solver" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;
}