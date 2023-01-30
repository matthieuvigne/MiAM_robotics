#include <miam_utils/trajectory/Trajectory.h>
#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>

#include<iostream>
#include<fstream>

#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

#include "mpc_parameterization.hpp"

using namespace miam;

int main() {
    miam::trajectory::TrajectoryConfig c = getMPCTrajectoryConfig();

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
    ocp.subjectTo( -c.maxWheelVelocity / 1000.0 <= v + (c.robotWheelSpacing / 1000.0) * w <= c.maxWheelVelocity / 1000.0   );     // the control input u,
    ocp.subjectTo( -c.maxWheelVelocity / 1000.0 <= v - (c.robotWheelSpacing / 1000.0) * w <= c.maxWheelVelocity / 1000.0   );     // the control input u,
    ocp.subjectTo( -c.maxWheelAcceleration / 1000.0 <= vu + (c.robotWheelSpacing / 1000.0) * wu <= c.maxWheelAcceleration / 1000.0   );     // the control input u,
    ocp.subjectTo( -c.maxWheelAcceleration / 1000.0 <= vu - (c.robotWheelSpacing / 1000.0) * wu <= c.maxWheelAcceleration / 1000.0   );     // the control input u,

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


    if (mpc.exportCode( "../src/generated_code" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;
}