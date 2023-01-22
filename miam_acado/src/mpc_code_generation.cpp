#include <miam_utils/trajectory/Trajectory.h>
#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>

#include<iostream>
#include<fstream>

#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

double mu_traj = 100;
double mu_theta = 0.5;
double mu_vlin = 0.01;
double mu_vang = 0.01;

double ponderation_final_state = 10;

using namespace miam;

int main() {
    miam::trajectory::TrajectoryConfig c;
    c.maxWheelVelocity = 500;
    c.maxWheelAcceleration = 600;
    c.robotWheelSpacing = 100.5;

    USING_NAMESPACE_ACADO
    
    // Number of time intervals
    int N = 100;
    
    // Duration of the timestep
    // 10 ms
    double dt = 0.01;
    
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
    W(0, 0) = mu_traj;
    W(1, 1) = mu_traj;
    W(2, 2) = mu_theta;
    W(3, 3) = mu_vlin;
    W(4, 4) = mu_vang;
    
    DMatrix WN ( hN.getDim(), hN.getDim() );
    WN(0, 0) = ponderation_final_state * mu_traj;
    WN(1, 1) = ponderation_final_state * mu_traj;
    WN(2, 2) = ponderation_final_state * mu_theta;
    WN(3, 3) = ponderation_final_state * mu_vlin;
    WN(4, 4) = ponderation_final_state * mu_vang;

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
    mpc.set( NUM_INTEGRATOR_STEPS,        N              );

    mpc.set( QP_SOLVER,                   QP_QPOASES      );
    mpc.set( GENERATE_TEST_FILE,          NO             );
    mpc.set( GENERATE_MAKE_FILE,          NO             );
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO             );
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO             );

//     mpc.set( USE_SINGLE_PRECISION,        YES             );

    if (mpc.exportCode( "../src/generated_code" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;
}