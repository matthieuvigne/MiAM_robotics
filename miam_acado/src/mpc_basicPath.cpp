#include <miam_utils/trajectory/Trajectory.h>
#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>

#include<iostream>
#include<fstream>

#include "generated_code/acado_common.h"
#include "generated_code/acado_auxiliary_functions.h"

#include "mpc_parameterization.hpp"
#include "gnuplot-iostream.h"

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/*
 * Parameters with which the custom solver was compiled
 */
#define DELTA_T    MPC_DELTA_T
#define HORIZON_T   DELTA_T * N

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

using namespace miam;
using namespace std;

int main() {
    RobotPosition targetPosition;
    trajectory::TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;

    miam::trajectory::TrajectoryConfig c = getMPCTrajectoryConfig();
    miam::trajectory::TrajectoryConfig cplan = getMPCTrajectoryConfig();
    cplan.maxWheelVelocity *= 0.8; // give 20% overhead to the controller

    positions.push_back(RobotPosition(0, 0, 0));
    positions.push_back(RobotPosition(200, 200, 0));
    positions.push_back(RobotPosition(200, 400, 0));
    positions.push_back(RobotPosition(500, 500, 0));
    positions.push_back(RobotPosition(750, 500, 0));
    positions.push_back(RobotPosition(200, 900, 0));
    traj = computeTrajectoryBasicPath(cplan, positions);

    std::cout << "EndPoint : " << traj.getEndPoint().position << std::endl;

    // write ref traj to file
    std::ofstream myfile;
    myfile.open ("ref.txt");

        // variables for plots
    std::vector<std::pair<real_t, real_t> > xy_pts_init;
    std::vector<std::pair<real_t, real_t> > xy_pts_true;
    std::vector<std::pair<real_t, real_t> > xy_pts_res;

    std::vector<std::pair<real_t, real_t> > theta_pts_init;
    std::vector<std::pair<real_t, real_t> > theta_pts_true;
    std::vector<std::pair<real_t, real_t> > theta_pts_res;

    std::vector<std::pair<real_t, real_t> > v_pts_init;
    std::vector<std::pair<real_t, real_t> > v_pts_true;
    std::vector<std::pair<real_t, real_t> > v_pts_res;

    std::vector<std::pair<real_t, real_t> > w_pts_init;
    std::vector<std::pair<real_t, real_t> > w_pts_true;
    std::vector<std::pair<real_t, real_t> > w_pts_res;

    std::cout << "Duration : " << getDurationBasicPath(traj) << std::endl;

    /* Initialize the solver. */
    cout << "Initializing solver" << endl;
    acado_initializeSolver();    


    cout << "N: " << N << endl;
    cout << "ACADO_N: " << ACADO_N << endl;
    cout << "MPC_N_TIME_INTERVALS: " << MPC_N_TIME_INTERVALS << endl;

    miam::trajectory::TrajectoryPoint tp = getCurrentPointBasicPath(traj, 0.0);

    acadoVariables.x0[0] = tp.position.x / 1000.0;
    acadoVariables.x0[1] = tp.position.y / 1000.0;
    acadoVariables.x0[2] = tp.position.theta;
    acadoVariables.x0[3] = tp.linearVelocity / 1000;
    acadoVariables.x0[4] = tp.angularVelocity;

    // perturbation
    float perturbation_scale = 0.0 / 1000.0;
    float perturbation_scale_angle = 0.0;

    int indice = 0;

    for (int indice = 0; indice < N+1; indice++) {

        cout << indice << endl;

        real_t t = indice * DELTA_T ; 

        if (t > getDurationBasicPath(traj)) {
            t = getDurationBasicPath(traj);
        }

        tp = getCurrentPointBasicPath(traj, t);
        myfile << tp.position.x << "\t" << tp.position.y << "\t" << tp.position.theta << "\t" << tp.linearVelocity << "\t" << tp.angularVelocity << std::endl;
        std::cout << "true: " << "t=" << t << " --- " << tp.position.x << "\t" << tp.position.y << "\t" << tp.position.theta << "\t" << tp.linearVelocity << "\t" << tp.angularVelocity << std::endl;
                    
        theta_pts_init.push_back(std::make_pair(t, tp.position.theta));

        xy_pts_true.push_back(std::make_pair(tp.position.x, tp.position.y));
        v_pts_true.push_back(std::make_pair(t, tp.linearVelocity));
        w_pts_true.push_back(std::make_pair(t, tp.angularVelocity));

        /* Initialize the states. */        
        acadoVariables.x[ indice * NX ] = tp.position.x / 1000.0 + perturbation_scale * ((float) rand()/RAND_MAX - 0.5) * 2;
        acadoVariables.x[ indice * NX + 1] = tp.position.y / 1000.0  + perturbation_scale * ((float) rand()/RAND_MAX - 0.5) * 2;
        acadoVariables.x[ indice * NX + 2] = tp.position.theta + perturbation_scale_angle * ((float) rand()/RAND_MAX - 0.5) * 2;
    
        /* Initialize the controls. */
        acadoVariables.x[ indice * NX + 3] = tp.linearVelocity / 1000.0 + perturbation_scale * ((float) rand()/RAND_MAX - 0.5) * 2;
        acadoVariables.x[ indice * NX + 4] = tp.angularVelocity + perturbation_scale_angle * ((float) rand()/RAND_MAX - 0.5) * 2;

        std::cout << "pert: " << "t=" << t << " --- " << 
            acadoVariables.x[ indice * NX ] * 1000 << "\t" << 
            acadoVariables.x[ indice * NX + 1] * 1000 << "\t" << 
            acadoVariables.x[ indice * NX + 2] << "\t" << 
            acadoVariables.x[ indice * NX + 3] * 1000 << "\t" << 
            acadoVariables.x[ indice * NX + 4] << std::endl;


        xy_pts_init.push_back(std::make_pair(acadoVariables.x[ indice * NX ] * 1000, acadoVariables.x[ indice * NX + 1] * 1000));
        theta_pts_init.push_back(std::make_pair(t, acadoVariables.x[ indice * NX + 2]));
        v_pts_init.push_back(std::make_pair(t, acadoVariables.x[ indice * NX + 3] * 1000));
        w_pts_init.push_back(std::make_pair(t, acadoVariables.x[ indice * NX + 4]));

        if (indice < N) {
            acadoVariables.y[ indice * NY ] = tp.position.x / 1000.0;
            acadoVariables.y[ indice * NY + 1] = tp.position.y / 1000.0;
            acadoVariables.y[ indice * NY + 2] = tp.position.theta;
            acadoVariables.y[ indice * NY + 3] = tp.linearVelocity / 1000.0;
            acadoVariables.y[ indice * NY + 4] = tp.angularVelocity ;
        }
    }

    acadoVariables.yN[0] = tp.position.x / 1000.0;
    acadoVariables.yN[1] = tp.position.y / 1000.0;
    acadoVariables.yN[2] = tp.position.theta;
    acadoVariables.yN[3] = tp.linearVelocity / 1000.0;
    acadoVariables.yN[4] = tp.angularVelocity;

    /* Some temporary variables. */
    acado_timer t;

    /* Get the time before start of the loop. */
    acado_tic( &t );

    /* Prepare step */
    acado_preparationStep();

    /* Perform the feedback step. */
    acado_feedbackStep( );

    /* Apply the new control immediately to the process, first NU components. */

    if( VERBOSE ) printf("\t KKT Tolerance = %.3e\n", acado_getKKT() );

    /* Read the elapsed time. */
    real_t te = acado_toc( &t );

    if( VERBOSE ) printf("Average time of one real-time iteration:   %.3g microseconds\n", 1e6 * te);

    if (VERBOSE )
    {

        Gnuplot gp;

        // acado_printDifferentialVarNUiables();
        // acado_printControlVariables();

        for (int indice = 0; indice < N+1; indice++) {

            real_t t = indice * DELTA_T;

            if (t > getDurationBasicPath(traj)) {
                t = getDurationBasicPath(traj);
            }

            tp = getCurrentPointBasicPath(traj, t);
            std::cout << "true: " << "t=" << t << " --- " << 
                tp.position.x << "\t" << 
                tp.position.y << "\t" << 
                tp.position.theta << "\t" << 
                tp.linearVelocity << "\t" << 
                tp.angularVelocity << std::endl;

            std::cout << "resu: " << "t=" << t << " --- " << 
                acadoVariables.x[ indice * NX ] * 1000 << "\t" << 
                acadoVariables.x[ indice * NX + 1] * 1000 << "\t" << 
                acadoVariables.x[ indice * NX + 2] << "\t" << 
                acadoVariables.x[ indice * NX + 3] * 1000 << "\t" << 
                acadoVariables.x[ indice * NX + 4] << std::endl;


            xy_pts_res.push_back(std::make_pair(acadoVariables.x[ indice * NX ] * 1000, acadoVariables.x[ indice * NX + 1] * 1000));
            theta_pts_res.push_back(std::make_pair(t, acadoVariables.x[ indice * NX + 2]));
            v_pts_res.push_back(std::make_pair(t, acadoVariables.x[ indice * NX + 3] * 1000));
            w_pts_res.push_back(std::make_pair(t, acadoVariables.x[ indice * NX + 4]));
	        
        }
        
        gp << "set term wxt 0\n";
        gp << "set xrange [0:1000]\nset yrange [0:1000]\n";
        gp << "plot" << gp.file1d(xy_pts_true) << "with lines title 'true',"
            << gp.file1d(xy_pts_res) << "with lines title 'res',"
            << gp.file1d(xy_pts_init) << "with lines title 'init'" << std::endl;
        
        gp << "set term wxt 1\n";
        gp << "set xrange [0:10]\nset yrange [-4:4]\n";
        gp << "plot" << gp.file1d(theta_pts_true) << "with lines title 'true',"
            << gp.file1d(theta_pts_res) << "with lines title 'res'," 
            << gp.file1d(theta_pts_init) << "with lines title 'init'" << std::endl;

        gp << "set term wxt 2\n";
        gp << "set xrange [0:10]\nset yrange [0:1000]\n";
        gp << "plot" << gp.file1d(v_pts_true) << "with lines title 'true',"
            << gp.file1d(v_pts_res) << "with lines title 'res'," 
            << gp.file1d(v_pts_init) << "with lines title 'init'" << std::endl;

        gp << "set term wxt 3\n";
        gp << "set xrange [0:10]\nset yrange [-4:4]\n";
        gp << "plot" << gp.file1d(w_pts_true) << "with lines title 'true',"
            << gp.file1d(w_pts_res) << "with lines title 'res'," 
            << gp.file1d(w_pts_init) << "with lines title 'init'" << std::endl;


        acado_printControlVariables();
    }

}