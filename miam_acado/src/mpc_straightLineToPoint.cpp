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

    targetPosition = RobotPosition(0, 0, 0);
    endPosition = RobotPosition(1000, 0, 0);

    traj = trajectory::computeTrajectoryStraightLineToPoint(c, targetPosition, endPosition);

    std::cout << "EndPoint : " << traj.getEndPoint().position << std::endl;

    // write ref traj to file
    std::ofstream myfile;
    myfile.open ("ref.txt");

        // variables for plots
    std::vector<double> pts_true_theta;
    std::vector<double> pts_true_v;
    std::vector<double> pts_true_w;

    std::vector<double> pts_res_theta;
    std::vector<double> pts_res_v;
    std::vector<double> pts_res_w;

    std::vector<std::pair<double, double> > xy_pts_init;
    std::vector<std::pair<double, double> > xy_pts_true;
    std::vector<std::pair<double, double> > xy_pts_res;

    std::vector<std::pair<double, double> > v_pts_init;
    std::vector<std::pair<double, double> > v_pts_true;
    std::vector<std::pair<double, double> > v_pts_res;

    std::vector<std::pair<double, double> > w_pts_init;
    std::vector<std::pair<double, double> > w_pts_true;
    std::vector<std::pair<double, double> > w_pts_res;

    trajectory::StraightLine* current_traj = static_cast<trajectory::StraightLine*>(traj.back().get());
    std::cout << "Duration : " << current_traj->getDuration() << std::endl;

    /* Initialize the solver. */
    cout << "Initializing solver" << endl;
    acado_initializeSolver();    


    miam::trajectory::TrajectoryPoint tp = current_traj->getCurrentPoint(0.0);

    acadoVariables.x0[0] = tp.position.x / 1000.0;
    acadoVariables.x0[1] = tp.position.y / 1000.0;
    acadoVariables.x0[2] = tp.position.theta;
    acadoVariables.x0[3] = tp.linearVelocity / 1000;
    acadoVariables.x0[4] = tp.angularVelocity;

    // perturbation
    float perturbation_scale = 100.0 / 1000.0;
    float perturbation_scale_angle = 0.2;

    for (int j = 0; j < N+1; j++) {

        double t = j * DELTA_T ; 

        if (t > current_traj->getDuration()) {
            t = current_traj->getDuration();
        }

        tp = current_traj->getCurrentPoint(t);
        myfile << tp.position.x << "\t" << tp.position.y << "\t" << tp.position.theta << "\t" << tp.linearVelocity << "\t" << tp.angularVelocity << std::endl;
        std::cout << "true: " << "t=" << t << " --- " << tp.position.x << "\t" << tp.position.y << "\t" << tp.position.theta << "\t" << tp.linearVelocity << "\t" << tp.angularVelocity << std::endl;
                    
        pts_true_theta.push_back(tp.position.theta);
        pts_true_v.push_back(tp.linearVelocity);
        pts_true_w.push_back(tp.angularVelocity);

        xy_pts_true.push_back(std::make_pair(tp.position.x, tp.position.y));
        v_pts_true.push_back(std::make_pair(t, tp.linearVelocity));
        w_pts_true.push_back(std::make_pair(t, tp.angularVelocity));

        /* Initialize the states. */        
        acadoVariables.x[ j * NX ] = tp.position.x / 1000.0 + perturbation_scale * ((float) rand()/RAND_MAX - 0.5) * 2;
        acadoVariables.x[ j * NX + 1] = tp.position.y / 1000.0  + perturbation_scale * ((float) rand()/RAND_MAX - 0.5) * 2;
        acadoVariables.x[ j * NX + 2] = tp.position.theta + perturbation_scale_angle * ((float) rand()/RAND_MAX - 0.5) * 2;
    
        /* Initialize the controls. */
        acadoVariables.x[ j * NX + 3] = tp.linearVelocity / 1000.0 + perturbation_scale * ((float) rand()/RAND_MAX - 0.5) * 2;
        acadoVariables.x[ j * NX + 4] = tp.angularVelocity + perturbation_scale_angle * ((float) rand()/RAND_MAX - 0.5) * 2;

        std::cout << "pert: " << "t=" << t << " --- " << 
            acadoVariables.x[ j * NX ] * 1000 << "\t" << 
            acadoVariables.x[ j * NX + 1] * 1000 << "\t" << 
            acadoVariables.x[ j * NX + 2] * 1000 << "\t" << 
            acadoVariables.x[ j * NX + 3] * 1000 << "\t" << 
            acadoVariables.x[ j * NX + 4] * 1000 << std::endl;


        xy_pts_init.push_back(std::make_pair(acadoVariables.x[ j * NX ] * 1000, acadoVariables.x[ j * NX + 1] * 1000));
        v_pts_init.push_back(std::make_pair(t, acadoVariables.x[ j * NX + 3] * 1000));
        w_pts_init.push_back(std::make_pair(t, acadoVariables.x[ j * NX + 4]));

        if (j < N) {
            acadoVariables.y[ j * NY ] = tp.position.x / 1000.0;
            acadoVariables.y[ j * NY + 1] = tp.position.y / 1000.0;
            acadoVariables.y[ j * NY + 2] = tp.position.theta;
            acadoVariables.y[ j * NY + 3] = tp.linearVelocity / 1000.0;
            acadoVariables.y[ j * NY + 4] = tp.angularVelocity ;
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

        for (int j = 0; j < N+1; j++) {

            double t = j * DELTA_T;

            if (t > current_traj->getDuration()) {
                t = current_traj->getDuration();
            }

            tp = current_traj->getCurrentPoint(t);
            std::cout << "true: " << "t=" << t << " --- " << 
                tp.position.x << "\t" << 
                tp.position.y << "\t" << 
                tp.position.theta << "\t" << 
                tp.linearVelocity << "\t" << 
                tp.angularVelocity << std::endl;

            std::cout << "resu: " << "t=" << t << " --- " << 
                acadoVariables.x[ j * NX ] * 1000 << "\t" << 
                acadoVariables.x[ j * NX + 1] * 1000 << "\t" << 
                acadoVariables.x[ j * NX + 2] << "\t" << 
                acadoVariables.x[ j * NX + 3] * 1000 << "\t" << 
                acadoVariables.x[ j * NX + 4] << std::endl;

            pts_res_theta.push_back(acadoVariables.x[ j * NX + 2]);
            pts_res_v.push_back(acadoVariables.x[ j * NX + 3] * 1000);
            pts_res_w.push_back(acadoVariables.x[ j * NX + 4]);

            xy_pts_res.push_back(std::make_pair(acadoVariables.x[ j * NX ] * 1000, acadoVariables.x[ j * NX + 1] * 1000));
            v_pts_res.push_back(std::make_pair(t, acadoVariables.x[ j * NX + 3] * 1000));
            w_pts_res.push_back(std::make_pair(t, acadoVariables.x[ j * NX + 4]));
	        
        }
        
        gp << "set term wxt 0\n";
        gp << "set xrange [0:1000]\nset yrange [0:1000]\n";
        gp << "plot" << gp.file1d(xy_pts_true) << "with lines title 'true',"
            << gp.file1d(xy_pts_res) << "with lines title 'res',"
            << gp.file1d(xy_pts_init) << "with lines title 'init'" << std::endl;
        
        gp << "set term wxt 1\n";
        gp << "set xrange [0:10]\nset yrange [0:1000]\n";
        gp << "plot" << gp.file1d(v_pts_true) << "with lines title 'true',"
            << gp.file1d(v_pts_res) << "with lines title 'res'," 
            << gp.file1d(v_pts_init) << "with lines title 'init'" << std::endl;

        gp << "set term wxt 2\n";
        gp << "set xrange [0:10]\nset yrange [-1:1]\n";
        gp << "plot" << gp.file1d(w_pts_true) << "with lines title 'true',"
            << gp.file1d(w_pts_res) << "with lines title 'res'," 
            << gp.file1d(w_pts_init) << "with lines title 'init'" << std::endl;


        acado_printControlVariables();
    }

}