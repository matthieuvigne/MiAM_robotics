/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#include "MotionControllerTestingViewer.h"

#include <fstream>

double random(double const& min, double const& max)
{
    return static_cast<double>(std::rand()) / RAND_MAX * (max - min) + min;
}

RobotPosition generateRandomValidPosition(Map const& map)
{
    RobotPosition pos;
    bool done = false;
    while (!done)
    {
        pos.x = random(0, table_dimensions::table_size_x);
        pos.y = random(0, table_dimensions::table_size_y);
        pos.theta = random(0, 2 * M_PI);
        done = !map.detectCollision(pos);
    }
    return pos;
}

int main (int argc, char *argv[])
{
    miam::RobotPosition startPosition(2820.81, 1113.5, 1.94296);
    miam::RobotPosition targetPosition(1322.05, 698.885, 2.00345);


    Logger logger;

    MotionController motionController(
        main_robot::generateParams(),
        &logger);


    bool nogui = false;
    bool unittest = false;
    for (int i = 1; i < argc; i++)
    {
        if (std::string(argv[i]) == "--nogui")
            nogui = true;
        if (std::string(argv[i]) == "--unittest")
            unittest = true;
    }
    if (nogui)
    {
        logger.start("testOutput.miam");
        motionController.resetPosition(startPosition);
        motionController.computeMPCTrajectory(
            targetPosition,
            motionController.getDetectedObstacles(),
            tf::DEFAULT);
        return 0;
    }

    if (unittest)
    {
        std::srand(42);
        // Generate random scenarios, solve them, and store result in CSV file
        int const N_SCENARIOS = 100;
        int const N_OBS_MAX = 2;

        std::ofstream fOutput("test_result.csv");
        fOutput << "AStarComputeDuration,MPCComputeDuration,PrintDuration,PathLength,MPCDuration" << std::endl;

        int niter = 0;
        for (int nObs = 0; nObs <= N_OBS_MAX; nObs++)
            for (int i = 0; i < N_SCENARIOS; i++)
            {
                logger.start("unittest" + std::to_string(niter) + ".miam");
                niter++;
                Map map = motionController.getGameState()->generateMap();
                // Generate obstacles
                std::vector<Obstacle> obstacles;
                for (int k = 0; k < nObs; k++)
                {
                    Obstacle o(generateRandomValidPosition(map), random(200, 400));
                    obstacles.push_back(o);
                    map.addCollisionCircle(std::get<0>(o), std::get<1>(o));
                }

                startPosition = generateRandomValidPosition(map);
                motionController.resetPosition(startPosition);
                RobotPosition endPosition;
                bool done = false;
                while (!done)
                {
                    endPosition = generateRandomValidPosition(map);
                    done = (endPosition - startPosition).norm() > 600;
                }

                auto traj = motionController.computeMPCTrajectory(
                    endPosition,
                    obstacles,
                    tf::DEFAULT);

                fOutput << UNITTEST_planningComputeDuration << ",";
                fOutput << UNITTEST_mpcComputeDuration << ",";
                fOutput << UNITTEST_printDuration << ",";
                double length = 0;
                for (auto traj: UNITTEST_POINTTURN_TRAJ)
                    length += (traj->getEndPoint().position - traj->getCurrentPoint(0.0).position).norm();
                fOutput << length << ",";
                fOutput << traj.getDuration() << std::endl;
                if (nObs == 3 && i == 15)
                    return -1;
            }
        fOutput.close();
        std::cout << "Test done, results written in 'test_result.csv'" << std::endl;
        return 0;
    }

    auto app = Gtk::Application::create();
    //load main window layout from glade
    auto refBuilder = Gtk::Builder::create();
    try
    {
        refBuilder->add_from_file("./config/MainWindow.glade");
    }
    catch(const Glib::FileError& ex)
    {
        std::cerr << "FileError: " << ex.what() << std::endl;
    }
    catch(const Glib::MarkupError& ex)
    {
        std::cerr << "MarkupError: " << ex.what() << std::endl;
    }
    catch(const Gtk::BuilderError& ex)
    {
        std::cerr << "BuilderError: " << ex.what() << std::endl;
    }

    // Create handler.
    MotionControllerTestingViewer *viewer = nullptr;
    refBuilder->get_widget_derived(
        "mainWindow",
        viewer,
        "./config/vinyle.png",
        &motionController,
        &logger);

    viewer->setStartPosition(startPosition);
    viewer->setEndPosition(targetPosition);
    return app->run(*viewer);
}


