/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#include "MotionControllerTestingViewer.h"


int main (int argc, char *argv[])
{
    miam::RobotPosition startPosition(310.0, 1690.0, 0.0);
    miam::RobotPosition targetPosition(900.0, 1300.0, -0.5);

    Logger logger;

    MotionController motionController(
        main_robot::generateParams(),
        &logger);


    bool nogui = false;
    for (int i = 1; i < argc; i++)
    {
        if (std::string(argv[i]) == "--nogui")
            nogui = true;
    }
    if (nogui)
    {
        logger.start("testOutput.hdf5");
        motionController.resetPosition(startPosition);
        motionController.computeMPCTrajectory(
            targetPosition,
            motionController.getDetectedObstacles(),
            true);
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
        "./config/vinyles_table_2024_FINAL_V1.png",
        &motionController,
        &logger);

    viewer->setStartPosition(startPosition);
    viewer->setEndPosition(targetPosition);
    return app->run(*viewer);
}


