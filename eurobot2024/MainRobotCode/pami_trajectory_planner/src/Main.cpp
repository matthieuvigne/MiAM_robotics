/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#include <gtkmm/application.h>
#include <cstdlib>
#include <iostream>
#include <miam_utils/Logger.h>

#include "Viewer.h"
// #include "main_robot/Strategy.h"


#include "main_robot/Parameters.h"
// #include "main_robot/Strategy.h"

Glib::RefPtr<Gtk::Application> app;

int main (int argc, char *argv[])
{
    srand (time(NULL));
    app = Gtk::Application::create();

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
    Viewer *viewer = nullptr;
    refBuilder->get_widget_derived("mainWindow", viewer, "./config/vinyles_table_2024_FINAL_V1.png");
    // main_robot::Strategy mainStrategy;
    // ViewerRobot mainRobot(main_robot::generateParams(), "./config/mainRobot2023.png", &mainStrategy, 1.0, 0.0, 0.0, "mainRobot.");
    // viewer->addRobot(mainRobot);

    // Start telemetry for both robots.
    system("/miam_workspace/src/MiAM_robotics/teleplot_binaries/teleplot-linux > /dev/null 2> /dev/null&");
    std::cout << "Teleplot server started, open your browser at 127.0.0.1:8080" << std::endl;

    viewer->resetClicked();

    return app->run(*viewer);
}


