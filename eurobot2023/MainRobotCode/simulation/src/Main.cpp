/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#include <gtkmm/application.h>
#include <cstdlib>
#include <iostream>
#include <miam_utils/Logger.h>

#include "Viewer.h"
#include "main_robot/Strategy.h"
#include "common/ServoHandler.h"


#include "main_robot/Parameters.h"
#include "main_robot/Strategy.h"
#include "main_robot/RobotGUI.h"

#include "secondary_robot/Parameters.h"
#include "secondary_robot/Strategy.h"

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
    refBuilder->get_widget_derived("mainWindow", viewer, "./config/tableCherryOnTheCake.png");
    main_robot::Strategy mainStrategy;
    ViewerRobot mainRobot(main_robot::generateParams(), "./config/mainRobotAgeOfBots.png", &mainStrategy);
    viewer->addRobot(mainRobot);

    // secondary_robot::Strategy secondaryStrategy;
    // ViewerRobot secondaryRobot(secondary_robot::generateParams(), "./config/secondaryRobot.png", &secondaryStrategy, 0.0, 0.0, 1.0);
    // viewer->addRobot(secondaryRobot);

    viewer->resetClicked();

    // Create gui
    std::thread t(main_robot::startRobotGUI, &mainRobot);
    t.detach();

    return app->run(*viewer);
}


