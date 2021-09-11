/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#include <gtkmm/application.h>
#include <cstdlib>
#include <iostream>
#include <miam_utils/Logger.h>

#include "Viewer.h"
#include "Strategy.h"

Glib::RefPtr<Gtk::Application> app;

int main (int argc, char *argv[])
{
    srand (time(NULL));
    app = Gtk::Application::create(argc, argv);

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

    // Create robots.
    ViewerRobot mainRobot("./config/mainRobot.png", mainRobotStrategy);

    ViewerRobot secondaryRobot("./config/secondaryRobot.png", secondaryRobotStrategy, 0, 0, 1);

    // Create handler.
    Viewer *viewer = nullptr;
    refBuilder->get_widget_derived("mainWindow", viewer);
    viewer->addRobot(mainRobot);
    viewer->addRobot(secondaryRobot);

    return app->run(*viewer);
}


