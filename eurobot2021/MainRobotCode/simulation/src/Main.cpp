/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#include <gtkmm/application.h>
#include <cstdlib>
#include <iostream>
#include <miam_utils/Logger.h>

#include "Viewer.h"
#include "Strategy.h"
#include "ServoHandler.h"

Glib::RefPtr<Gtk::Application> app;

int main (int argc, char *argv[])
{
    bool isOldRobot = false;
    if (argc > 1)
    {
        if (strcmp(argv[1], "old") == 0)
            isOldRobot = true;
        else
        {
            std::cout << argv[1] << std::endl;
            std::cout << "Unknown argument. Usage: ./StrategyViewer [old]" << std::endl;
            exit(-1);
        }
    }

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
    refBuilder->get_widget_derived("mainWindow", viewer, "./config/tableAgeOfBots.png");
    ViewerRobot mainRobot("./config/mainRobotAgeOfBots.png", matchStrategy);
    viewer->addRobot(mainRobot);

    return app->run(*viewer);
}


