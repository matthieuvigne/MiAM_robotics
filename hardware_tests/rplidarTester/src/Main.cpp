/// \file Main.cpp Main application file.
/// \brief This file is responsible for creating the GTK application and corresponding window.
///
/// \author Matthieu Vigne


#include <gtkmm/application.h>

#include "GRPLidar.h"


GRPLidar *gprlidar;

// Stop motor before exit.
void killCode(int x)
{
    gprlidar->drawingArea_->lidar.stop();
    exit(0);
}


int main (int argc, char *argv[])
{
    std::string portName = "/dev/ttyUSB0";
    if (argc > 1)
        portName = argv[1];

    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create(argc, argv);

    //load main window layout from glade
    auto refBuilder = Gtk::Builder::create();
    try
    {
        refBuilder->add_from_file("GRPLidar.glade");
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
    refBuilder->get_widget_derived("mainWindow", gprlidar);

	if(!gprlidar->drawingArea_->lidar.init(portName))
	{
		std::cout << "Failed to init lidar on " << portName << std::endl;
        return -1;
	}

    signal(SIGINT, killCode);
    signal(SIGTERM, killCode);

    return app->run(*gprlidar);
}


