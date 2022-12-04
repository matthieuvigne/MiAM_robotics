/// \file Main.cpp Main application file.
/// \brief This file is responsible for creating the GTK application and corresponding window.
///
/// \author Matthieu Vigne


#include <gtkmm/application.h>

#include "GRPLidar.h"

Glib::RefPtr<Gtk::Application> app;

int main (int argc, char *argv[])
{
    srand (time(NULL));
    app = Gtk::Application::create(argc, argv);

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
    GRPLidar *gprlidar = nullptr;
    refBuilder->get_widget_derived("mainWindow", gprlidar);

	if(!gprlidar->drawingArea_->lidar.init("/dev/ttyUSB0"))
	{
		std::cout << "Failed to init Lidar" << std::endl;
        return -1;
	}

    return app->run(*gprlidar);
}


