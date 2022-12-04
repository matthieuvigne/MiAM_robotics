/// \file GRPLidar.h Main application file.
/// \details This header contains the decleration of the GRPLidar class, the main code of the application
///          handling the GUI and lidar communcation.
///
/// \author Matthieu Vigne, matthieu.vigne@laposte.net


#ifndef GRPLIDAR_HANDLER
#define GRPLIDAR_HANDLER

#include <gtkmm.h>
#include <iostream>

#include "LidarDrawing.h"


class GRPLidar : public Gtk::Window
{
	public:
		/// \brief Constructor.
		GRPLidar(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade);

		/// \brief Destructor.
		~GRPLidar();

		bool update();

		bool exit(GdkEventAny* event);
		LidarDrawing *drawingArea_;
	private:
};

#endif
