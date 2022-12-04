/// \file GRPLidar.cpp Implementation of the GRPLidar class.
///
/// \author Matthieu Vigne


#include "GRPLidar.h"


// Build window from Glade.
GRPLidar::GRPLidar(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade) : Gtk::Window(cobject)
{
	refGlade->get_widget_derived("drawingArea", drawingArea_);
    drawingArea_->lidar = RPLidarHandler(M_PI_4);
	Glib::signal_timeout().connect(sigc::mem_fun(*this, &GRPLidar::update), 10);
	signal_delete_event().connect(sigc::mem_fun(*this,&GRPLidar::exit));

	drawingArea_->set_events(Gdk::SCROLL_MASK);
	drawingArea_->signal_scroll_event().connect(sigc::mem_fun(drawingArea_, &LidarDrawing::userZoom));
	drawingArea_->zoom_ = 1.0;
}


GRPLidar::~GRPLidar()
{

}


bool GRPLidar::update()
{
	drawingArea_->queue_draw();
	return true;
}

bool GRPLidar::exit(GdkEventAny* event)
{
	drawingArea_->lidar.stop();
	return false;
}

