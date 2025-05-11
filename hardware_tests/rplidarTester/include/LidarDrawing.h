/// \file LidarDrawing.h Drawing handling.
/// \details LidarDrawing is a Gtk::DrawingArea contained inside the main window, responsible for displaying
///          lidar data.
/// \author Matthieu Vigne, matthieu.vigne@laposte.net

#ifndef LIDAR_DRAWING
	#define LIDAR_DRAWING

	#include <gtkmm.h>
	#include <iostream>
	#include <miam_utils/RPLidarHandler.h>

	/// \class LidarDrawing A Gtk::DrawingArea for displaying lidar data.
	class LidarDrawing : public Gtk::DrawingArea
	{
		public:
			/// \brief Constructor.
			LidarDrawing();

			LidarDrawing(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade) : Gtk::DrawingArea(cobject)
			{
			}

			/// \brief Callback function to redraw the drawing area.
			bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr);

			// Zoom handling
			bool userZoom(GdkEventScroll *event);

			RPLidarHandler lidar;

			double zoom_;
	};

#endif
