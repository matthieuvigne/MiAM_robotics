/// \file LidarDrawing.cpp Implementation of the LidarDrawing class.
///
/// \author Matthieu Vigne

#include "LidarDrawing.h"

// Dimension of the window, in mm.
#define REAL_DIM 2200

LidarDrawing::LidarDrawing():
    zoom_(1.0)
{

}


bool LidarDrawing::on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
{
    // Get new lidar data.
    lidar.update();

    // Draw.
    int widgetWidth = this->get_allocated_width();
    int widgetHeight = this->get_allocated_height();

    // Translate to have the origin at the center.
    cr->translate(widgetWidth / 2.0, widgetHeight / 2.0);

    // Scale to have units in mm.
    double scale = 2 * REAL_DIM / static_cast<double>(std::min(widgetWidth, widgetHeight));
    if (zoom_ < 0.1)
        zoom_ = 0.1;
    scale /= zoom_;
    cr->scale(1 / scale, 1 / scale);

    // Draw target.
    cr->set_source_rgb(0.1,0.1,0.1);
    cr->set_line_width(2);
    for(int i = 0; i < 8; i++)
    {
        cr->save();
        cr->rotate_degrees(45 * i);
        cr->move_to(0,0);
        cr->line_to(REAL_DIM, 0);
        cr->stroke();
        cr->restore();
    }

    for(int i = 1000; i <= REAL_DIM; i+=1000)
    {
        cr->begin_new_sub_path();
        cr->arc(0, 0, i, 0, 2 * G_PI);
        cr->stroke();
        cr->move_to(i, 50);
        cr->set_font_size(15 * scale);
        cr->show_text(std::to_string(i));
    }

    // Draw robots.
    cr->set_source_rgb(0, 0, 1);
    cr->set_line_width(5);
    for(DetectedRobot robot : lidar.detectedRobots_)
    {
        LidarPoint point = robot.point;
        cr->begin_new_sub_path();
        // Minus sign as y axis is pointing downward.
        cr->arc(point.r * std::cos(point.theta), -point.r * std::sin(point.theta), 80, 0, 2 * G_PI);
        cr->stroke();
        cr->move_to(0, 0);
        cr->line_to(point.r * std::cos(point.theta), -point.r * std::sin(point.theta));
        cr->stroke();
    }

    // Draw points.
    cr->set_source_rgb(1, 0, 0);
    // Start from current point and do at most one full cycle
    int startIndex = lidar.debuggingBufferPosition_;
    double startAngle = lidar.debuggingBuffer_[startIndex].theta;

    for(int i = 0; i < DEBUGGING_BUFFER_LENGTH; i ++)
    {
        LidarPoint point = lidar.debuggingBuffer_[(startIndex + i) % DEBUGGING_BUFFER_LENGTH];
        if(point.theta < startAngle && point.theta > startAngle - 0.1 && i > 100)
            break;
        // Choose color based on blob number.
        //~ switch(point.blobNumber)
        //~ {
            //~ case 1: cr->set_source_rgb(1, 0, 0); break;
            //~ case 2: cr->set_source_rgb(0, 1, 0); break;
            //~ case 3: cr->set_source_rgb(0, 0, 1); break;
            //~ case 4: cr->set_source_rgb(1, 1, 0); break;
            //~ default: cr->set_source_rgb(0, 0, 0); break;
        //~ }
        cr->begin_new_sub_path();
        // Minus sign as y axis is pointing downward.
        cr->arc(point.r * std::cos(point.theta), -point.r * std::sin(point.theta), 5, 0, 2 * G_PI);
        cr->fill();
    }

    return TRUE;
}



bool LidarDrawing::userZoom(GdkEventScroll *event)
{
    if(event->direction == GDK_SCROLL_UP)
        zoom_ += 0.1;
    else
        zoom_ -= 0.1;
    return true;
}