/// \author Matthieu Vigne
/// \copyright GNU GPLv3

#include "Viewer.h"
#include <iomanip>
#include <sstream>
#include "Parameters.h"
#include "MessageSender.h"

// double const ROBOT_DT = 1.0e-9 * ROBOT_UPDATE_PERIOD;
double const MATCH_TIME = 100.0;
double const TRAJ_SERIALIZATION_INTERVAL = 0.1;

std::unique_ptr<float > serializationResults;
int serializationResultsSizeInFloatNumber = 0;

ObjectRow objectrow;

RobotParameters robotParameters;
TrajectoryConfig trajectoryConfig;
std::vector<RobotPosition > positions;
TrajectoryVector newTrajectory;

bool exitApp(GdkEventAny* event)
{
    Gtk::Main::quit();
    return true;
}

// Build window from Glade.
Viewer::Viewer(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade, std::string const& tableImagePath) :
    Gtk::Window(cobject),
    mmToCairo_(1.0),
    originX_(0.0),
    originY_(0.0)
{
    // Associate key press to entry.
    Gtk::Widget *window;
    refGlade->get_widget("mainWindow", window);

    // window->signal_destroy().connect(Gtk::Main::quit());
    window->signal_delete_event().connect(sigc::ptr_fun(&exitApp));

    // Associate widgets.
    refGlade->get_widget("mousePositionLabel", mousePositionLabel);
    refGlade->get_widget("timeLabel", timeLabel);
    refGlade->get_widget("scoreLabel", scoreLabel);
    refGlade->get_widget("progressBar", progressBar);
    refGlade->get_widget("switchButton", switchButton);
    refGlade->get_widget("recipientIPTextView", recipientIPTextView);
    refGlade->get_widget("drawingArea", drawingArea);
    
    refGlade->get_widget("coordinateList", treeView);
    refGlade->get_widget("maxVelocityTextView", maxVelocityTextView);
    refGlade->get_widget("maxAccelerationTextView", maxAccelerationTextView);
    
    refGlade->get_widget("recipientIDTextView", recipientIDTextView);

    //create the tree
	treeModel = Gtk::ListStore::create(objectrow);
	treeView->set_model(treeModel);
	//define the cells
	treeView->append_column_editable("x", objectrow.x);
	treeView->append_column_editable("y", objectrow.y);	
	treeView->append_column_editable("theta", objectrow.theta);	
	// treeView->append_column_editable("v", objectrow.v);	
	// treeView->append_column_editable("w", objectrow.w);	
    treeView->set_reorderable();
    treeModel->signal_row_changed().connect(sigc::mem_fun(this, &Viewer::valueChanged));
	for(int x = 0; x < 2; x++) 
	{
		//for all columns, center
		Gtk::TreeViewColumn* pColumn = treeView->get_column(x);
		Gtk::CellRendererText* pRenderer =	static_cast<Gtk::CellRendererText*>(pColumn->get_first_cell());
		pRenderer->property_xalign().set_value(0.5);
        pRenderer->property_editable() = true;
		pColumn->set_alignment(0.5);
		pColumn->set_expand(true);
		pColumn->set_sort_column(x);
		// //last column: weight, color...
		// if (x== 3)
		// {
		// 	pRenderer->property_weight().set_value(Pango::Weight::WEIGHT_BOLD);
		// 	// pColumn->add_attribute(pRenderer->property_background_rgba(), objectrow.color);
		// }
	}

    selection = treeView->get_selection(); 
    // selection->set_mode(Gtk::SELECTION_MULTIPLE); 

    row = *(treeModel->append());
    row[objectrow.x] = 1000.0;
    row[objectrow.y] = 100.0;
    row[objectrow.theta] = 0.0;
    // row[objectrow.v] = 1000.0;
    // row[objectrow.w] = 10.0;

    row = *(treeModel->append());
    row[objectrow.x] = 2000.0;
    row[objectrow.y] = 200.0;
    row[objectrow.theta] = 0.0;
    // row[objectrow.v] = 2000.0;
    // row[objectrow.w] = 20.0;


    row = *(treeModel->append());
    row[objectrow.x] = 2200.0;
    row[objectrow.y] = 1000.0;
    row[objectrow.theta] = 0.0;
    // row[objectrow.v] = 2000.0;
    // row[objectrow.w] = 20.0;


    drawingArea->signal_draw().connect(sigc::mem_fun(this, &Viewer::redraw));

    drawingArea->set_events(Gdk::POINTER_MOTION_MASK | Gdk::BUTTON_PRESS_MASK  | Gdk::BUTTON_RELEASE_MASK | Gdk::BUTTON_MOTION_MASK | Gdk::SCROLL_MASK);

    drawingArea->signal_motion_notify_event().connect(sigc::mem_fun(this, &Viewer::mouseMove));
    drawingArea->signal_button_press_event().connect(sigc::mem_fun(this, &Viewer::clickObstacle));
    // drawingArea->signal_button_release_event().connect(sigc::mem_fun(this, &Viewer::clickObstacle));

    Gtk::Button *button;
    refGlade->get_widget("computeTrajectoryButton", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::playClicked));
    refGlade->get_widget("serializeAndSendButton", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::pauseClicked));
    refGlade->get_widget("sendIDButton", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::sendIDButtonClicked));

    refGlade->get_widget("deletePointButton", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::deletePointButtonClicked));

    // Load images.
    tableImage = Gdk::Pixbuf::create_from_file(tableImagePath, -1, -1);

    Glib::RefPtr<Gtk::TextBuffer> buffer = maxVelocityTextView->get_buffer();
    buffer->set_text("500");    

    // Glib::signal_timeout().connect(sigc::mem_fun(*this,&Viewer::runSimulation), 50);

    // Initialize motion planner
    trajectoryConfig.maxWheelVelocity = MAX_WHEEL_SPEED_MM_S;
    trajectoryConfig.maxWheelAcceleration = MAX_WHEEL_ACCELERATION_MM_S;
    trajectoryConfig.robotWheelSpacing = WHEEL_SPACING_MM;
    motionPlanner = new MotionPlanner(robotParameters);
}


Viewer::~Viewer()
{

}

bool Viewer::runSimulation()
{
    // // Increment currentTrajectoryIndex_ based on time elapsed: increment as much as needed to catch back with real
    // // clock.
    // std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
    // double realElapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastTime_).count();
    // while (isRunning_ && realElapsedTime > ROBOT_DT / simulationTimeRatio_)
    // {
    //     // std::cout << "Running -- simulationTime_: " << simulationTime_ << std::endl;
    //     simulationTime_ += ROBOT_DT;
    //     for(unsigned int i = 0; i < robots_.size(); i++)
    //     {
    //         ViewerRobot* r = robots_.at(i);
    //         // get the other robots
    //         std::vector<Vector2 > obstaclesPosition;
    //         for (unsigned int j = 0; j < robots_.size(); j++)
    //         {
    //             if (j != i)
    //             {
    //                 Vector2 v;
    //                 v << robots_.at(j)->getPosition().x, robots_.at(j)->getPosition().y;
    //                 obstaclesPosition.push_back(v);
    //             }
    //         }
    //         // obstacles
    //         obstaclesPosition.push_back(obstaclePosition_);
    //         obstaclesPosition.push_back(obstacle2Position_);

    //         SimulatorData data;
    //         data.isStartingSwitchPluggedIn = switchButton->get_active();
    //         data.obstaclesPosition = obstaclesPosition;
    //         r->tick(data);
    //     }
    //     // Reset time on match start.
    //     static double lastMatchTime = 0.0;
    //     if (robots_.at(0)->getMatchTime() > 0 && lastMatchTime == 0)
    //         simulationTime_ = 0;
    //     lastMatchTime = robots_.at(0)->getMatchTime();

    //     realElapsedTime -= ROBOT_DT / simulationTimeRatio_;
    //     lastTime_ += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(ROBOT_DT / simulationTimeRatio_));
    //     if (simulationTime_ > MATCH_TIME && robots_.at(0)->getMatchTime() > 0)
    //     {
    //         isRunning_ = false;
    //         simulationTime_ = MATCH_TIME;
    //     }
    // }
    // if (!isRunning_)
    //     lastTime_ = currentTime;

    // progressBar->set_fraction(std::min(simulationTime_ / MATCH_TIME, 1.0));
    // std::stringstream stream;
    // stream << "Time: " << std::fixed << std::setprecision(3) << simulationTime_;
    // progressBar->set_text(stream.str());

    drawingArea->queue_draw();
    return true;
}


void Viewer::start()
{
    Glib::signal_timeout().connect(sigc::mem_fun(*this,&Viewer::runSimulation), 50);
    // resetClicked();
}

// void Viewer::addRobot(ViewerRobot & robot)
// {
//     robots_.push_back(&robot);
// }

bool Viewer::redraw(const Cairo::RefPtr<Cairo::Context>& cr)
{

    positions.clear();
    auto children = treeModel->children();
    for (auto iter = children.begin(), end = children.end(); iter != end; ++iter)
    {
        auto row = *iter;
        //Do something with the row - see above for set/get.
        RobotPosition newPosition;
        newPosition.x = row[objectrow.x];
        newPosition.y = row[objectrow.y];
        newPosition.theta = row[objectrow.theta];
        positions.push_back(newPosition);
        // std::cout << newPosition << std::endl;
    }

    // Put scaled table, keeping ratio.
    double heightToWidthRatio = tableImage->get_height() / (1.0 * tableImage->get_width());
    int widgetWidth = drawingArea->get_allocated_width();
    int widgetHeight = drawingArea->get_allocated_height();

    double newWidth = std::min(widgetWidth, static_cast<int>(widgetHeight / heightToWidthRatio));
    double newHeight = std::min(widgetHeight, static_cast<int>(heightToWidthRatio * widgetWidth));
    Gdk::Cairo::set_source_pixbuf(cr,
                                  tableImage->scale_simple(newWidth, newHeight, Gdk::INTERP_BILINEAR ),
                                  (widgetWidth - newWidth) / 2,
                                  (widgetHeight - newHeight) / 2);
    cr->paint();

    // Get table origin and scaling.
    mmToCairo_ = newWidth / TABLE_WIDTH_MM;

    originX_ = (widgetWidth - newWidth) / 2 + TABLE_MARGIN_MM * mmToCairo_;
    originY_ = (widgetHeight - newHeight) / 2 + TABLE_MARGIN_MM * mmToCairo_;

    cr->translate(originX_, originY_);

    // int score = 0;
    // for(auto robot : robots_)
    // {
    //     robot->draw(cr, mmToCairo_);
    // }

    // Draw path .
    if (!positions.empty()) 
    {
        double pointX =  mmToCairo_ * positions.at(0).x;
        double pointY =  mmToCairo_ * (TABLE_HEIGHT_MM - positions.at(0).y);

        cr->move_to(pointX, pointY);
        for(unsigned long i = 0; i < positions.size(); i+=1)
        {
            pointX =  mmToCairo_ * positions.at(i).x;
            pointY =  mmToCairo_ * (TABLE_HEIGHT_MM - positions.at(i).y);
            cr->line_to(pointX, pointY);
        }
        cr->set_source_rgb(1.0, 1.0, 1.0);
        cr->set_line_width(5.0);
        cr->stroke_preserve();
        cr->set_source_rgb(1.0, 0.0, 0.0);
        cr->set_line_width(2.0);
        cr->stroke();
    }

    // Draw result
    if (!newTrajectory.empty()) 
    {
        double pointX =  mmToCairo_ * newTrajectory.getCurrentPoint(0).position.x;
        double pointY =  mmToCairo_ * (TABLE_HEIGHT_MM - newTrajectory.getCurrentPoint(0).position.y);

        cr->move_to(pointX, pointY);
        double time = 0.0;
        do
        {
            time += 0.01;
            pointX =  mmToCairo_ * newTrajectory.getCurrentPoint(time).position.x;
            pointY =  mmToCairo_ * (TABLE_HEIGHT_MM - newTrajectory.getCurrentPoint(time).position.y);
            cr->line_to(pointX, pointY);
        } while (time <= newTrajectory.getDuration());

        // last point
        time = newTrajectory.getDuration();
        pointX =  mmToCairo_ * newTrajectory.getCurrentPoint(time).position.x;
        pointY =  mmToCairo_ * (TABLE_HEIGHT_MM - newTrajectory.getCurrentPoint(time).position.y);
        cr->line_to(pointX, pointY);

        cr->set_source_rgb(1.0, 1.0, 1.0);
        cr->set_line_width(5.0);
        cr->stroke_preserve();
        cr->set_source_rgb(0.0, 0.0, 1.0);
        cr->set_line_width(2.0);
        cr->stroke();
    }

    // // Draw obstacle.
    // cr->set_source_rgb(1.0, 0.0, 0.0);
    // cr->arc(mmToCairo_ * obstaclePosition_(0), mmToCairo_ * (TABLE_HEIGHT_MM - obstaclePosition_(1)), mmToCairo_ * 200, 0, 2 * M_PI - 0.1);
    // cr->fill();

    // // Draw obstacle2.
    // cr->set_source_rgb(0.0, 0.0, 1.0);
    // cr->arc(mmToCairo_ * obstacle2Position_(0), mmToCairo_ * (TABLE_HEIGHT_MM - obstacle2Position_(1)), mmToCairo_ * 200, 0, 2 * M_PI - 0.1);
    // cr->fill();

    // Update labels.
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << simulationTime_;
    timeLabel->set_text("Time: " + stream.str());
    scoreLabel->set_text("Traj time: " + std::to_string(newTrajectory.getDuration()));

    // Draw game state.
    cr->scale(mmToCairo_, mmToCairo_);
    // robots_[0]->gameState_.draw(cr, robots_[0]->getPosition());

    return true;
}


bool Viewer::mouseMove(GdkEventMotion* motion_event)
{
    double posX, posY;
    posX = (motion_event->x - originX_) / mmToCairo_;
    posY = TABLE_HEIGHT_MM - (motion_event->y - originY_) / mmToCairo_;

    if (motion_event->state & Gdk::BUTTON1_MASK)
    {
        obstaclePosition_(0) = posX;
        obstaclePosition_(1) = posY;
    }
    else if (motion_event->state & Gdk::BUTTON3_MASK)
    {
        obstacle2Position_(0) = posX;
        obstacle2Position_(1) = posY;
    }
    mousePositionLabel->set_text("(" + std::to_string(static_cast<int>(posX)) + ", " + std::to_string(static_cast<int>(posY)) + ")");
    drawingArea->queue_draw();
    return true;
}


bool Viewer::clickObstacle(GdkEventButton* motion_event)
{

    if (motion_event->button == 1)
    {
        obstaclePosition_(0) = motion_event->x;
        obstaclePosition_(1) = motion_event->y;
        // // Convert coordinates to table coordinates
        // obstaclePosition_(0) = (obstaclePosition_(0) - originX_) / mmToCairo_;
        // obstaclePosition_(1) = TABLE_HEIGHT_MM - (obstaclePosition_(1) - originY_) / mmToCairo_;
        row = *(treeModel->append());
        row[objectrow.x] = (obstaclePosition_(0) - originX_) / mmToCairo_;
        row[objectrow.y] = TABLE_HEIGHT_MM - (obstaclePosition_(1) - originY_) / mmToCairo_;
        row[objectrow.theta] = 0.0;
    }
    // else if (motion_event->button == 3)
    // {
    //     obstacle2Position_(0) = motion_event->x;
    //     obstacle2Position_(1) = motion_event->y;
    //     // Convert coordinates to table coordinates
    //     obstacle2Position_(0) = (obstacle2Position_(0) - originX_) / mmToCairo_;
    //     obstacle2Position_(1) = TABLE_HEIGHT_MM - (obstacle2Position_(1) - originY_) / mmToCairo_;
    // }

    drawingArea->queue_draw();
    return true;
}

void Viewer::playClicked()
{
    // isRunning_ = true;

    std::cout << "playClicked" << std::endl;

    // Compute new trajectory
    newTrajectory = motionPlanner->solveTrajectoryFromWaypoints(positions, trajectoryConfig);


}

void Viewer::pauseClicked()
{
    // isRunning_ = false;
    std::cout << "Serializing" << std::endl;

    int N = std::ceil(newTrajectory.getDuration() / TRAJ_SERIALIZATION_INTERVAL);
    double deltat = newTrajectory.getDuration() / N;
    serializationResultsSizeInFloatNumber = 3 + 5*(N+1);

    std::cout << "Size=" << serializationResultsSizeInFloatNumber << ", deltat=" << deltat << ", duration: " << newTrajectory.getDuration() << std::endl;

    serializationResults.reset(new float[serializationResultsSizeInFloatNumber]());

    int serializationIndex = 0;

    // first 3 bytes are size of trajectory (nb of pts) and duration
    serializationResults.get()[serializationIndex++] = MessageType::NEW_TRAJECTORY;
    serializationResults.get()[serializationIndex++] = N+1;
    serializationResults.get()[serializationIndex++] = newTrajectory.getDuration();

    for (int i = 0; i <= N; i++) 
    {
        TrajectoryPoint pt;
        if (i < N)
        {
            pt = newTrajectory.getCurrentPoint(deltat * i);
        } else {
            pt = newTrajectory.getEndPoint();
        }
        // res.push_back((float)pt.position.x);
        // res.push_back((float)pt.position.y);
        // res.push_back((float)pt.position.theta);
        // res.push_back((float)pt.linearVelocity);
        // res.push_back((float)pt.angularVelocity);
        // serializationResults.get()[5*i] = (float)pt.position.x;
        // serializationResults.get()[5*i+1] = (float)pt.position.y;
        // serializationResults.get()[5*i+2] = (float)pt.position.theta;
        // serializationResults.get()[5*i+3] = (float)pt.linearVelocity;
        // serializationResults.get()[5*i+4] = (float)pt.angularVelocity;
        serializationResults.get()[serializationIndex++] = (float)pt.position.x;
        serializationResults.get()[serializationIndex++] = (float)pt.position.y;
        serializationResults.get()[serializationIndex++] = (float)pt.position.theta;
        serializationResults.get()[serializationIndex++] = (float)pt.linearVelocity;
        serializationResults.get()[serializationIndex++] = (float)pt.angularVelocity;
    }

    std::cout << "Size in float number: " << serializationResultsSizeInFloatNumber << std::endl;

    // for (float f : res)
    // {
    //     std::cout << f << std::endl;
    // }

    // for (int i=0; i< serializationResultsSizeInFloatNumber; i++)
    // {
    //     float f = serializationResults.get()[i];
    //     std::cout << f << std::endl;
    // }

    // only send if trajectory not empty
    if (serializationResultsSizeInFloatNumber > 0 && N > 0)
    {
        std::string str_ip_address = recipientIPTextView->get_buffer()->get_text();
        std::cout << "Sending trajectory to IP " << recipientIPTextView->get_buffer()->get_text() << std::endl;
        // message_sender::send_message(serializationResults.get(), serializationResultsSizeInFloatNumber, str_ip_address.c_str());
        // message_sender::send_message(serializationResults.get(), serializationResultsSizeInFloatNumber*sizeof(serializationResults.get()[0]), "127.0.0.1");
        message_sender::send_message(serializationResults.get(), serializationResultsSizeInFloatNumber, str_ip_address.c_str());
    }
    else
    {
        std::cout << "Not sending" << std::endl;
    }

}

void Viewer::valueChanged(const Gtk::TreeModel::Path& path, const Gtk::TreeModel::iterator& iter)
{
    drawingArea->queue_draw();
}

void Viewer::updateTimeRatio()
{
    // simulationTimeRatio_ = simulationRatioSpin->get_value();
}

void Viewer::deletePointButtonClicked()
{
//     Gtk::TreeModel::Row* selected_row = treeView->get_selection()->get_selected();

    auto iter = selection->get_selected();
    if(iter) //If anything is selected
    {
        // auto row = *iter;
        //Do something with the row.
        treeModel->erase(iter);
    }
    // treeModel->remove()

    // treeView->erase(selected_row);
    // // for(int i = paths.size()-1; i>=0; i--) {
    // //     store->remove(store->get_iter(paths.at(i)));
    // // }
    drawingArea->queue_draw();
}

void Viewer::sendIDButtonClicked()
{

}