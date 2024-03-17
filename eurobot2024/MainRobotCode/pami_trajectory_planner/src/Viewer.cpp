/// \author Matthieu Vigne
/// \copyright GNU GPLv3

#include "Viewer.h"
#include <iomanip>
#include <sstream>
#include "Parameters.h"
#include "MessageSender.h"

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

    window->signal_delete_event().connect(sigc::ptr_fun(&exitApp));

    // Associate widgets.
    refGlade->get_widget("mousePositionLabel", mousePositionLabel);
    refGlade->get_widget("timeLabel", timeLabel);
    refGlade->get_widget("scoreLabel", scoreLabel);
    refGlade->get_widget("progressBar", progressBar);
    refGlade->get_widget("switchButton", switchButton);

    refGlade->get_widget("recipientIPTextView", recipientIPTextView);
    // default IP
    recipientIPTextView->get_buffer()->set_text("127.0.0.1");

    refGlade->get_widget("drawingArea", drawingArea);
    
    refGlade->get_widget("coordinateList", treeView);
    refGlade->get_widget("maxVelocityTextView", maxVelocityTextView);
    refGlade->get_widget("maxAccelerationTextView", maxAccelerationTextView);
    
    refGlade->get_widget("recipientIDTextView", recipientIDTextView);
    // default recipient ID
    recipientIDTextView->get_buffer()->set_text("0");

    refGlade->get_widget("matchTimeTextView", matchTimeTextView);

    //create the tree
	treeModel = Gtk::ListStore::create(objectrow);
	treeView->set_model(treeModel);
	//define the cells
	treeView->append_column_editable("x", objectrow.x);
	treeView->append_column_editable("y", objectrow.y);	
	treeView->append_column_editable("theta", objectrow.theta);	
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
	}

    selection = treeView->get_selection(); 

    row = *(treeModel->append());
    row[objectrow.x] = 1000.0;
    row[objectrow.y] = 100.0;
    row[objectrow.theta] = 0.0;

    row = *(treeModel->append());
    row[objectrow.x] = 2000.0;
    row[objectrow.y] = 200.0;
    row[objectrow.theta] = 0.0;

    row = *(treeModel->append());
    row[objectrow.x] = 2200.0;
    row[objectrow.y] = 1000.0;
    row[objectrow.theta] = 0.0;


    drawingArea->signal_draw().connect(sigc::mem_fun(this, &Viewer::redraw));
    drawingArea->set_events(Gdk::POINTER_MOTION_MASK | Gdk::BUTTON_PRESS_MASK  | Gdk::BUTTON_RELEASE_MASK | Gdk::BUTTON_MOTION_MASK | Gdk::SCROLL_MASK);

    drawingArea->signal_motion_notify_event().connect(sigc::mem_fun(this, &Viewer::mouseMove));

    Gtk::Button *button;
    refGlade->get_widget("computeTrajectoryButton", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::computeTrajectoryButtonClicked));
    refGlade->get_widget("serializeAndSendButton", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::serializeAndSendButtonClicked));
    refGlade->get_widget("sendIDButton", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::sendIDButtonClicked));

    refGlade->get_widget("deletePointButton", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::deletePointButtonClicked));

    refGlade->get_widget("setActiveTimeButton", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::setActiveTimeButtonClicked));
    refGlade->get_widget("stopMatchButton", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::stopMatchButtonClicked));

    // Load images.
    tableImage = Gdk::Pixbuf::create_from_file(tableImagePath, -1, -1);

    Glib::RefPtr<Gtk::TextBuffer> buffer = maxVelocityTextView->get_buffer();
    buffer->set_text("500");    

    // Initialize motion planner
    trajectoryConfig.maxWheelVelocity = MAX_WHEEL_SPEED_MM_S;
    trajectoryConfig.maxWheelAcceleration = MAX_WHEEL_ACCELERATION_MM_S;
    trajectoryConfig.robotWheelSpacing = WHEEL_SPACING_MM;
    motionPlanner = new MotionPlanner(robotParameters);
}


Viewer::~Viewer()
{

}

void Viewer::start()
{
    
}

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

    // Update labels.
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << simulationTime_;
    timeLabel->set_text("Time: " + stream.str());
    scoreLabel->set_text("Traj time: " + std::to_string(newTrajectory.getDuration()));

    // Draw game state.
    cr->scale(mmToCairo_, mmToCairo_);

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


void Viewer::computeTrajectoryButtonClicked()
{
    // isRunning_ = true;

    std::cout << "computeTrajectoryButtonClicked" << std::endl;

    // Compute new trajectory
    newTrajectory = motionPlanner->solveTrajectoryFromWaypoints(positions, trajectoryConfig);


}

void Viewer::serializeAndSendButtonClicked()
{
    std::cout << "Serializing" << std::endl;

    int N = std::ceil(newTrajectory.getDuration() / TRAJ_SERIALIZATION_INTERVAL);
    double deltat = newTrajectory.getDuration() / N;
    serializationResultsSizeInFloatNumber = 3 + 5*(N+1);

    std::cout << "Size=" << serializationResultsSizeInFloatNumber << ", deltat=" << deltat << ", duration: " << newTrajectory.getDuration() << std::endl;

    serializationResults.reset(new float[serializationResultsSizeInFloatNumber]());

    int serializationIndex = 0;

    // first 3 bytes are size of trajectory (nb of pts) and duration
    if (switchButton->get_active())
    {
        std::cout << "Trajectory saving enabled" << std::endl;
        serializationResults.get()[serializationIndex++] = MessageType::NEW_TRAJECTORY_SAVE;
    }
    else
    {
        std::cout << "Trajectory saving disabled" << std::endl;
        serializationResults.get()[serializationIndex++] = MessageType::NEW_TRAJECTORY;
    }
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
        serializationResults.get()[serializationIndex++] = (float)pt.position.x;
        serializationResults.get()[serializationIndex++] = (float)pt.position.y;
        serializationResults.get()[serializationIndex++] = (float)pt.position.theta;
        serializationResults.get()[serializationIndex++] = (float)pt.linearVelocity;
        serializationResults.get()[serializationIndex++] = (float)pt.angularVelocity;
    }

    std::cout << "Size in float number: " << serializationResultsSizeInFloatNumber << std::endl;

    // only send if trajectory not empty
    if (serializationResultsSizeInFloatNumber > 0 && N > 0)
    {
        std::string str_ip_address = recipientIPTextView->get_buffer()->get_text();
        std::cout << "Sending trajectory to IP " << recipientIPTextView->get_buffer()->get_text() << std::endl;
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

void Viewer::deletePointButtonClicked()
{
    auto iter = selection->get_selected();
    if(iter) //If anything is selected
    {
        //Do something with the row.
        treeModel->erase(iter);
    }
    drawingArea->queue_draw();
}

bool is_number(const std::string& s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}

void Viewer::sendIDButtonClicked()
{
    // get id
    std::string newID = recipientIDTextView->get_buffer()->get_text();
    
    if (is_number(newID)) 
    {
        int newID_number = std::stoi(newID);
        serializationResultsSizeInFloatNumber = 2;
        serializationResults.reset(new float[serializationResultsSizeInFloatNumber]());

        int serializationIndex = 0;
        serializationResults.get()[serializationIndex++] = MessageType::SET_ID;
        serializationResults.get()[serializationIndex++] = (float)newID_number;

        std::cout << "Size in float number: " << serializationResultsSizeInFloatNumber << std::endl;

        std::string str_ip_address = recipientIPTextView->get_buffer()->get_text();
        std::cout << "Sending trajectory to IP " << recipientIPTextView->get_buffer()->get_text() << std::endl;
        message_sender::send_message(serializationResults.get(), serializationResultsSizeInFloatNumber, str_ip_address.c_str());
    }
    else
    {
        std::cout << "ID is not a number: " << newID << std::endl;
    }
}

void Viewer::setActiveTimeButtonClicked()
{
    // get match time
    std::string newMatchTime = matchTimeTextView->get_buffer()->get_text();
    
    if (is_number(newMatchTime)) 
    {
        float matchTime = std::stof(newMatchTime);
        serializationResultsSizeInFloatNumber = 3;
        serializationResults.reset(new float[serializationResultsSizeInFloatNumber]());

        int serializationIndex = 0;
        serializationResults.get()[serializationIndex++] = MessageType::MATCH_STATE;
        serializationResults.get()[serializationIndex++] = (float)true;
        serializationResults.get()[serializationIndex++] = matchTime;

        std::cout << "Size in float number: " << serializationResultsSizeInFloatNumber << std::endl;

        std::string str_ip_address = "10.42.0.255";
        std::cout << "Sending trajectory to IP " << "10.42.0.255" << std::endl;
        message_sender::send_message_udp(serializationResults.get(), serializationResultsSizeInFloatNumber, str_ip_address.c_str());
    }
    else
    {
        std::cout << "Match time is not a number: " << newMatchTime << std::endl;
    }
}

void Viewer::stopMatchButtonClicked()
{
    serializationResultsSizeInFloatNumber = 3;
    serializationResults.reset(new float[serializationResultsSizeInFloatNumber]());

    int serializationIndex = 0;
    serializationResults.get()[serializationIndex++] = MessageType::MATCH_STATE;
    serializationResults.get()[serializationIndex++] = (float)false;
    serializationResults.get()[serializationIndex++] = 0.0;

    std::cout << "Size in float number: " << serializationResultsSizeInFloatNumber << std::endl;

    std::string str_ip_address = recipientIPTextView->get_buffer()->get_text();
    std::cout << "Sending trajectory to IP " << recipientIPTextView->get_buffer()->get_text() << std::endl;
    message_sender::send_message(serializationResults.get(), serializationResultsSizeInFloatNumber, str_ip_address.c_str());
}