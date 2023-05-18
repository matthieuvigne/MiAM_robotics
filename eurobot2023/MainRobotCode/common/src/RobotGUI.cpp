/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <miam_utils/raspberry_pi/RPiGPIO.h>
#include "common/RobotGUI.h"

#include <unistd.h>
#include <iomanip>
#include <iostream>

RobotGUI::RobotGUI()
{
    set_size_request(300, 300);
    fullscreen();

    // Styling
    auto provider = Gtk::CssProvider::create();
    provider->load_from_data("label {font-size:22px;}"
            "#blue {background:#005B8C;}"
            "#green {background:#007749;}"
            "#button_text {color:#FFFFFF;}"
            "#score {font-size:40; color:#0000FF;}");
    Gtk::StyleContext::add_provider_for_screen(get_screen(), provider, GTK_STYLE_PROVIDER_PRIORITY_USER);

    box_ = Gtk::Box(Gtk::Orientation::ORIENTATION_VERTICAL);
    box_.set_hexpand(true);
    box_.set_vexpand(true);
    box_.set_spacing(5);
    box_.set_margin_start(5);
    box_.set_margin_end(5);
    box_.set_margin_top(5);
    box_.set_margin_bottom(5);

    topBox_ = Gtk::Box(Gtk::Orientation::ORIENTATION_HORIZONTAL);
    topBox_.set_spacing(5);

    labelBattery_ = Gtk::Label("Battery");
    labelBattery_.set_hexpand(true);
    topBox_.pack_start(labelBattery_);
    matchTime_ = Gtk::Label("State");
    matchTime_.set_hexpand(true);
    topBox_.pack_start(matchTime_);
    labelState_ = Gtk::Label("State");
    labelState_.set_hexpand(true);
    topBox_.pack_start(labelState_);
    topBox_.set_valign(Gtk::Align::ALIGN_START);
    box_.pack_start(topBox_);

    sideButton_ = Gtk::Button("Bleu");
    sideButton_.signal_clicked().connect(sigc::mem_fun(*this, &RobotGUI::sideButtonClicked));

    scoreLabel_.set_name("score");
    add(box_);

    drawingArea_.set_size_request(300, 300);
    drawingArea_.set_hexpand(true);
    drawingArea_.set_vexpand(true);


    // Refresh at 20Hz.
    Glib::signal_timeout().connect(sigc::mem_fun(*this, &RobotGUI::doUpdate), 50);
}

RobotGUI::~RobotGUI()
{

}

bool RobotGUI::getIsPlayingRightSide()
{
    return isPlayingRightSide_;
}

bool RobotGUI::doUpdate()
{
    mutex_.lock();
    RobotGUIData robotData = robotData_;
    mutex_.unlock();
    drawingArea_.robotData_ = robotData;

    std::stringstream stream;
    stream << "Battery: " << std::fixed << std::setprecision(1) << robotData.batteryVoltage<< "V";
    labelBattery_.set_text(stream.str());
    stream.str(std::string());
    stream << "Time: " << std::fixed << std::setprecision(1) << robotData.currentMatchTime<< "s";
    matchTime_.set_text(stream.str());
    labelState_.set_text(robotStateNames[static_cast<int>(robotData.state)]);

    // Update bottom part, if needed
    if (robotData.state == robotstate::UNDERVOLTAGE)
        debugLabel_.set_text("Undervoltage, replace battery!");
    else
        debugLabel_.set_text(robotData.debugStatus);
    scoreLabel_.set_text("Score: " + std::to_string(robotData.score));
    if (isPlayingRightSide_)
    {
        sideButton_.set_label("Vert");
        sideButton_.set_name("green");
        sideButton_.get_child()->set_name("button_text");
    }
    else
    {
        sideButton_.set_label("Bleu");
        sideButton_.set_name("blue");
        sideButton_.get_child()->set_name("button_text");
    }

    if (robotData.state == robotstate::MATCH)
        drawingArea_.queue_draw();
    if (robotData.state != lastState_)
    {
        // Remove second widget, if needed.
        auto childs = box_.get_children();
        if (childs.size() == 2)
            box_.remove(*childs.at(1));
        if (robotData.state == robotstate::INIT || robotData.state == robotstate::UNDERVOLTAGE)
        {
            box_.pack_start(debugLabel_);
        }
        if (robotData.state == robotstate::WAITING_FOR_START)
        {
            box_.pack_start(sideButton_);
            box_.pack_start(drawingArea_);
        }
        if (robotData.state == robotstate::MATCH)
        {
            box_.pack_start(drawingArea_);
            box_.pack_start(scoreLabel_);
        }
        if (robotData.state == robotstate::MATCH_DONE)
        {
            box_.pack_start(scoreLabel_);
        }
        show_all();
        lastState_ = robotData.state;
    }

    return true;
}

void RobotGUI::update(RobotGUIData const& robotData)
{
    mutex_.lock();
    robotData_ = robotData;
    mutex_.unlock();
}

void RobotGUI::sideButtonClicked()
{
    isPlayingRightSide_ = !isPlayingRightSide_;
    doUpdate();
}

bool TableDrawing::on_draw(Cairo::RefPtr<Cairo::Context> const& cr)
{
    // Real size
    double const TABLE_WIDTH = 2000.0;
    double const TABLE_HEIGHT = 3000.0;
    double const MARGIN = 300;

    // Scale
    double scaling = std::min(get_width() / (TABLE_WIDTH + 2 * MARGIN), get_height() / (TABLE_HEIGHT + 2 * MARGIN));

    // Draw
    cr->set_line_width(50.0 * scaling);
    cr->set_source_rgb(0, 0, 0);

    cr->line_to(TABLE_WIDTH * scaling, -TABLE_HEIGHT * scaling);


    // Move to center
    cr->translate(get_width() / 2.0, get_height() / 2.0);
    cr->translate((-TABLE_WIDTH / 2.0) * scaling, (TABLE_HEIGHT / 2.0) * scaling);

    cr->move_to(0, 0);
    cr->line_to(TABLE_WIDTH * scaling, 0);
    cr->line_to(TABLE_WIDTH * scaling, -TABLE_HEIGHT * scaling);
    cr->line_to(0, -TABLE_HEIGHT * scaling);
    cr->close_path();
    cr->stroke();

    // Draw robot
    cr->save();
    cr->translate(scaling * robotData_.currentPosition.x, - scaling * robotData_.currentPosition.y);
    cr->rotate(-robotData_.currentPosition.theta);
    cr->set_source_rgb(0, 0, 1);
    cr->move_to(-90 * scaling, -110 * scaling);
    cr->line_to(90 * scaling, 0);
    cr->line_to(-90 * scaling, 110 * scaling);
    cr->close_path();
    cr->fill();
    cr->restore();

    // Draw obstacles
    for (auto p : robotData_.detectedObstacles)
    {
        cr->save();
        cr->translate(scaling * p.x, - scaling * p.y);
        cr->arc(0, 0, 80 * scaling, 0, 2 * M_PI);
        if (p.theta > M_PI_2)
            cr->set_source_rgb(1, 0.5, 0);
        else
            cr->set_source_rgb(1, 0, 0);
        cr->fill();
        cr->restore();
    }

    return true;
}

