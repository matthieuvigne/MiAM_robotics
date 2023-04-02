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
        }
        if (robotData.state == robotstate::MATCH)
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

