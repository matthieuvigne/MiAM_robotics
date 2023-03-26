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

    // Font size
    auto provider = Gtk::CssProvider::create();
    provider->load_from_data("label {"
            "font-size:22px;"
        "}");
    // get_style_context()->add_provider(TextViewStyle,1);
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


    // TODO bottom widgets
    add(box_);
    // add(box_);

    // // Get the button, and connect it to the callback "buttonClicked"
    // Gtk::Button *button;
    // refGlade->get_widget("button", button);
    // button->signal_clicked().connect(sigc::mem_fun(this, &RobotGUI::buttonClicked));

    // // Get the label, to change its text
    // refGlade->get_widget("clickLabel", clickLabel);
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
    debugLabel_.set_text(robotData.debugStatus);
    std::string const markup = "<span foreground=\"blue\" size=\"x-large\">Score: " +
                            std::to_string(robotData.score) + "</span>";
    scoreLabel_.set_markup(markup);

    if (robotData.state != lastState_)
    {
        // Remove second widget, if needed.
        auto childs = box_.get_children();
        if (childs.size() == 2)
            box_.remove(*childs.at(1));
        if (robotData.state == robotstate::INIT)
        {
            box_.pack_start(debugLabel_);
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

// void RobotGUI::buttonClicked()
// {
//     static int nClick = 0;
//     nClick++;
//     clickLabel->set_text("Number of clicks: " + std::to_string(nClick));
// }

