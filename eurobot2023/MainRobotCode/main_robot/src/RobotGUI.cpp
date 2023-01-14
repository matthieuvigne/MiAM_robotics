/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <miam_utils/raspberry_pi/RPiGPIO.h>
#include "main_robot/RobotGUI.h"

#include <unistd.h>

namespace main_robot
{
RobotGUI::RobotGUI(BaseObjectType* cobject,
                   const Glib::RefPtr<Gtk::Builder>& refGlade,
                   RobotInterface *robot):
    Gtk::Window(cobject),
    robot_(robot)
{
    // Get the button, and connect it to the callback "buttonClicked"
    Gtk::Button *button;
    refGlade->get_widget("button", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &RobotGUI::buttonClicked));

    // Get the label, to change its text
    refGlade->get_widget("clickLabel", clickLabel);

    // Show window
    show_all();
}

RobotGUI::~RobotGUI()
{

}

void RobotGUI::buttonClicked()
{
    static int nClick = 0;
    nClick++;
    clickLabel->set_text("Number of clicks: " + std::to_string(nClick));
}

void startRobotGUI(RobotInterface *robot)
{
    // Create GTK app
    Glib::RefPtr<Gtk::Application> app =  Gtk::Application::create();

    // Create GTK builder
    auto refBuilder = Gtk::Builder::create();
    try
    {
        refBuilder->add_from_file("/miam_workspace/src/MiAM_robotics/eurobot2023/MainRobotCode/main_robot/robot_gui.glade");
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

    // Load GUI
    RobotGUI *robotGUI;
    refBuilder->get_widget_derived("mainWindow", robotGUI,  robot);
    app->run(*robotGUI);
}
}