#include "DemoGUI.h"

#include <unistd.h>
#include <iomanip>
#include <iostream>
#include <signal.h>

DemoGUI::DemoGUI()
{
    set_size_request(300, 300);
    fullscreen();

    // Styling
    auto provider = Gtk::CssProvider::create();
    provider->load_from_data("label {font-size:22px;}"
            "#blue {background:#5485a4;}"
            "#yellow {background:#e0bc48;}"
            "#red {background:#ff0000;}"
            "#green {background:#00B000;}"
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



    status_ = Gtk::Label("Status");
    status_.set_hexpand(true);

    nextButton_ = Gtk::Button("Next");
    nextButton_.signal_clicked().connect(sigc::mem_fun(*this, &DemoGUI::nextClicked));

    quitButton_ = Gtk::Button("Quit");
    quitButton_.set_name("red");
    quitButton_.get_child()->set_name("button_text");
    quitButton_.signal_clicked().connect(sigc::mem_fun(*this, &DemoGUI::quit));

    box_.pack_start(status_);
    box_.pack_start(nextButton_);
    box_.pack_start(quitButton_);
    add(box_);
    Glib::signal_timeout().connect(sigc::mem_fun(*this, &DemoGUI::doUpdate), 50);
}

DemoGUI::~DemoGUI()
{

}

bool DemoGUI::doUpdate()
{
    if (updateNeeded_)
    {
        status_.set_label(statusText_);
        nextButton_.set_label(buttonText_);
        updateNeeded_ = false;
    }
    return true;
}

void DemoGUI::nextClicked()
{
    wasClicked_ = true;
}

void DemoGUI::quit()
{
    ::raise(SIGINT);
}

void DemoGUI::set_label(std::string const& status, std::string const& buttonLabel)
{
    statusText_ = status;
    buttonText_ = buttonLabel;
    updateNeeded_ = true;
}