#include "MainWindow.h"
#include <iostream>

MainWindow::MainWindow(STSServoDriver *driver):
    driver_(driver)
{
    this->add(scroll_);
    grid_.set_margin_start(10);
    grid_.set_margin_end(10);
    grid_.set_margin_left(10);
    grid_.set_margin_right(10);
    grid_.set_row_spacing(10);
    grid_.set_column_spacing(10);
    grid_.set_size_request(800, 600);
    scroll_.set_size_request(800, 600);
    scroll_.add(grid_);

    Gtk::Label *header;
    header = new Gtk::Label("Servo\nnumber");
    grid_.attach(*header, 0, 0, 1, 1);
    header = new Gtk::Label("Position");
    grid_.attach(*header, 1, 0, 1, 1);
    header = new Gtk::Label("Speed");
    grid_.attach(*header, 2, 0, 1, 1);
    header = new Gtk::Label("Enable");
    grid_.attach(*header, 3, 0, 1, 1);
    header = new Gtk::Label("Control mode");
    grid_.attach(*header, 4, 0, 1, 1);
    header = new Gtk::Label("Target position");
    grid_.attach(*header, 5, 0, 1, 1);
    header = new Gtk::Label("Target velocity");
    grid_.attach(*header, 6, 0, 1, 1);
    header = new Gtk::Label("Reset position");
    grid_.attach(*header, 7, 0, 1, 1);
    Gtk::Button* button = new Gtk::Button("Rescan");
    grid_.attach(*button, 8, 0, 1, 1);
    button->signal_clicked().connect(sigc::mem_fun(this, &MainWindow::rescan));

    rescan();
    Glib::signal_timeout().connect(sigc::mem_fun(*this,&MainWindow::updateReadings), 100);
}

void MainWindow::rescan()
{
    std::vector<unsigned char> ids = driver_->detectServos();

    servoIds_.clear();
    servoNames_.clear();
    servoPositions_.clear();
    servoVelocities_.clear();
    controlModes_.clear();
    targetPositions_.clear();
    targetVelocities_.clear();
    torqueEnabled_.clear();
    resetButtons_.clear();
    changeIdButtons_.clear();

    int servoNumber = 0;
    for (auto i : ids)
    {
        servoIds_.push_back(i);
        servoNames_.push_back(Gtk::Label(std::to_string(i)));
        servoPositions_.push_back(Gtk::Label(std::to_string(driver_->getCurrentPosition(i))));
        servoVelocities_.push_back(Gtk::Label(std::to_string(driver_->getCurrentSpeed(i))));

        torqueEnabled_.push_back(Gtk::CheckButton());
        torqueEnabled_.back().set_halign(Gtk::ALIGN_CENTER);
        torqueEnabled_.back().signal_toggled().connect(sigc::bind(sigc::mem_fun(this, &MainWindow::updateEnable), servoNumber));

        resetButtons_.push_back(Gtk::Button("Set center"));
        resetButtons_.back().set_halign(Gtk::ALIGN_CENTER);
        resetButtons_.back().signal_clicked().connect(sigc::bind(sigc::mem_fun(this, &MainWindow::resetPosition), servoNumber));

        changeIdButtons_.push_back(Gtk::Button("Change Id"));
        changeIdButtons_.back().set_halign(Gtk::ALIGN_CENTER);
        changeIdButtons_.back().signal_clicked().connect(sigc::bind(sigc::mem_fun(this, &MainWindow::updateId), servoNumber));

        controlModes_.push_back(Gtk::ComboBoxText());
        controlModes_.back().append("Position");
        controlModes_.back().append("Velocity");
        controlModes_.back().append("Step");
        controlModes_.back().set_active(0);
        controlModes_.back().signal_changed().connect(sigc::bind(sigc::mem_fun(this, &MainWindow::updateControlMode), servoNumber));

        targetPositions_.push_back(Gtk::SpinButton());
        targetPositions_.back().set_numeric(true);
        targetPositions_.back().set_range(-32766, 32766);
        targetPositions_.back().set_increments(10, 100);
        targetPositions_.back().set_width_chars(6);
        targetPositions_.back().signal_value_changed().connect(sigc::bind(sigc::mem_fun(this, &MainWindow::updateTargetPosition), servoNumber));

        targetVelocities_.push_back(Gtk::SpinButton());
        targetVelocities_.back().set_numeric(true);
        targetVelocities_.back().set_range(-32766, 32766);
        targetVelocities_.back().set_increments(100, 1000);
        targetVelocities_.back().set_width_chars(6);
        targetVelocities_.back().signal_value_changed().connect(sigc::bind(sigc::mem_fun(this, &MainWindow::updateTargetVelocity), servoNumber));

        servoNumber += 1;
        grid_.attach(servoNames_.back(), 0, servoNumber, 1, 1);
        grid_.attach(servoPositions_.back(), 1, servoNumber, 1, 1);
        grid_.attach(servoVelocities_.back(), 2, servoNumber, 1, 1);
        grid_.attach(torqueEnabled_.back(), 3, servoNumber, 1, 1);
        grid_.attach(controlModes_.back(), 4, servoNumber, 1, 1);
        grid_.attach(targetPositions_.back(), 5, servoNumber, 1, 1);
        grid_.attach(targetVelocities_.back(), 6, servoNumber, 1, 1);
        grid_.attach(resetButtons_.back(), 7, servoNumber, 1, 1);
        grid_.attach(changeIdButtons_.back(), 8, servoNumber, 1, 1);
    }
    show_all();
}


bool MainWindow::updateReadings()
{
    for (unsigned int i = 0; i < servoIds_.size(); i++)
    {
        servoPositions_[i].set_text(std::to_string(driver_->getCurrentPosition(servoIds_[i])));
        servoVelocities_[i].set_text(std::to_string(driver_->getCurrentSpeed(servoIds_[i])));
    }
    return true;
}



void MainWindow::updateId(int const& servoNumber)
{
    // driver_->setTargetPosition(servoIds_[servoNumber], targetPositions_[servoNumber].get_value_as_int());
    Gtk::Dialog dialog("ChangeId", *this);

    dialog.add_button("Cancel", -1);
    dialog.add_button("Change Id", 1);

    Gtk::Box* box = dialog.get_content_area();
    box->set_margin_start(10);
    box->set_margin_end(10);
    box->set_margin_left(10);
    box->set_margin_right(10);
    box->set_spacing(10);
    Gtk::Label text("Set new id:");
    box->pack_start(text);

    Gtk::SpinButton button;
    button.set_numeric(true);
    button.set_range(0, 253);
    button.set_increments(1, 10);
    button.set_width_chars(4);
    box->pack_start(button);
    box->show_all();

    int const responseId = dialog.run();
    if (responseId > 0)
    {
        int const newId = button.get_value_as_int();
        bool const wasIdChanged = driver_->setId(servoIds_[servoNumber], newId);
        if (wasIdChanged)
        {
            servoIds_[servoNumber] = newId;
            servoNames_[servoNumber].set_text(std::to_string(newId));
        }
    }

}


void MainWindow::resetPosition(int const& servoNumber)
{
    driver_->resetPositionAsCenter(servoIds_[servoNumber]);
}


void MainWindow::updateEnable(int const& servoNumber)
{
    if (torqueEnabled_[servoNumber].get_active())
    {
        targetPositions_[servoNumber].set_value(driver_->getCurrentPosition(servoIds_[servoNumber]));
        targetVelocities_[servoNumber].set_value(0);
        updateTargetPosition(servoNumber);
        updateTargetVelocity(servoNumber);
    }
    driver_->disable(servoIds_[servoNumber], !torqueEnabled_[servoNumber].get_active());
}


void MainWindow::updateTargetPosition(int const& servoNumber)
{
    if (torqueEnabled_[servoNumber].get_active())
        driver_->setTargetPosition(servoIds_[servoNumber], targetPositions_[servoNumber].get_value_as_int());
}


void MainWindow::updateTargetVelocity(int const& servoNumber)
{
    if (torqueEnabled_[servoNumber].get_active())
        driver_->setTargetVelocity(servoIds_[servoNumber], targetVelocities_[servoNumber].get_value_as_int());
}


void MainWindow::updateControlMode(int const& servoNumber)
{
    Glib::ustring const mode = controlModes_[servoNumber].get_active_text();
    if (mode == "Position")
    {
        driver_->setMode(servoIds_[servoNumber], STS::Mode::POSITION);
        targetPositions_[servoNumber].set_value(driver_->getCurrentPosition(servoIds_[servoNumber]));
    }
    else if (mode == "Velocity")
    {
        driver_->setMode(servoIds_[servoNumber], STS::Mode::VELOCITY);
        targetVelocities_[servoNumber].set_value(0);
    }
    else
    {
        driver_->setMode(servoIds_[servoNumber], STS::Mode::STEP);
        targetPositions_[servoNumber].set_value(0);
    }
}