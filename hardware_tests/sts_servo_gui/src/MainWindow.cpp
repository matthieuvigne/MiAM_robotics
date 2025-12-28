#include "MainWindow.h"
#include <iostream>
#include <iomanip>

struct PrintReg{
    unsigned char addr;
    std::string name;
    bool twoBytes;
};


PrintReg registers[] = {
    PrintReg({STS::registers::FIRMWARE_MAJOR,           "Firmware maj.", false}),
    PrintReg({STS::registers::FIRMWARE_MINOR,           "Firmware min.", false}),
    PrintReg({STS::registers::SERVO_MAJOR,              "Servo major", false}),
    PrintReg({STS::registers::SERVO_MINOR,              "Servo minor", false}),
    PrintReg({STS::registers::ID,                       "Id", false}),
    PrintReg({STS::registers::BAUDRATE,                 "Baudrate",   false}),
    PrintReg({STS::registers::RESPONSE_DELAY,           "Response delay", false}),
    PrintReg({STS::registers::RESPONSE_STATUS_LEVEL,    "Response staus level",  false}),
    PrintReg({STS::registers::MINIMUM_ANGLE,            "Min angle",  true}),
    PrintReg({STS::registers::MAXIMUM_ANGLE,            "Max angle",  true}),
    PrintReg({STS::registers::MAXIMUM_TEMPERATURE,      "Max temp.",false}),
    PrintReg({STS::registers::MAXIMUM_VOLTAGE,          "Max voltage",false}),
    PrintReg({STS::registers::MINIMUM_VOLTAGE,          "Min voltage",false}),
    PrintReg({STS::registers::MAXIMUM_TORQUE,           "Max torque", true}),
    PrintReg({STS::registers::UNLOADING_CONDITION,      "Unload. cond.",false}),
    PrintReg({STS::registers::LED_ALARM_CONDITION,      "Led alarm",false}),
    PrintReg({STS::registers::POS_PROPORTIONAL_GAIN,    "Pos Kp",  false}),
    PrintReg({STS::registers::POS_DERIVATIVE_GAIN,      "Pos Kd",false}),
    PrintReg({STS::registers::POS_INTEGRAL_GAIN,        "Pos Ki",  false}),
    PrintReg({STS::registers::MINIMUM_STARTUP_FORCE,    "Min. start force",  true}),
    PrintReg({STS::registers::CK_INSENSITIVE_AREA,      "CK insen. area",false}),
    PrintReg({STS::registers::CCK_INSENSITIVE_AREA,     "CCK insen. area",   false}),
    PrintReg({STS::registers::CURRENT_PROTECTION_TH,    "Current prot. th.",  true}),
    PrintReg({STS::registers::ANGULAR_RESOLUTION,       "Ang. res.", false}),
    PrintReg({STS::registers::POSITION_CORRECTION,      "Pos corr", true}),
    PrintReg({STS::registers::OPERATION_MODE,           "Operation mode", false}),
    PrintReg({STS::registers::TORQUE_PROTECTION_TH,     "Torque prot. th.",   false}),
    PrintReg({STS::registers::TORQUE_PROTECTION_TIME,   "Torque prot. time", false}),
    PrintReg({STS::registers::OVERLOAD_TORQUE,          "Overload torque",false}),
    PrintReg({STS::registers::SPEED_PROPORTIONAL_GAIN,  "Vel Kp",false}),
    PrintReg({STS::registers::OVERCURRENT_TIME,         "Overcurrent time",   false}),
    PrintReg({STS::registers::SPEED_INTEGRAL_GAIN,      "Vel Ki",false}),
    PrintReg({STS::registers::TORQUE_SWITCH,            "Torque switch",  false}),
    PrintReg({STS::registers::TARGET_ACCELERATION,      "Target accel",false}),
    PrintReg({STS::registers::TARGET_POSITION,          "Target pos", true}),
    PrintReg({STS::registers::RUNNING_TIME,             "Running time",   true}),
    PrintReg({STS::registers::RUNNING_SPEED,            "Running speed",   true}),
    PrintReg({STS::registers::TORQUE_LIMIT,             "Torque limit",   true}),
    PrintReg({STS::registers::WRITE_LOCK,               "Write lock", false}),
    PrintReg({STS::registers::CURRENT_POSITION,         "Current pos",   true}),
    PrintReg({STS::registers::CURRENT_SPEED,            "Current speed",  true}),
    PrintReg({STS::registers::CURRENT_DRIVE_VOLTAGE,    "Current drive voltage",  false}),
    PrintReg({STS::registers::CURRENT_VOLTAGE,          "Current voltage",false}),
    PrintReg({STS::registers::CURRENT_TEMPERATURE,      "Current temperature",false}),
    PrintReg({STS::registers::ASYNCHRONOUS_WRITE_ST,    "Async. write. st",  false}),
    PrintReg({STS::registers::STATUS,                   "Status", false}),
    PrintReg({STS::registers::MOVING_STATUS,            "Moving",  false}),
    PrintReg({STS::registers::CURRENT_CURRENT,          "Current current",true})
};


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

    button = new Gtk::Button("Dump memory");
    grid_.attach(*button, 9, 0, 1, 1);
    button->signal_clicked().connect(sigc::mem_fun(this, &MainWindow::dumpMemory));

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
        servoVelocities_.push_back(Gtk::Label(std::to_string(driver_->getCurrentVelocity(i))));

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
    driver_->setTorqueLimit(0x01, 0.1);
    for (unsigned int i = 0; i < servoIds_.size(); i++)
    {
        servoPositions_[i].set_text(std::to_string(driver_->getCurrentPosition(servoIds_[i])));
        servoVelocities_[i].set_text(std::to_string(driver_->getCurrentVelocity(servoIds_[i])));
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



void MainWindow::dumpMemory()
{
    std::cout << "\nMemory dump:" << std::endl;
    for (auto const& reg : registers)
    {
        std::cout << std::setw(30) << reg.name;
        for (auto const& i : servoIds_)
        {
            int value;
            if (reg.twoBytes)
                value = static_cast<int>(driver_->readTwoBytesRegister(i, reg.addr));
            else
                value = static_cast<int>(driver_->readRegister(i, reg.addr));
            std::cout << std::setw(7) << value;
        }
        std::cout << std::endl;
    }
};


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
        usleep(5000);
        targetPositions_[servoNumber].set_value(driver_->getCurrentPosition(servoIds_[servoNumber]));
        updateTargetPosition(servoNumber);
    }
    else if (mode == "Velocity")
    {
        targetVelocities_[servoNumber].set_value(0);
        updateTargetVelocity(servoNumber);
        driver_->setMode(servoIds_[servoNumber], STS::Mode::VELOCITY);
    }
    else
    {
        targetPositions_[servoNumber].set_value(0);
        updateTargetPosition(servoNumber);
        driver_->setMode(servoIds_[servoNumber], STS::Mode::STEP);
    }
}