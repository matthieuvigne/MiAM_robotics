/// \file MainWindow.h
/// \brief Main window of the GUI.

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <gtkmm.h>

#include <miam_utils/drivers/STSServoDriver.h>

class MainWindow : public Gtk::Window
{
    public:
        MainWindow(STSServoDriver *driver);


    protected:
        // Rescan network to find all servos.
        void rescan();

        // Action callbacks
        void resetPosition(int const& servoNumber);
        void updateEnable(int const& servoNumber);
        void updateTargetPosition(int const& servoNumber);
        void updateTargetVelocity(int const& servoNumber);
        void updateControlMode(int const& servoNumber);
        void updateId(int const& servoNumber);
        void dumpMemory();

        // Update servo readings
        bool updateReadings();

        STSServoDriver* driver_;

        std::vector<int> servoIds_;
        std::vector<Gtk::Label> servoNames_;
        std::vector<Gtk::Label> servoPositions_;
        std::vector<Gtk::Label> servoVelocities_;
        std::vector<Gtk::ComboBoxText> controlModes_;
        std::vector<Gtk::SpinButton> targetPositions_;
        std::vector<Gtk::SpinButton> targetVelocities_;
        std::vector<Gtk::CheckButton> torqueEnabled_;
        std::vector<Gtk::Button> resetButtons_;
        std::vector<Gtk::Button> changeIdButtons_;

        Gtk::Grid grid_;
        Gtk::ScrolledWindow scroll_;
};

#endif