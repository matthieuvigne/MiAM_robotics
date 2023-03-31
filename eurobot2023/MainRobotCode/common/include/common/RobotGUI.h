/// \file RobotGUI.h
/// \brief Graphical interface for the robot

#ifndef ROBOT_GUI_H
    #define ROBOT_GUI_H

        #include <gtkmm.h>
        #include "common/Types.h"

        class RobotGUI : public Gtk::Window
        {
            public:
                RobotGUI();
                virtual ~RobotGUI();

                void update(RobotGUIData const& robotData);

                bool getIsPlayingRightSide();

            protected:
                void sideButtonClicked();

            private:
                bool doUpdate();


                Gtk::Box box_;
                Gtk::Box topBox_;

                Gtk::Label labelBattery_;
                Gtk::Label matchTime_;
                Gtk::Label labelState_;

                Gtk::Label debugLabel_;
                Gtk::Label scoreLabel_;

                Gtk::Button sideButton_;

                RobotGUIData robotData_;
                bool isPlayingRightSide_ = false;

                robotstate lastState_ = robotstate::MATCH_DONE;
                std::mutex mutex_;
        };


 #endif
