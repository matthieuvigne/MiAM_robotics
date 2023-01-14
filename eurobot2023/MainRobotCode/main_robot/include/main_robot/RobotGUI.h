/// \file RobotGUI.h
/// \brief Graphical interface for the robot

#ifndef ROBOT_GUI_H
    #define ROBOT_GUI_H

        #include <gtkmm.h>
        #include "common/RobotInterface.h"

        namespace main_robot
        {
        class RobotGUI : public Gtk::Window
        {
            public:
                RobotGUI(BaseObjectType* cobject,
                         const Glib::RefPtr<Gtk::Builder>& refGlade,
                         RobotInterface *robot);
                virtual ~RobotGUI();

            private:
                void buttonClicked();

                RobotInterface *robot_;
                Gtk::Label *clickLabel;
        };

        void startRobotGUI(RobotInterface *robot);
        }
 #endif
