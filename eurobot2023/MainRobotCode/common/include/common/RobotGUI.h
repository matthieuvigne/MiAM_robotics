/// \file RobotGUI.h
/// \brief Graphical interface for the robot

#ifndef ROBOT_GUI_H
    #define ROBOT_GUI_H

        #include <gtkmm.h>
        #include "common/Types.h"


        class TableDrawing : public Gtk::DrawingArea
        {
            public:
                TableDrawing() {};
                ~TableDrawing() {};

                RobotGUIData robotData_;

            protected:
                bool on_draw(Cairo::RefPtr<Cairo::Context> const& cr) override;
        };

        class RobotGUI : public Gtk::Window
        {
            public:
                RobotGUI();
                virtual ~RobotGUI();

                void update(RobotGUIData const& robotData);

                bool getIsPlayingRightSide();
                bool getIsTopStrategy();

            protected:
                void sideButtonClicked();
                void strategyButtonClicked();

            private:
                bool doUpdate();

                void drawTable(Cairo::RefPtr<Cairo::Context> const& cr, int width, int height);

                Gtk::Box box_;
                Gtk::Box topBox_;

                Gtk::Label labelBattery_;
                Gtk::Label matchTime_;
                Gtk::Label labelState_;

                Gtk::Label debugLabel_;
                Gtk::Label scoreLabel_;

                Gtk::Button sideButton_;
                Gtk::Button strategyButton_;

                TableDrawing drawingArea_;


                RobotGUIData robotData_;
                bool isPlayingRightSide_ = false;
                bool isStrategyTop_ = false;

                robotstate lastState_ = robotstate::MATCH_DONE;
                std::mutex mutex_;
        };


 #endif
