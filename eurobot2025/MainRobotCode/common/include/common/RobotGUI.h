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

                miam::RobotPosition getStartPosition();

                /// @brief Block motor during init phase
                bool getBlockMotors();

                /// @brief Detect borders to setup initial position
                bool getAskedDetectBorders();

            protected:
                void sideButtonClicked();
                void startPositionButtonClicked();
                void blockMotorsButtonClicked();
                void detectBordersClicked();

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
                Gtk::Label actionNameLabel_;

                Gtk::Button sideButton_;
                Gtk::Button startPositionButton_;
                Gtk::Button blockMotorsButton_;
                Gtk::Button detectBordersButton_;

                bool areMotorsBlocked_ = false;
                bool askedDetectBorders_ = false;

                TableDrawing drawingArea_;


                RobotGUIData robotData_;
                int startPositionIdx_ = 0;  // Index of start position in START_POSITIONS vector.
                bool isPlayingRightSide_ = false;

                robotstate lastState_ = robotstate::MATCH_DONE;
                std::mutex mutex_;
        };


 #endif
