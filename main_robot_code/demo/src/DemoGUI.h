#ifndef DEMO_GUI_H
    #define DEMO_GUI_H

        #include <gtkmm.h>


        class DemoGUI : public Gtk::Window
        {
            public:
                DemoGUI();
                virtual ~DemoGUI();

                /// @brief Disable VLX sensor
                void set_label(std::string const& status, std::string const& buttonLabel);

                bool wasClicked_ = false;

            private:
                bool doUpdate();
                void nextClicked();
                void quit();
                Gtk::Box box_;

                Gtk::Label status_;
                Gtk::Button nextButton_;
                Gtk::Button quitButton_;


                std::string statusText_;
                std::string buttonText_;
                bool updateNeeded_ = false;

        };


 #endif
