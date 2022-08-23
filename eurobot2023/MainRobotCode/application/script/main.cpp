#include <application/application.hpp>

//--------------------------------------------------------------------------------------------------
// Main routine of the application
//--------------------------------------------------------------------------------------------------

int main(int argc, char* argv[])
{
  Glib::RefPtr<application::RobotApplication> robot_app =
    application::RobotApplication::create();
  return robot_app->run(argc, argv);
}

//--------------------------------------------------------------------------------------------------
