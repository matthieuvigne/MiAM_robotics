#include "Robot.h"
#include "common/RobotGUI.h"
#include "main_robot/Parameters.h"
#include "main_robot/Strategy.h"
#include "main_robot/ServoManager.h"

#include <iostream>
#include <thread>
#include <filesystem>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/resource.h>

#include <filesystem>
#include <fstream>

int main(int argc, char* argv[])
{
  // --------------------------
  // Servos 2, 3, 4, 10, 11, 12
  // --------------------------
  // 2:
  // 3:
  // 4:
  // 10: tourelle verticale
  // 11: lever les pinces (pas au bout)
  // 12: lever les pinces (au bout)

  // Position basse
  // 11: 1625
  // 12: 3432

  // Position médiane
  // ----------------
  // 11: 2335
  // 12: 2702

  // Position haute
  // 11: 2695
  // 12: 2352

  // Position toute droite
  // 10: 702
  // Position +90°
  // 10: 3362


  // Try to communicate with servo.
  RPi_enablePorts();

  // Create objects
  Glib::RefPtr<Gtk::Application> app =  Gtk::Application::create();
  RobotGUI gui;
  bool testMode = true;
  bool noLidar = true;
  main_robot::Strategy strategy;
  Robot robot(main_robot::generateParams(), &strategy, &gui, testMode, noLidar);

  if (!robot.getServos()->init("/dev/ttyAMA0", -1))
  {
      std::cout << "Failed to init communication with servos." << std::endl;
      return 0;
  }

  // Instantiate the servo manager
  ServoManager servo_manager;
  servo_manager.init(&robot);

  servo_manager.waitForTurret();

  std::cout << "Turret calibrated" << std::endl;
  robot.wait(1.0);

  // Sequence
  while (true)
  {
  servo_manager.moveTurret(0); // Initialisation a faire avec l'interrupteur
  robot.wait(1.0);
  servo_manager.waitForTurret();
  robot.wait(1.0);

  servo_manager.moveTurret(M_PI_2); // Initialisation a faire avec l'interrupteur
  robot.wait(1.0);
  servo_manager.waitForTurret();
  robot.wait(1.0);
  }

  servo_manager.waitForTurret();
  while (true)
    usleep(1000000);

  //~ driver.setMode(10, STS::Mode::STEP);
  servo_manager.setClawPosition(ClawSide::FRONT, ClawPosition::LOW_POSITION);
  usleep(1000000);
  servo_manager.openClaws(true);
  usleep(4000000);
  servo_manager.closeClaws(true);
  usleep(1000000);
  servo_manager.setClawPosition(ClawSide::FRONT, ClawPosition::HIGH_POSITION);
  usleep(1000000);
  servo_manager.moveTurret(90);
  usleep(1000000);
  servo_manager.setClawPosition(ClawSide::FRONT, ClawPosition::MEDIUM_POSITION);
  usleep(1000000);
  servo_manager.openClaws(true);
  usleep(2000000);
  servo_manager.setClawPosition(ClawSide::FRONT, ClawPosition::HIGH_POSITION);
  usleep(1000000);
  servo_manager.moveTurret(0);
  usleep(1000000);
  robot.getServos()->disable(0xFE);

  // Get and display the current position of the turret
  //~ double const turret_position = servo_manager.getTurretPosition();
  //~ std::cout << "Current turret position : " << turret_position << std::endl;
  //~ servo_manager.moveTurret(0);
  //~ usleep(1000000);
  //~ servo_manager.setClawPosition(ClawPosition::LOW_POSITION);
  //~ usleep(1000000);
  //~ servo_manager.openClaws(true);
  //~ usleep(1000000);
  //~ servo_manager.closeClaws(true);
  //~ usleep(1000000);
  //~ servo_manager.openClaws(true);
  //~ usleep(1000000);
  //~ servo_manager.moveTurret(-20);
  //~ usleep(1000000);
  //~ servo_manager.setClawPosition(ClawPosition::MEDIUM_POSITION);
  //~ usleep(1000000);
  //~ servo_manager.setClawPosition(ClawPosition::HIGH_POSITION);
  //~ usleep(1000000);
  //~ servo_manager.moveTurret(30);
  //~ usleep(1000000);
  //~ servo_manager.moveTurret(0);
  //~ usleep(1000000);
  //~ servo_manager.moveTurret(90);
  //~ usleep(2000000);
  //~ servo_manager.moveTurret(180);
  //~ usleep(2000000);
  //~ driver.disable(0xFE);

  return EXIT_SUCCESS;
}
