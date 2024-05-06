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
  Robot robot(main_robot::generateParams(), &strategy, &gui, testMode, noLidar, false);

  if (!robot.getServos()->init("/dev/ttyAMA0", -1))
  {
      std::cout << "Failed to init communication with servos." << std::endl;
      return 0;
  }

  if (!robot.getMPC23008()->init(&RPI_I2C))
  {
      std::cout << "Failed to init communication with MCP IO expander." << std::endl;
      return 0;
  }

  // Instantiate the servo manager
  ServoManager servo_manager;
  ServoManager *servoManager_ = &servo_manager;
  Robot *robot_ = &robot;
  servo_manager.init(&robot, true);
  robot.wait(1.0);

  while (true)
  {
    servo_manager.turnOnMagnets();
    robot.wait(1.0);
    servo_manager.turnOffFrontMagnets();
    robot.wait(1.0);
    servo_manager.turnOffMagnets();
    robot.wait(1.0);
  }

  servo_manager.waitForTurret();

  std::cout << "Turret calibrated" << std::endl;
  robot.wait(1.0);

  GameState gameState;

  servo_manager.setClawPosition(ClawSide::FRONT, ClawPosition::HIGH_POSITION);
  servo_manager.setClawPosition(ClawSide::BACK, ClawPosition::HIGH_POSITION);

  servoManager_->closeClaws(false);
  servoManager_->closeClaws(true);

  // Test: drop plants to pot


  bool isDroppingFront_ = true;
  int dropSign = 1;

  double turretOffset = (isDroppingFront_ ? M_PI : 0);
  int servoOffset = (isDroppingFront_ ? 0 : 3);

  // Drop plants in pots.
  servoManager_->moveTurret(turretOffset - dropSign * 0.2);
  robot_->wait(0.2);
  servoManager_->waitForTurret();
  servoManager_->openClaw(servoOffset + 1, false);
  robot_->wait(0.010);
  servoManager_->openClaw(servoOffset + (dropSign > 0 ? 2 : 0), false);
  robot_->wait(0.4);

  // TODO: drop in third pot

  // Move back

  // Move turret and drop
  servoManager_->moveTurret(turretOffset - dropSign * 0.65);
  robot_->wait(0.2);
  servoManager_->waitForTurret();
  servoManager_->openClaw(servoOffset + (dropSign > 0 ? 0 : 2), false);

  robot_->wait(0.4);
  int nPlants = -servoManager_->updateClawContent(isDroppingFront_, robot_->gameState_);
  std::cout << "[DropPlantsWithPotAction] Dropped " << nPlants << " plants." << std::endl;

while(true) ;;

  // Test claw object detection.
  // while (true)
  // {
  //   servo_manager.closeClaws(true);
  //   servo_manager.closeClaws(false);
  //   robot.wait(1.0);
  //   servo_manager.updateClawContent(true, robot.gameState_);
  //   servo_manager.updateClawContent(false, robot.gameState_);

  //   for (int i = 0; i < 6; i++)
  //     if (robot.gameState_.robotClawContent[i] == ClawContent::EMPTY)
  //       std::cout << "E ";
  //     else
  //       std::cout << "P ";
  //   std::cout << std::endl;
  //   servo_manager.openClaws(true, false);
  //   servo_manager.openClaws(false, false);
  //   robot.wait(1.0);
  // }


  // servo_manager.closeClaws(false);
  robot.wait(2.0);
  servo_manager.closeClaws(true);
  robot.wait(2.0);
  servo_manager.setClawPosition(ClawSide::FRONT, ClawPosition::MEDIUM_POSITION);
  robot.wait(2.0);
  servo_manager.openClaws(true, false);
  robot.wait(0.3);
  servo_manager.closeClaws(true);
  servo_manager.setClawPosition(ClawSide::FRONT, ClawPosition::MEDIUM_POSITION_PLUS);

  //~ servo_manager.moveTurret(-0.2);
  //~ servo_manager.waitForTurret();
  //~ servo_manager.openClaw(6, false);
  //~ robot.wait(0.050);
  //~ servo_manager.openClaw(7, false);

  while (true)
  {
  // servo_manager.moveTurret(0); // Initialisation a faire avec l'interrupteur
  // robot.wait(1.0);
  // servo_manager.waitForTurret();
  // robot.wait(1.0);

  // servo_manager.moveTurret(M_PI_2); // Initialisation a faire avec l'interrupteur
  // robot.wait(1.0);
  // servo_manager.waitForTurret();
  // robot.wait(1.0);
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
