/// \file Strategy.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef MAIN_ROBOT_STRATEGY_H
#define MAIN_ROBOT_STRATEGY_H

#include "common/RobotInterface.h"
#include "common/ServoHandler.h"
#include "common/AbstractAction.h"
#include "common/AbstractStrategy.h"
#include "common/MotionPlanner.h"
#include "main_robot/ArmAction.h"

#include <queue>
#include <mutex>
#include <array>

namespace main_robot
{
  namespace arm{

    // Reference positions of the cakes
    double const CAKES_FRONT_DISTANCE = 0.120; //0.115; [REMOVE]
    double const CAKES_SIDE_DISTANCE = 0.130; //0.060; [REMOVE]

    // Reference angles for left arm
    double constexpr RAD = M_PI/180.;
    double const MIDDLE_PILE_ANGLE = -30.0*RAD; //-35.0*RAD; //-0.27;
    double const FRONT_PILE_ANGLE = 15.5*RAD;
    double const SIDE_PILE_ANGLE = 95*RAD;

    // Reference layer heights
    double const GROUND_HEIGHT = -0.195;
    double const PILE_CLEAR_HEIGHT = GROUND_HEIGHT + 0.085;
    double const LAYER_HEIGHT = 0.02;
    double const LAYER_MOVEMENT_CLEARANCE = 0.025;

    static ArmPosition DISTRIBUTOR_CHERRY(125, -0.55, -130);
    ArmPosition servoAnglesToArmPosition(double thetaH, double thetaV1, double thetaV2, double thetaV3);

  } // namespace arm

  // Reference values
  static int const ABS = 0;
  static int const REL = 1;

  // Reference indexes
  static int const RIGHT_ARM = 10;
  static int const LEFT_ARM = 20;
  static int const PUMP_RIGHT = 12;
  static int const PUMP_LEFT = 13;
  static int const VALVE_RIGHT = 24;
  static int const VALVE_LEFT = 26;
  
  class Strategy : public AbstractStrategy
  {
    public:

      // Constructor
      Strategy();

      // Called before the start of the match, to setup the robot.
      bool setup(RobotInterface *robot) override;

      // Code executed when shutting down the robot
      void shutdown() override;

      // The actual match code, which runs in its own thread.
      void match() override;

      bool moveArm(double r, double angle, double z);

      void periodicAction() override;

    private:

      void funnyAction();
      ArmPosition left_arm_position_;
      ArmPosition right_arm_position_;
      std::queue<std::shared_ptr<ArmAction > > left_arm_positions;
      std::queue<std::shared_ptr<ArmAction > > right_arm_positions;
      std::mutex pileLock;
      std::array<int, 5> pileHeight;
      enum PILE_IDX {LEFT_SIDE = 0,LEFT_FRONT = 1, MIDDLE = 2, RIGHT_FRONT = 3, RIGHT_SIDE = 4};
      
      // Right arm has inverted angles!
      ArmPosition middlePile{
        arm::CAKES_FRONT_DISTANCE + 10e-3,
        arm::MIDDLE_PILE_ANGLE,
        arm::GROUND_HEIGHT + 60e-3};
      ArmPosition frontPile{
        arm::CAKES_FRONT_DISTANCE,
        arm::FRONT_PILE_ANGLE,
        arm::GROUND_HEIGHT + 60e-3};
      ArmPosition sidePile{
        arm::CAKES_SIDE_DISTANCE,
        arm::SIDE_PILE_ANGLE,
        arm::GROUND_HEIGHT + 40e-3};

      ArmPosition last_left_position;
      ArmPosition last_right_position;

      // Common cake dimensions
      double const cake_radius = 60; // [mm]
      double const robot_chassis_front = 90.0;

      // Initial positions of the genoeses
      RobotPosition const genoese_top_left{725,1875,0};
      RobotPosition const genoese_top_right{1275,1875,0};
      RobotPosition const genoese_bottom_left{725,1125,0};
      RobotPosition const genoese_bottom_right{1275,1125,0};

      // Initial barycenters of the cream/ganache couples
      RobotPosition const cream_ganache_top_left{225,2325,0};
      RobotPosition const cream_ganache_top_right{1775,2325,0};
      RobotPosition const cream_ganache_bottom_left{225,675+30,0};
      RobotPosition const cream_ganache_bottom_right{1775,675+30,0};

      // Arm positions ; r theta z
      ArmPosition left_arm_center_up{100, 0, 250};
      ArmPosition left_arm_left_down{100, M_PI_4, 10};
      ArmPosition right_arm_center_up{100, 0, 250};
      ArmPosition right_arm_right_down{100, -M_PI_4, 10};

      void addPositionToQueue_Right(ArmPosition target);
      void addPositionToQueue_Left(ArmPosition target);

      void initPosition(int arm_idx, double r, double theta_rad, double z);
      void initPosition(int arm_idx, ArmPosition const& position);
      void setTargetPosition(int arm_idx,
        int absrel_r, double r,
        int absrel_theta, double theta_rad,
        int absrel_z, double z);
      void setTargetPositionTicks(int arm_idx, 
        int16_t tick0, int16_t tick1, int16_t tick2, int16_t tick3);
      ArmPosition getPileFromIndex(int pile_idx);
      void grabCakeFromPile(int arm_idx, int pile_idx, bool oscillate = false);
      void dumbCakeToPile(int arm_idx, int pile_idx);
      void resetPileHeights();
      void oscillate(int arm_idx, double amplitude_rad);
      void wait(int arm_idx, double duration);
      void pump(int arm_idx, bool activate);
      void adjustRobotPosition();
      void runActionBlock();
      void clearActionSequence();
      void takeCherry();
      int switch_arm(int arm_idx);
      int switch_pile(int pile_idx);
      double switch_angle(double angle);

      void addSyncToQueue();
      void changePileHeight(int pileIndex, int delta);
      int getPileHeight(int pileIndex);
      double getPileZ(int pileIndex);
      void addPumpToLeftQueue(bool activated);
      void addPumpToRightQueue(bool activated);

      void match_impl(); /// Actual implementation of the match code.

      STSServoDriver *servo;

      Action *chooseNextAction(
          std::vector<Action> &actions,
          RobotPosition currentPosition,
          MotionPlanner &motionPlanner);

      /// @brief  \brief Blocks until arms have finished moving
      ArmPosition getArmPosition(int const& armFirstServoId);
      void waitForArmMotion();
      void waitForArmMotionSequenced();
      void depileArm(std::queue<std::shared_ptr<ArmAction > >& actions, int armServoId);
      std::vector<std::shared_ptr<ArmPosition > > computeSequenceToPosition(int const& armFirstServoId, ArmPosition& destination);

      /// @brief  Try to move an arm to a set position, returns false if could not be computed.
      /// @param[in] armPosition Target arm position
      /// @param[in] armFirstServoId Id of the first servo of the arm
      /// @return True if compuation succeeded
      bool setArmPosition(int const& armFirstServoId, ArmPosition const& armPosition);

      /// @brief Execute the cake building sequence
      void buildCakes();
      
      void goBackToBase();
      bool isAtBase_;
      
      // Moving average of the servo current
      std::deque<double> left_arm_current_;
      std::deque<double> right_arm_current_;
  };


  std::ostream& operator<<(std::ostream& os, const main_robot::ArmPosition& p);
}



#endif
