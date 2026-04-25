#ifndef MAIN_ROBOT_SERVO_MANAGER_H
#define MAIN_ROBOT_SERVO_MANAGER_H

#include "common/RobotInterface.h"
#include "main_robot/VisionHandler.h"

enum ArmPosition {
    CALIBRATE,
    GRAB,
    RAISE,
    FOLD_MID,
    FOLD,
    CAMERA_POSE
};

enum RailPosition {
    FORWARD,
    INTERNAL
};

enum class Side : int {
    RIGHT = 0,
    LEFT = 1
};

class ServoManager
{
public:
    ServoManager();
    ~ServoManager();

    void init(RobotInterface *robot);

    void cursorFold();
    void cursorUnfold();

    void bedFold();
    void bedUnfold();

    void moveRails(RailPosition const& position);
    bool areRailsMoving();

    void moveArm(ArmPosition const& position);

    /// Transate suction, 0=close, 1=full open
    void translateSuction(Side const side, double const ratio =0.0);


    void shutdown();

    void pumpOn(Side const side);
    void pumpOff(Side const side);


    // Complexe actions
    // Gab all crates visible by the robot, handle color logic etc...
    void grabCrates();
    void dropCrates();

    // Hide arm inside robot to take picture
    void hideArm();
    void unhideArm();
    VisionHandler visionHandler_;

    void testArm();

private:
    void grabTags(std::vector<Tag> const& tags, std::vector<int> tagsToGrab);

    void moveCratesInBed();

    RobotInterface *robot_;
    STSScheduler *servos_;

    std::shared_ptr<RailServo> railX_;
    std::shared_ptr<RailServo> railY_;

    int lastCloseTarget_back_R;
    int lastCloseTarget_back_L;
    int lastCloseTarget_front_R;
    int lastCloseTarget_front_L;

    ArmPosition currentArmPosition;
};

#endif
