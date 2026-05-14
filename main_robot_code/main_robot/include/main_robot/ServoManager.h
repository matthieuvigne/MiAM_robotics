#ifndef MAIN_ROBOT_SERVO_MANAGER_H
#define MAIN_ROBOT_SERVO_MANAGER_H

#include "common/RobotInterface.h"

enum ArmPosition {
    CALIBRATE,
    GRAB,
    RAISE,
    FOLD_MID,
    FOLD,
    CAMERA_POSE,
    BED_UNFOLD,
    DO_CURSOR
};

enum RailPosition {
    FORWARD,
    INTERNAL,
    DROP
};

enum class Side : int {
    RIGHT = 0,
    LEFT = 1
};

struct CameraResult
{
    bool cratesPresent = false;
    std::vector<Tag> tags;
    double lateralOffset = 0.0;
    double depthOffset = 0.0;
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
    void bedMidUnfold();

    void moveRails(RailPosition const& position);
    bool areRailsMoving();

    void moveArm(ArmPosition const& position);

    /// Transate suction, 0=close, 1=full open
    void translateSuction(Side const side, double const ratio =0.0);


    void releaseSuction();

    void shutdown();

    void pumpOn(Side const side);
    void pumpOff(Side const side);

    // On valve lets air flow
    void valveOn(Side const side);
    void valveOff(Side const side);

    // Complexe actions
    // Gab all crates visible by the robot, handle color logic etc...
    void grabCrates(CameraResult const& cameraResult);
    void dropCrates();

    void emptyBed();

    void doGrab();

    // Hide arm inside robot to take picture
    void hideArm();
    void unhideArm();

    void testArm();

    void moveCratesInBed();

    void fingerOpen();
    void fingerClose();

    CameraResult cameraDetectCrates();

    std::string updateInfoString();

    VisionHandler *getVisionHandler()
    {
        return visionHandler_;
    }
private:
    void grabTags(std::vector<Tag> const& tags, std::vector<int> tagsToGrab, bool secondGrab = false);

    RobotInterface *robot_;
    STSScheduler *servos_;

    std::shared_ptr<RailServo> railX_;
    std::shared_ptr<RailServo> railY_;

    ArmPosition currentArmPosition;

    bool hasElementInSuction_ = true;

    VisionHandler *visionHandler_ = nullptr;
};

#endif
