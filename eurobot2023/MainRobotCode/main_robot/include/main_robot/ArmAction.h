/// \file ArmAction.h
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef MAIN_ROBOT_ARMACTION_H
#define MAIN_ROBOT_ARMACTION_H

#include <memory>
#include <queue>
#include <mutex>
#include <array>

namespace main_robot
{
    enum ActionType
    {
        SYNC = 0,
        MOVE = 1,
        PUMP = 2,
        WAIT = 3
    };

    class ArmAction
    {
    public:
        ArmAction() {}
        virtual ~ArmAction() = default;
        ActionType type_;
    };

    class ArmSync : public ArmAction
    {
        public:
        typedef std::shared_ptr<ArmSync > Ptr;
        ArmSync() : ArmAction() 
        {
            type_ = ActionType::SYNC;
        };
    };

    class ArmWait : public ArmAction
    {
        public:
        typedef std::shared_ptr<ArmWait > Ptr;
        ArmWait(double time) : ArmAction(), time_(time)
        {
            type_ = ActionType::WAIT;
        };
        double time_;
    };

    class ArmPosition : public ArmAction
    {
    public:
        typedef std::shared_ptr<ArmPosition > Ptr;
        double r_;
        double theta_;
        double z_;

        ArmPosition(double r, double theta, double z) : ArmAction(), r_(r), theta_(theta), z_(z)
        {
            type_ = ActionType::MOVE;
        };

        ArmPosition() : ArmPosition(0.0, 0.0, 0.0) {};
    };

    class ArmPump : public ArmAction
    {
    public:
        typedef std::shared_ptr<ArmPump > Ptr;
        bool activated_;

        ArmPump(double activated) : ArmAction(), activated_(activated) {
            type_ = ActionType::PUMP;
        };
    };
}



#endif