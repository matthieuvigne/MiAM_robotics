#ifndef PUSHING_CAKES_ACTION_H
#define PUSHING_CAKES_ACTION_H

#include "miam_utils/trajectory/RobotPosition.h"
#include "miam_utils/trajectory/Trajectory.h"
#include "common/RobotInterface.h"
#include "common/AbstractStrategy.h"

using namespace miam;

class PushingCakesAction
{
    public:
        RobotPosition start_position;
        RobotPosition end_position;

        std::vector<Obstacle > obstacles_on_the_road;
        std::vector<Obstacle > obstacles_in_the_end;
};

// Zones :

// 1      2

// 3      4
// 5      6

// 7 8 9 10

class PushCakes1to5 : public PushingCakesAction
{
    public:
        PushCakes1to5()
        {
            start_position.x = 230;
            start_position.y = 2600;
            start_position.theta = -M_PI_2;

            end_position.x = 230;
            end_position.y = 1270;
            end_position.theta = -M_PI_2;

            {
                RobotPosition obstacle;
                obstacle.x = 230;
                obstacle.y = 2300;
                obstacles_on_the_road.push_back(std::make_tuple(obstacle, 300));
            }

            {
                RobotPosition obstacle;
                obstacle.x = 230;
                obstacle.y = 1200;
                obstacles_in_the_end.push_back(std::make_tuple(obstacle, 300));
            }
        }
};

class PushCakes7to5 : public PushingCakesAction
{
    public:
        PushCakes7to5()
        {
            start_position.x = 230;
            start_position.y = 209;
            start_position.theta = M_PI_2;

            end_position.x = 230;
            end_position.y = 1000;
            end_position.theta = M_PI_2;

            {
                RobotPosition obstacle;
                obstacle.x = 230;
                obstacle.y = 670;
                obstacles_on_the_road.push_back(std::make_tuple(obstacle, 300));
            }

            {
                RobotPosition obstacle;
                obstacle.x = 230;
                obstacle.y = 1200;
                obstacles_in_the_end.push_back(std::make_tuple(obstacle, 300));
            }
        }
};

class PushCakes3to4 : public PushingCakesAction
{
    public:
        PushCakes3to4()
        {
            start_position.x = 446;
            start_position.y = 1890;
            start_position.theta = 0;

            end_position.x = 1592;
            end_position.y = 1890;
            end_position.theta = 0;

            {
                RobotPosition obstacle;
                obstacle.x = 738;
                obstacle.y = 1890;
                obstacles_on_the_road.push_back(std::make_tuple(obstacle, 300));
            }

            {
                RobotPosition obstacle;
                obstacle.x = 1770;
                obstacle.y = 1890;
                obstacles_in_the_end.push_back(std::make_tuple(obstacle, 300));
            }
        }
};

class PushCakes6to10 : public PushingCakesAction
{
    public:
        PushCakes6to10()
        {
            start_position.x = 1780;
            start_position.y = 1400;
            start_position.theta = -M_PI_2;

            end_position.x = 1780;
            end_position.y = 410;
            end_position.theta = -M_PI_2;

            // {
            //     RobotPosition obstacle;
            //     obstacle.x = 738;
            //     obstacle.y = 1890;
            //     obstacles_on_the_road.push_back(std::make_tuple(obstacle, 300));
            // }

            {
                RobotPosition obstacle;
                obstacle.x = 1770;
                obstacle.y = 200;
                obstacles_in_the_end.push_back(std::make_tuple(obstacle, 300));
            }
        }
};

#endif