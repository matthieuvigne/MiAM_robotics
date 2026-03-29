#ifndef MAIN_ROBOT_VISION_HANDLER_H
#define MAIN_ROBOT_VISION_HANDLER_H

#include "VisionSharedMem.h"
#include <eigen3/Eigen/Dense>
#include <vector>


struct Tag
{
    int markerId;
    Eigen::Vector3d position;
};

int constexpr YELLOW = 47;
int constexpr BLUE = 36;

class VisionHandler
{
    public:
        VisionHandler();

        // Return the tags found, sorted along the y axis.
        std::vector<Tag> getTags();

    private:
        SHMReader reader_;

};

#endif
