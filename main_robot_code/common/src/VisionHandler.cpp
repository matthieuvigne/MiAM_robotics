#include "common/VisionHandler.h"
#include <algorithm>

VisionHandler::VisionHandler()
{
}

bool VisionHandler::init()
{
    #ifdef SIMULATION
        return true;
    #endif
    isInit_ = reader_.init();
    return isInit_;
}

std::vector<Tag> VisionHandler::getTags()
{
    std::vector<Tag> tags;

    #ifdef SIMULATION
        for (int i = 0; i < 4; i++)
        {
            Eigen::Vector3d v;
            v.x() = 0.05;
            v.y() = 0.05 * (i - 1.5);
            v.z() = -0.200;
            tags.push_back(Tag{i % 2 ? YELLOW : BLUE, v});
        }
        return tags;
    #endif
    if (isInit_)
    {
        VisionBuffer buffer;
        reader_.update(buffer);
        for (int i = 0; i < buffer.nMarkers; i++)
        {
            Eigen::Vector3d v;
            v.x() = buffer.markers[i].posX;
            v.y() = buffer.markers[i].posY;
            v.z() = buffer.markers[i].posZ;
            tags.push_back(Tag{buffer.markers[i].markerId, v});
        }

        // Sort tags from left to right
        std::sort(tags.begin(), tags.end(), [](Tag a, Tag b) {return a.position.y() > b.position.y();});
    }
    return tags;
}
