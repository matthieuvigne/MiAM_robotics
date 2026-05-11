#include "ESP32Listener.h"

ESP32Listener::ESP32Listener()
{
}


bool ESP32Listener::init(std::string const& portName, int const& baudRate)
{
    return listener_.init(portName, baudRate);
}

bool ESP32Listener::update()
{
    bool result = listener_.update();
    if (result)
    {
        std::vector<uint8_t> data = listener_.getLastFrame();
        result = data.size() == 4;
        if (result)
        {
            lastData_.obstacleDistance = static_cast<int32_t>(
                    (static_cast<uint32_t>(data[3]) << 24) |
                    (static_cast<uint32_t>(data[2]) << 16) |
                    (static_cast<uint32_t>(data[1]) << 8)  |
                    static_cast<uint32_t>(data[0]));
        }
    }
    return result;
}

ESP32Data ESP32Listener::getLastData()
{
    return lastData_;
}


std::ostream& operator<<(std::ostream& os, ESP32Data const& d)
{
    os << "distance: " << d.obstacleDistance;
    return os;
}