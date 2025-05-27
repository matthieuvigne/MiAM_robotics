/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/RPLidarHandler.h"
#include "miam_utils/trajectory/Utilities.h"
#include <stdio.h>
#include <unistd.h>

using namespace rp::standalone::rplidar;

double const ANGLE_CONVERSION =  90.f / 16384.f * M_PI / 180.0;

// Return angle between 0 and 2 Pi
inline double modulo(double angle)
{
    while(angle > 2 * M_PI)
        angle -= 2 * M_PI;
    while(angle < 0)
        angle += 2 * M_PI;
    return angle;
}

RPLidarHandler::RPLidarHandler(double mountingOffset) :
    debuggingBufferPosition_(0),
    detectedRobots_(),
    isInit_(false),
    lidar(NULL),
    lidarMode_(0),
    lastPointAngle_(0),
    lastPointAddedToBlobDistance_(0),
    pointsNotAddedToBlob_(),
    pointsInBlob_(),
    mountingOffset_(mountingOffset),
    timeHandler_(1.0)
{
}

RPLidarHandler::~RPLidarHandler()
{
    delete lidar;
}

bool RPLidarHandler::init(std::string const& portNameIn, unsigned int const& nPointsPerTurn)
{
    isInit_ = false;
    lidar =  RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if(IS_FAIL(lidar->connect(portNameIn.c_str(), 256400)))
    {
        // Retry because A2M12 or A2 don't use the same baudrate.
        lidar->disconnect();
        usleep(50000);
        lidar =  RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if(IS_FAIL(lidar->connect(portNameIn.c_str(), 115200)))
            return false;
    }

    // Set rotation speed and start with fastest scan rate possible.
    std::vector<RplidarScanMode> modes;
    rplidar_response_device_health_t health;
    if(IS_FAIL(lidar->getAllSupportedScanModes(modes)))
        return false;

    // Find fastest mode.
    fastestModeTime_ = 1.0e6;
    for(unsigned int i = 0; i < modes.size(); i++)
    {
        if (modes.at(i).us_per_sample < fastestModeTime_)
        {
            fastestModeTime_ =  modes.at(i).us_per_sample;
            lidarMode_ = i;
        }
    }
    // Start scan, at default speed.
    isInit_ = true;
    setPointsPerTurn(nPointsPerTurn);
    if(!start())
        isInit_ = false;
    return isInit_;
}


void RPLidarHandler::addPointToBlob(LidarPoint *point)
{
     // Reset points outside of the blob.
    pointsNotAddedToBlob_.clear();
    // Add point to blob.
    lastPointAddedToBlobDistance_ = point->r;
    pointsInBlob_.push_back(*point);
}


int RPLidarHandler::update()
{
    if (!isInit_)
        return 0;
    // Get pending data from the lidar.
    size_t nPoint = 8000;
    rplidar_response_measurement_node_hq_t data[nPoint];
    uint32_t result = lidar->getScanDataWithIntervalHq(data, nPoint);

    if (result == RESULT_OPERATION_TIMEOUT || nPoint == 8000)
        return 0;

    // Remove any robot that was added more than TIMEOUT ago.
    // Element in the queue will be sorted by ascending addition time, so we just need to pop the elements.
    bool wasElementRemoved = true;
    double time = timeHandler_.getElapsedTime();
    while (wasElementRemoved)
    {
        wasElementRemoved = false;
        if (!detectedRobots_.empty())
        {
            if (detectedRobots_.front().addedTime < time - robotTimeout_)
            {
                detectedRobots_.pop_front();
                wasElementRemoved = true;
            }
        }
    }

    for(uint i = 0; i < nPoint; i++)
    {
        // Compute new point.
        LidarPoint newPoint(data[i].dist_mm_q2 /4.0f, 2 * M_PI - (data[i].angle_z_q14 * ANGLE_CONVERSION) + mountingOffset_);

        // If new point is not in order (recall scan is done in decreasing angle), just discard the new data point.
        if (newPoint.isOlder(lastPointAngle_))
            continue;
        lastPointAngle_ = newPoint.theta;

        // Determine if the current point is to be added to the blob or not.
        if(newPoint.r < MAX_DISTANCE && std::abs(lastPointAddedToBlobDistance_ - newPoint.r) < BLOB_THICKNESS && newPoint.r > MIN_DISTANCE)
        {
            addPointToBlob(&newPoint);
        }
        else
        {
            // Point not added to the blob: add it to vector.
            pointsNotAddedToBlob_.push_back(newPoint);
            if (pointsNotAddedToBlob_.size()  >= BLOB_BREAK)
            {
                // The blob is over: process its data.
                int nPoints = pointsInBlob_.size();
                if (nPoints >= MIN_POINTS || (newPoint.r > 700 && nPoints >= MIN_POINTS / 2))
                {
                    LidarPoint a = pointsInBlob_[0];
                    LidarPoint b = pointsInBlob_[nPoints - 1];
                    double blobDiameter = std::sqrt(a.r * a.r + b.r * b.r - 2 * a.r * b.r * std::cos(a.theta - b.theta));
                    if (blobDiameter > BLOB_MIN_SIZE && blobDiameter < BLOB_MAX_SIZE)
                    {
                        // We have a new robot: add it to the list.
                        double blobDistance = 0.0;
                        for(int j = 0; j < nPoints; j++)
                            blobDistance += pointsInBlob_[j].r;
                        blobDistance /= (1.0 * nPoints);
                        double arcAngle = std::min(modulo(b.theta - a.theta), modulo(a.theta - b.theta));
                        double blobAngle = modulo(a.theta - arcAngle / 2.0);

                        LidarPoint robot(blobDistance, blobAngle);
                        detectedRobots_.push_back(DetectedRobot(robot, timeHandler_.getElapsedTime(), nPoints));
                    }
                }
                // Clear blob.
                pointsInBlob_.clear();

                // Create a new blob with the current point.
                currentBlobNumber_ = (currentBlobNumber_ + 1) % 5;
                if(currentBlobNumber_ == 0)
                    currentBlobNumber_ = 1;

                addPointToBlob(&newPoint);
            }
        }
        // Add new point to debugging buffer.
        debuggingBuffer_[debuggingBufferPosition_] = newPoint;
        debuggingBufferPosition_ = (debuggingBufferPosition_ + 1) % DEBUGGING_BUFFER_LENGTH;
    }
    return nPoint;
}

void RPLidarHandler::stop()
{
    if (!isInit_)
        return;
    lidar->stopMotor();
    lidar->stop();
}

bool RPLidarHandler::start()
{
    if (!isInit_)
        return false;
    lidar->startMotor();
    lidar->setMotorPWM(desiredSpeed_);
    return !IS_FAIL(lidar->startScanExpress(false, lidarMode_));
}

void RPLidarHandler::setPointsPerTurn(unsigned int const& nPoints)
{
    isInit_ = true;
    double const speed = 60 / (nPoints * fastestModeTime_ * 1e-6);
    desiredSpeed_ = static_cast<uint16_t>(speed);
    robotTimeout_ = 1.2 * 60 / speed;
    lidar->setMotorPWM(desiredSpeed_);
}