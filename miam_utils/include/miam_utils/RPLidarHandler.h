/// \file RPLidarHandler.h
/// \brief Handler for RPLidar sensor.
///
/// \details Class RPLidarHandler deals directly with the lidar measurement, and performs robot detection based on LIDAR
///          data.
///             Robot detection works as follow: whenever a new data point is aquired, we look at its distance from the current
///             blob average. A point more than BLOB_THICKNESS distance away from the average is not added. If more than
///          BLOB_BREAK consecutive points are not added, the current blob is considered terminated. If its width matches
///          a robot beacon, the corresponding position is stored. In any case, a new blob is created by the current point.

/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef RPLIDAR_HANDLER
#define RPLIDAR_HANDLER
    #include <rplidar.h>
    #include <iostream>
    #include <cmath>
    #include <deque>
    #include "MiAMEurobot/Metronome.h"

    // Constant parameters.
    #define DEBUGGING_BUFFER_LENGTH 2000

    const double LIDAR_RPM = 600.0;    ///< Lidar velocity, in rpm.
    const double MAX_DISTANCE = 1700.0; ///< Maximum distance for processing, in mm: points above that distance are discarded.
    const double MIN_DISTANCE = 50.0; ///< Minimum distance for processing, in mm: points below that distance are discarded.

    const double BLOB_THICKNESS = 40.0;///< Distance between two adjacent points to consider that they belong to the same blob, in mm.
    const double BLOB_MIN_SIZE = 40.0; ///< Minimum size of the blob to consider it as a robot.
    const double BLOB_MAX_SIZE = 200.0; ///< Maximum size of the blob to consider it as a robot.

    const int BLOB_BREAK = 2;///< Number of points needed to consider that a block has come to an endMinimum number of points to be a valid obstacle.

    const int MIN_POINTS = 9;///< Minimum number of points inside a blob to be considered a robot.
                             ///< At 1.5m, 600rpm, 8ksamples/s, a circle of 70mm corresponds to 6 points.

    const double ROBOT_TIMEOUT = 1.2 * 60 / LIDAR_RPM; ///< Timeout, in s, to remove a robot for the list of ostacle.
                                                       ///  Corresponds to 1.2 theoretical lidar motion.

    /// \brief Structure representing a data point returned by the lidar.
    struct LidarPoint
    {
        LidarPoint():
            r(1000.0),
            theta(2 * M_PI)
        {
        }

        LidarPoint(double const& rIn, double const& thetaIn):
            r(rIn),
            theta(thetaIn)
        {
            if (theta < 0)
                theta += 2 * M_PI;
            if (theta > 2 * M_PI)
                theta -= 2 * M_PI;
        }

        /// \brief Determine if the current point is older than the comparison point.
        /// \details Knowing that the lidar is turning clockwise, this function returns true if the current point
        ///          was taken before the comparison point. A point is considered older if its continuous angle
        ///          is in [comparisonPointAngle, comparisonPointAngle + tolerance]
        bool isOlder(double const& comparisonPointAngle, double const& tolerance = M_PI_2)
        {
            // Both angles are between 0 and 2 pi: modify current point angle if necessary to have a continuous mapping.
            double currentAngle = this->theta;
            if (currentAngle < comparisonPointAngle)
                currentAngle += 2 * M_PI;

            // If point is bettween comparisonPoint.theta and comparisonPoint.theta + pi, it is older than the given point.
            if (currentAngle >= comparisonPointAngle && currentAngle < comparisonPointAngle + tolerance)
                return true;
            return false;
        }

        double r;
        double theta;
    };


    /// \brief Structure representing a robot seen by the lidar.
    struct DetectedRobot
    {
        DetectedRobot():
            point(),
            addedTime(0.0)
        {
        }

        DetectedRobot(LidarPoint const& p, double const& t, int n = 0):
            point(p),
            addedTime(t),
            nPoints(n)
        {
        }

        LidarPoint point; ///< Robot position.
        double addedTime; ///< Absolute time at which the robot was detected.
        int nPoints;    ///< Number of points in the blob.
    };


    class RPLidarHandler
    {
        public:
            /// \brief Constructor.
            ///
            /// \details The constructor does not perform any communication operation: use init for that.
            RPLidarHandler(double mountingOffset = 0.0);

            /// \brief Destructor.
            ~RPLidarHandler();

            /// \brief Init communication with the sensor.
            /// \details This function initialises communication with the RPLidar, and starts scan mode.
            ///
            /// \param[in] portNameIn Name of the port to use.
            /// \return True on successful sensor initialization.
            bool init(std::string const& portNameIn);

            /// \brief Stop the lidar (motor and scanning core).
            void stop();

            /// \brief Start the lidar (motor and scanning core).
            bool start();

            /// \brief Update sensor data.
            /// \details This function should be called frequently in a loop.
            /// \return Number of points recieved.
            int update();

            // For debugging - display of the last scan data.
            LidarPoint debuggingBuffer_[DEBUGGING_BUFFER_LENGTH];
            int debuggingBufferPosition_; ///< Position in the debugging buffer.

            std::deque<DetectedRobot> detectedRobots_;    ///< Vector holding the detected robots, as fifo of robot angle.

        private:
            /// \brief Add a point to the current blob.
            void addPointToBlob(LidarPoint *point);

            rp::standalone::rplidar::RPlidarDriver *lidar; ///< The lidar instance.
            int lidarMode_; ///< Scan mode for the lidar.

            double lastPointAngle_; ///< Angle of the last processed point.
            double lastPointAddedToBlobDistance_; ///< Distance of the last point added to the blob.

            std::vector<LidarPoint> pointsNotAddedToBlob_; ///< Vector of consecutive points that were not added to the blob.
            std::vector<LidarPoint> pointsInBlob_; ///< Vector of points constituting a blob.

            int currentBlobNumber_; ///< Current blob number, for display coloring.

            double mountingOffset_; ///< Lidar mounting offset.

            Metronome timeHandler_; ///< Metronome - for getting relative time.
    };
#endif
