#ifndef MIAM_UTILS_LIDAR_POINT_HPP
#define MIAM_UTILS_LIDAR_POINT_HPP

#include <cmath>

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

#endif // MIAM_UTILS_LIDAR_POINT_HPP