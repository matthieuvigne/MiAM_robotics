/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/RPLidarHandler.h"
#include "miam_utils/trajectory/Utilities.h"

#include <stdio.h>
#include <unistd.h>
#include <random>

#include <eigen3/Eigen/Dense>

#define ANSI_RED        "\033[0;31m]"
#define ANSI_GREEN      "\033[0;32m]"
#define ANSI_YELLOW     "\033[0;33m]"
#define ANSI_BLUE       "\033[0;34m]"
#define ANSI_MAGENTA    "\033[0;35m]"
#define ANSI_CYAN       "\033[0;36m]"
#define ANSI_WHITE      "\033[0;37m]"
#define ANSI_RESET      "\033[0;0m]"

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
        // Remove outdated robots
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

    // Remove any beacon that was added more than TIMEOUT ago.
    // Element in the queue will be sorted by ascending addition time, so we just need to pop the elements.
    wasElementRemoved = true;
    time = timeHandler_.getElapsedTime();
    while (wasElementRemoved)
    {
        // Remove outdated robots
        wasElementRemoved = false;
        if (!detectedBeacons_.empty())
        {
            if (detectedBeacons_.front().addedTime < time - robotTimeout_)
            {
                detectedBeacons_.pop_front();
                wasElementRemoved = true;
            }
        }
    }

    // Detect blobs
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

                // Process the blob to detect a beacon
                detectBeaconInCurrentBlob();

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

bool RPLidarHandler::detectBeaconInCurrentBlob()
{
    // Check whether the blob gathers points which belong to the same circle
    // and such that the circle is compatible with being a beacon. It uses a
    // RANSAC estimation loop.

    // Set RANSAC parameters
    //double const max_radius_error = 5; // [mm]
    Blob const& blob = pointsInBlob_;
    int const num_points = static_cast<int>(blob.size());
    printf("---------------------------------------------------------\n");
    printf("NEW BEACON -> RANSAC LOOP (%d points):\n", num_points);
    for(LidarPoint const& point : blob)
        printf("(r=%.0f,theta=%.0f)\n",point.r,point.theta*180./M_PI);
    int const min_num_inliers = std::max(10.,std::floor(0.70*num_points));
    if(num_points < 20) return false; // TODO reduce

    // Set the random device
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0,num_points-1);

    // Save the best results
    double best_xc = NAN;
    double best_yc = NAN;
    int best_num_inliers = 0;
    std::vector<int> best_inliers;

    // Run the RANSAC loop
    int iter_idx = 0;
    int const max_iters = 50;
    while(true)
    {
        // Sample three points from the blob
        int i1 = dis(gen);
        int i2 = dis(gen);
        while(i2==i1) i2 = dis(gen);
        LidarPoint const& p1 = blob[i1];
        LidarPoint const& p2 = blob[i2];

        // Triangulate the center of the circle
        // [Note] We get two center hypothesis
        double const x1 = p1.r*cos(p1.theta);
        double const y1 = p1.r*sin(p1.theta);
        double const x2 = p2.r*cos(p2.theta);
        double const y2 = p2.r*sin(p2.theta);
        double const dx = x2-x1;
        double const dy = y2-y1;
        double const d2 = dx*dx + dy*dy;
        double const d = sqrt(d2);
        if(d > 2*BEACON_RADIUS) continue;
        double const h = sqrt(std::fmax(0.,pow(BEACON_RADIUS,2.) - d2/4.));
        // >> First center hypothesis
        double const xc1 = 0.5*(x1+x2) - h*dy/d;
        double const yc1 = 0.5*(y1+y2) + h*dx/d;
        //printf("H1 -> xc1=%.0f, yc1=%.0f\n", xc1, yc1);
        // >> Second center hypothesis
        double const xc2 = 0.5*(x1+x2) + h*dy/d;
        double const yc2 = 0.5*(y1+y2) - h*dx/d;
        //printf("H2 -> xc2=%.0f, yc2=%.0f\n", xc2, yc2);

        // Count the number of inliers for each center hypothesis
        std::vector<int> inliers1, inliers2;
        for(int point_idx=0; point_idx<num_points; point_idx++)
        {
            // Get the point Cartesian coordinates
            LidarPoint const& p = blob[point_idx];
            double const xp = p.r*cos(p.theta);
            double const yp = p.r*sin(p.theta);

            // Get residuals for both hypothesis
            double const r1 = sqrt( pow(xp-xc1,2.) + pow(yp-yc1,2.) );
            if(std::fabs(r1-BEACON_RADIUS) < 0.10*BEACON_RADIUS)
                inliers1.push_back(point_idx);
            double const r2 = sqrt( pow(xp-xc2,2.) + pow(yp-yc2,2.) );
            if(std::fabs(r2-BEACON_RADIUS) < 0.10*BEACON_RADIUS)
                inliers2.push_back(point_idx);
            continue;
        }

        // Keep the best hypothesis
        int const num_inliers1 = static_cast<int>(inliers1.size());
        int const num_inliers2 = static_cast<int>(inliers2.size());
        if(num_inliers1 > best_num_inliers){
            best_num_inliers = num_inliers1;
            best_inliers = inliers1;
            best_xc = xc1;
            best_yc = yc1;
        } else if(num_inliers2 > best_num_inliers){
            best_num_inliers = num_inliers2;
            best_inliers = inliers2;
            best_xc = xc2;
            best_yc = yc2;
        }
        printf("[iter %d] xc=%.0f, yc=%.0f\n", iter_idx, best_xc, best_yc);

        // Check stop conditions
        if(iter_idx >= max_iters){
            printf("Max iterations reached -> break!\n");
            break;
        }
        if(best_num_inliers > min_num_inliers){
            printf("Sufficient number of inliers reached (%d/%d)-> break!\n",
                best_num_inliers, min_num_inliers);
            break;
        }
        iter_idx += 1;
        continue;
    }

    // Check if the best result is sufficient
    printf("Found beacon at x=%d and y=%d\n", int(best_xc), int(best_yc));
    if(std::isnan(best_xc)) return false;
    if(std::isnan(best_yc)) return false;
    if(best_num_inliers < min_num_inliers){
        printf("Not enough inliers -> abort!\n");
        return false;
    }
    printf("Passed the tests successfully!\n");

    // Refine the position of the center of the beacon
    // [Note] Least-square update of the best estimate
    iter_idx = 0;
    while(true)
    {
        Eigen::MatrixXd A(best_num_inliers,2);
        Eigen::VectorXd b(best_num_inliers);
        for(int inlier_idx=0; inlier_idx<best_num_inliers; inlier_idx+=1){
            LidarPoint const& p = blob[inlier_idx];
            double const x = p.r * cos(p.theta);
            double const y = p.r * sin(p.theta);
            A(inlier_idx,0) = 2*(best_xc-x);
            A(inlier_idx,1) = 2*(best_yc-y);
            b(inlier_idx) = (best_xc*best_xc - x*x) + (best_yc*best_yc - y*y) + pow(BEACON_RADIUS,2.);
            continue;
        }
        Eigen::Vector2d new_xc_yc = (A.transpose()*A).ldlt().solve(A.transpose()*b);
        bool const okx = (std::fabs(best_xc-new_xc_yc(0))<2);
        bool const oky = (std::fabs(best_yc-new_xc_yc(1))<2);
        best_xc = new_xc_yc(0);
        best_yc = new_xc_yc(1);
        if( (okx && oky) || (iter_idx>5) ) break;
        iter_idx += 1;
        continue;
    }
    printf("Refined at x=%d and y=%d\n", int(best_xc), int(best_yc));

    // Add the beacon to the list
    double const r = sqrt( best_xc*best_xc + best_yc*best_yc );
    double const theta = atan2(best_yc,best_xc);
    DetectedBeacon new_beacon;
    new_beacon.point.r = r;
    new_beacon.point.theta = theta;
    new_beacon.nPoints = best_num_inliers;
    new_beacon.addedTime = timeHandler_.getElapsedTime();
    printf( ANSI_GREEN "Added the new beacon r=%.0f, theta=%.0f (%d/%d)\n" ANSI_RESET, 
        r, theta*180./M_PI, best_num_inliers, num_points);
    detectedBeacons_.emplace_back(new_beacon);
    return true;
}

//-------------------------------------------------------------------------------------------------

void estimate_robot_position_from_beacons(
    double& x,      double const sigma_x, 
    double& y,      double const sigma_y, 
    double& theta,  double const sigma_theta,
    std::vector<DetectedBeacon> const& detected_beacons,
    std::vector<ReferenceBeacon> const& reference_beacons)
{       
    // Perform data association
    int const num_ref_beacons = static_cast<int>(reference_beacons.size());
    std::vector<int> detected_to_reference(detected_beacons.size(),-1);
    for(int idx=0; idx<static_cast<int>(detected_beacons.size()); idx++)
    {
        // Get the detected beacon infos
        DetectedBeacon const& beacon = detected_beacons[idx];
        double const xd = x + beacon.point.r * cos(beacon.point.theta);
        double const yd = y + beacon.point.r * sin(beacon.point.theta);

        // Find the best reference beacon
        int best_beacon_idx = -1;
        double best_distance = std::numeric_limits<double>::infinity();
        for(int ref_idx=0; ref_idx<num_ref_beacons; ref_idx+=1)
        {
            ReferenceBeacon const& ref_beacon = reference_beacons[ref_idx];
            double const xr = ref_beacon.x;
            double const yr = ref_beacon.y;
            double const d = sqrt( pow(xd-xr,2.) + pow(yd-yr,2.) );
            if(d<best_distance)
            {
                best_distance = d;
                best_beacon_idx = ref_idx;
            }
        }

        // If the best distance is acceptable, save it
        double const max_acceptable_distance = 50; // [mm]
        if(best_distance < max_acceptable_distance)
            detected_to_reference[idx] = best_beacon_idx;
        continue;
    }

    // Build and solve the MAP estimation problem
    int iter_idx = 0;
    int const max_iters = 10;
    while(true)
    {
        // Build the MAP estimation problem
        int const num_detected_beacons = static_cast<int>(detected_beacons.size());
        int const num_constraints = 3 + 2*num_detected_beacons;
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_constraints,3);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(num_constraints,3);
        // >> Add the prior term
        A(0,0) = 1/sigma_x;     b(0) = x/sigma_x;
        A(1,1) = 1/sigma_y;     b(1) = y/sigma_y;
        A(2,2) = 1/sigma_theta; b(2) = theta/sigma_theta;
        // >> Add the beacon residuals
        for(int idx=0; idx<num_detected_beacons; idx+=1)
        {
            int const row0_idx = 3 + 2*idx;

            // Get the detected beacon and associated measurements
            DetectedBeacon const& beacon = detected_beacons[idx];
            double const r_mes = beacon.point.r;
            double const theta_mes = beacon.point.theta;

            // Get the reference beacon and predicted measurements
            int const ref_idx = detected_to_reference[idx];
            if(idx<0) continue;
            ReferenceBeacon const& ref_beacon = reference_beacons[ref_idx];
            double const dx = ref_beacon.x - x;
            double const dy = ref_beacon.y - y;
            double const r_pred = sqrt(dx*dx + dy*dy);
            double const theta_pred = atan2(dy,dx) - theta;

            // Get the measurement standard deviations
            double const sigma_r = (0.1*r_pred)/3.;         // 10% @ 3sigma
            double const sigma_theta = (2*M_PI/180.)/3.;    // 2°  @ 3sigmas

            // Add the matrix terms
            /**
             * r = ||xb-x,yb-y||        => dr/dX        = ( -(xb-x)/r, -(yb-y)/r)
             * theta = atan2(yb-y,xb-x) => dtheta/dX    = (+(yb-y)/r2, -(xb-x)/r2)
             */

            // Add the range residual
            double const er = r_pred - r_mes;
            double const der_dx = dx/r_pred; 
            double const der_dy = dy/r_pred;
            A(row0_idx+0,0) = der_dx/sigma_r;
            A(row0_idx+0,1) = der_dy/sigma_r;
            b(row0_idx+0)   = (der_dx*x + der_dy*y - er)/sigma_r;

            // Add the angular residual
            double const et = theta_pred - theta_mes;
            double const det_dx     = -dy/pow(r_pred,2.);
            double const det_dy     =  dx/pow(r_pred,2.);
            double const det_dtheta = -1.0;
            A(row0_idx+1,0) = det_dx/sigma_theta;
            A(row0_idx+1,1) = det_dy/sigma_theta;
            A(row0_idx+1,2) = det_dtheta/sigma_theta;
            b(row0_idx+1)   = (der_dx*x + der_dy*y + det_dtheta*theta - et)/sigma_theta;
            
            continue;
        }

        // Solve the current iteration
        Eigen::Vector3d const s = (A.transpose()*A).ldlt().solve(A.transpose()*b);
        bool ok1 = (std::fabs(s(0) - x)     < 50); // [mm]
        bool ok2 = (std::fabs(s(1) - y)     < 50); // [mm]
        bool ok3 = (std::fabs(s(2) - theta) < 0.5*M_PI/180.);
        x = s(0);
        y = s(1);
        theta = s(2);
        if(ok1 && ok2 && ok3) break;
        if(iter_idx>=max_iters) break;
        iter_idx += 1;
        continue;
    }
    return;
}

//-------------------------------------------------------------------------------------------------