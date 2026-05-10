#include <random>

#include <eigen3/Eigen/Dense>

#include <miam_utils/beacon_detector.hpp>

//-------------------------------------------------------------------------------------------------

BeaconDetector::BeaconDetector()
: timeHandler_(1.0)
{}

//-------------------------------------------------------------------------------------------------

void BeaconDetector::remove_outdated_beacons(double timeout)
{
    bool was_element_removed = true;
    double const time = timeHandler_.getElapsedTime();
    while (was_element_removed)
    {
        was_element_removed = false;
        if (!detected_beacons_.empty())
        {
            if (detected_beacons_.front().addedTime < time - timeout)
            {
                detected_beacons_.pop_front();
                was_element_removed = true;
            }
        }
    }
    return;
}

//-------------------------------------------------------------------------------------------------

bool BeaconDetector::detect(Blob const& blob)
{
    // Check whether the blob gathers points which belong to the same circle
    // and such that the circle is compatible with being a beacon. It uses a
    // RANSAC estimation loop.

    // Set RANSAC parameters
    //double const max_radius_error = 5; // [mm]
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
        if(d > 2*R_) continue;
        double const h = sqrt(std::fmax(0.,R_*R_-d2/4.));
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
            if(std::fabs(r1-R_) < 0.10*R_) inliers1.push_back(point_idx);
            double const r2 = sqrt( pow(xp-xc2,2.) + pow(yp-yc2,2.) );
            if(std::fabs(r2-R_) < 0.10*R_) inliers2.push_back(point_idx);
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
    //printf("------------------------------------------------------\n");

    // Check if the best result is sufficient
    printf("Found beacon at x=%d and y=%d\n", int(best_xc), int(best_yc));
    if(std::isnan(best_xc)) return false;
    if(std::isnan(best_yc)) return false;
    if(best_num_inliers < min_num_inliers) return false;
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
            b(inlier_idx) = (best_xc*best_xc - x*x) + (best_yc*best_yc - y*y) + R_*R_;
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

    // Associate the detected beacon to a reference detected beacon
    // [Note] We assume we should not be too far from a reference beacon.
    /*int best_beacon_idx = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    int const num_ref_beacons = static_cast<int>(reference_beacons_.size());
    for(int ref_idx=0; ref_idx<num_ref_beacons; ref_idx+=1)
    {
        Beacon const& ref_beacon = reference_beacons_[ref_idx];
        double const xr = ref_beacon.x;
        double const yr = ref_beacon.y;
        double const d = sqrt( pow(best_xc-xr,2.) + pow(best_yc-yr,2.) );
        if(d<best_distance){
            best_distance = d;
            best_beacon_idx = ref_idx;
        }
    }
    double const max_acceptable_distance = 200; // [mm]
    if(best_distance > max_acceptable_distance) 
        return false;*/

    // Add the beacon to the list
    double const r = sqrt( best_xc*best_xc + best_yc*best_yc );
    double const theta = atan2(best_yc,best_xc);
    DetectedBeacon new_beacon;
    new_beacon.point.r = r;
    new_beacon.point.theta = theta;
    new_beacon.nPoints = best_num_inliers;
    new_beacon.addedTime = timeHandler_.getElapsedTime();
    /*new_beacon.ref_idx = best_beacon_idx;*/
    printf("Added the new beacon r=%.0f, theta=%.0f (%d/%d)\n", r, theta*180./M_PI, best_num_inliers, num_points);
    detected_beacons_.emplace_back(new_beacon);
    return true;
}

//-------------------------------------------------------------------------------------------------

void BeaconDetector::update_robot_position(
    double& x,      double const sigma_x,
    double& y,      double const sigma_y,
    double& theta,  double const sigma_theta) const
{
    int iter_idx = 0;
    int const max_iters = 10;
    while(true)
    {
        // Build the MAP estimation problem
        int const num_detected_beacons = static_cast<int>(detected_beacons_.size());
        int const num_constraints = 3 + 2*num_detected_beacons;
        Eigen::MatrixXd A(num_constraints,3);
        Eigen::VectorXd b(num_constraints,3);
        A(0,0) = 1/sigma_x;     b(0) = x/sigma_x;
        A(1,1) = 1/sigma_y;     b(1) = y/sigma_y;
        A(2,2) = 1/sigma_theta; b(2) = theta/sigma_theta;
        for(int idx=0; idx<num_detected_beacons; idx+=1)
        {
            int const row0_idx = 3 + 2*idx;
            DetectedBeacon const& beacon = detected_beacons_[idx];
            Beacon const& ref_beacon = reference_beacons_[beacon.ref_idx];
            double const dx = ref_beacon.x - x;
            double const dy = ref_beacon.y - y;
            double const r2 = std::fmax(1e-8,dx*dx + dy*dy);
            double const r = sqrt(r2);
            double const theta = atan2(dy,dx);
            double const sigma_r = (0.1*r)/3.;              // 10% @ 3sigma
            double const sigma_theta = (2*M_PI/180.)/3.;    // 2°  @ 3sigmas
            /**
             * r = ||xb-x,yb-y||        => dr/dX        = ( -(xb-x)/r, -(yb-y)/r)
             * theta = atan2(yb-y,xb-x) => dtheta/dX    = (+(yb-y)/r2, -(xb-x)/r2)
             */
            A(row0_idx+0,0) =  -dx/r/sigma_r;
            A(row0_idx+0,1) =  -dy/r/sigma_r;
            b(row0_idx+0)   = ((-x*dx-y*dy)/r-r)/sigma_r;
            A(row0_idx+1,0) =  dy/r2/sigma_theta;
            A(row0_idx+1,1) = -dx/r2/sigma_theta;
            b(row0_idx+1)   = (dy/r2*x -dx/r2*y-theta)/sigma_theta;
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

void BeaconDetector::for_each_detected_beacon(
    std::function<void(DetectedBeacon const&)> action)
{
    for(DetectedBeacon const& beacon : detected_beacons_)
        action(beacon);
    return;
}

//-------------------------------------------------------------------------------------------------