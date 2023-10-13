/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/SampledTrajectory.h"
#include <cmath>


namespace miam{
    namespace trajectory{
        SampledTrajectory::SampledTrajectory(
            TrajectoryConfig const& config,
            std::vector<TrajectoryPoint > sampledTrajectory,
            double duration
            ) : Trajectory(config)
        {
            description_ = "SampledTrajectory";
            duration_ = duration;
            sampledTrajectory_ = sampledTrajectory;
        }

        TrajectoryPoint SampledTrajectory::getCurrentPoint(double const& currentTime)
        {
            if (MIAM_DEBUG_TRAJECTORY_TYPE)
                std::cout << "SampledTrajectory::getCurrentPoint " << currentTime << std::endl;
            int N = sampledTrajectory_.size();

            if (N < 1)
            {
                return TrajectoryPoint();
            }

            if (currentTime >= duration_)
            {
                return sampledTrajectory_.back();
            }
            else if (currentTime <= 0.0)
            {
                return sampledTrajectory_.front();
            }

            int indexLow = std::floor((N-1) * currentTime / duration_);
            int indexHigh = std::ceil((N-1) * currentTime / duration_);

            TrajectoryPoint tpLow = sampledTrajectory_.at(indexLow);
            TrajectoryPoint tpHigh = sampledTrajectory_.at(indexHigh);

            double sampledTimestep = duration_ / (N-1);

            double residue = (currentTime - indexLow * sampledTimestep) / sampledTimestep;

            double ponderationLow = 1.0 - residue;
            double ponderationHigh = residue;

            // Linear interpolation
            TrajectoryPoint output;
            output.position.x = ponderationLow * tpLow.position.x + ponderationHigh * tpHigh.position.x;
            output.position.y = ponderationLow * tpLow.position.y + ponderationHigh * tpHigh.position.y;
            output.position.theta = ponderationLow * tpLow.position.theta + ponderationHigh * tpHigh.position.theta;
            output.linearVelocity = ponderationLow * tpLow.linearVelocity + ponderationHigh * tpHigh.linearVelocity;
            output.angularVelocity = ponderationLow * tpLow.angularVelocity + ponderationHigh * tpHigh.angularVelocity;

            return output;

        }

        void SampledTrajectory::replanify(double const& replanificationTime)
        {
            std::cout << "Called SampledTrajectory::replanify" << std::endl;

            int N = sampledTrajectory_.size();

            // case trajectory is finished
            if (replanificationTime >= getDuration() || N == 1)
            {
                sampledTrajectory_.clear();
                sampledTrajectory_.push_back(getCurrentPoint(replanificationTime));
                duration_ = 0.0;
                return;
            }

            double sampling_time = getDuration() / (N-1);
            double currentMaxLinearVelocity = 0.0; // maximum theoretical velocity that could be obtained
            double currentCurvilinearAbscissa = replanificationTime; // time in the old traj
            double currentScalingFactor = 0.0; // factor by which to slowdown the traj

            TrajectoryPoint tp;
            std::vector<TrajectoryPoint > newSampledTrajectory;
            
            while (currentCurvilinearAbscissa < getDuration())
            {
                // compute new trajectory point, scaling velocities
                tp = getCurrentPoint(currentCurvilinearAbscissa);
                tp.linearVelocity *= currentScalingFactor;
                tp.angularVelocity *= currentScalingFactor;
                newSampledTrajectory.push_back(tp);

                // prepare next point
                currentMaxLinearVelocity = std::min(
                    config_.maxWheelVelocity,
                    currentMaxLinearVelocity + config_.maxWheelAcceleration * sampling_time
                );
                currentScalingFactor = currentMaxLinearVelocity / config_.maxWheelVelocity;
                currentCurvilinearAbscissa += sampling_time * std::pow(currentScalingFactor, 2);
            }

            // finally, add last point to end trajectory properly
            newSampledTrajectory.push_back(getEndPoint());

            duration_ = (newSampledTrajectory.size() - 1) * sampling_time;
            sampledTrajectory_ = newSampledTrajectory;
        }

        void SampledTrajectory::removePoints(int n)
        {
            if (n <= 0)
            {
                return;
            }

            if (sampledTrajectory_.size() - n <= 0)
            {
                sampledTrajectory_.clear();
                duration_ = 0.0;
            }

            double dt = duration_ / (sampledTrajectory_.size() - 1);
            for (int i = 0; i < n; i++)
            {
                sampledTrajectory_.pop_back();
            }
            duration_ = std::max(0.0, duration_ - n * dt);
        }

        void SampledTrajectory::rescale(double scaling_factor)
        {
            std::cout << "[SampledTrajectory] rescaling by " << scaling_factor << std::endl;
            for (auto tp : sampledTrajectory_)
            {
                tp.linearVelocity *= scaling_factor;
                tp.angularVelocity *= scaling_factor;
            }
            duration_ /= scaling_factor;
        }
    }
}
