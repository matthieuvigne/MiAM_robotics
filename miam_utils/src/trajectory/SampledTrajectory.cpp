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
            // get index of replanification
            TrajectoryPoint startPoint = getCurrentPoint(replanificationTime);

            if (replanificationTime >= getDuration())
            {
                sampledTrajectory_.clear();
                sampledTrajectory_.push_back(startPoint);
                duration_ = 0.0;
                return;
            }

            int N = sampledTrajectory_.size();
            double sampling_time = getDuration() / (N-1);

            double newDuration = getDuration() - replanificationTime;
            int newN = newDuration / sampling_time + 1;

            std::vector<TrajectoryPoint > newSampledTrajectory;
            for (int i = 0; i < newN; i++) 
            {
                newSampledTrajectory.push_back(getCurrentPoint(replanificationTime + i * sampling_time));
            }
            
            // modify object
            duration_ = newDuration;
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
    }
}
