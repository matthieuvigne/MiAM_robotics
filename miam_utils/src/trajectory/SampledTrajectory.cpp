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

            int N = sampledTrajectory_.size();

            if (currentTime >= duration_)
            {
                return sampledTrajectory_.back();
            }
            else if (currentTime <= 0.0)
            {
                return sampledTrajectory_.front();
            }

            int indexLow = std::floor(N * currentTime / duration_);
            int indexHigh = std::ceil(N * currentTime / duration_);

            TrajectoryPoint tpLow = sampledTrajectory_[indexLow];
            TrajectoryPoint tpHigh = sampledTrajectory_[indexHigh];

            double sampledTimestep = duration_ / (N-1);

            double residue = (currentTime - indexLow * sampledTimestep) / sampledTimestep;

            double ponderationLow = 1.0 - residue;
            double ponderationHigh = residue;

            // Linear interpolation
            TrajectoryPoint output;
            output.position = ponderationLow * tpLow.position + ponderationHigh * tpHigh.position;
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

            std::vector<TrajectoryPoint > newSampledTrajectory;
            for (int i = 0; i < N + 1; i++) 
            {
                newSampledTrajectory.push_back(getCurrentPoint(replanificationTime + i * sampling_time));
            }
            
            // modify object
            duration_ = newDuration;
            sampledTrajectory_ = newSampledTrajectory;
        }
    }
}
