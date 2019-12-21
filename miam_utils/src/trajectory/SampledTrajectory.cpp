/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/SampledTrajectory.h"
#include <cmath>


namespace miam{
    namespace trajectory{
        SampledTrajectory::SampledTrajectory(
            std::vector<TrajectoryPoint > sampledTrajectory,
            double duration
            ) : Trajectory()
        {
            duration_ = duration;
            sampledTrajectory_ = sampledTrajectory;
        }

        TrajectoryPoint SampledTrajectory::getCurrentPoint(double const& currentTime)
        {

            int N = sampledTrajectory_.size() - 1;

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

            double sampledTimestep = duration_ / N;

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
    }
}
