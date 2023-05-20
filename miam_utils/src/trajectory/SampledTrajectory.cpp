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
            // make a trapezoid in time in order to replanify the time
            // scale using a time constant which is the time to go from 0 to max velocity (600) using max acceleration (700)
            // vfin = integral (amax * dtimeconstant) = amax * timeconstant
            // timeconstant = vfin / amax
            double timeconstant = 600.0 / 700.0;

            // suppose the trajectory to be replanified goes at vmax
            // chercher le temps jusqu'au quel scaler
            // x parcouru a vmax = integral(vmax * dtime)_{0, t1} = vmax * t1

            // x parcouru avec la rampe d'acceleration = timeconstant * vmax / 2 + integral(vmax * dtime)_{timeconstant, t2}
            //                                         = timeconstant * vmax / 2 + vmax * (t2 - timeconstant) = vmax * (t2 - timeconstant / 2)

            // t1 = t2 - timeconstant / 2
            // si t1 = timeconstant : t2 = timeconstant * 1.5

            // resume :
            // trapezoide en temps montant jusqu'a timeconstant * 1.5

            double timetoincrement = timeconstant * 1.5;

            std::cout << "timetoincrement: " << timetoincrement << std::endl;

            // temps mis pour faire 



            // distance faite en vmax en timetoincrement : vmax * timetoincrement
            // distance faite avec le trapeze de vitesse : vmax * timetoincrement / 2

            // donc scaler le temps par 2 sur timetoincrement


            // get index of replanification
            TrajectoryPoint startPoint = getCurrentPoint(replanificationTime);
            int N = sampledTrajectory_.size();

            if (replanificationTime >= getDuration() || N == 1)
            {
                sampledTrajectory_.clear();
                sampledTrajectory_.push_back(startPoint);
                duration_ = 0.0;
                return;
            }

            double sampling_time = getDuration() / (N-1);

            double newDuration = getDuration() - replanificationTime + timetoincrement / 2.0;
            int newN = ceil(newDuration / sampling_time);

            std::vector<TrajectoryPoint > newSampledTrajectory;
            for (int i = 0; i < newN; i++)
            {
                double newTrajTime = i * sampling_time;
                double oldTrajTime = newTrajTime - timetoincrement / 2.0;

                // get point slower
                if (newTrajTime < timetoincrement)
                {
                    oldTrajTime = newTrajTime / 2;
                }
                TrajectoryPoint tp = getCurrentPoint(replanificationTime + oldTrajTime);

                // scale velocity
                if (newTrajTime < timetoincrement)
                {
                    tp.linearVelocity  *= newTrajTime / timetoincrement;
                    tp.angularVelocity *= newTrajTime / timetoincrement;
                } 
                newSampledTrajectory.push_back(tp);
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
