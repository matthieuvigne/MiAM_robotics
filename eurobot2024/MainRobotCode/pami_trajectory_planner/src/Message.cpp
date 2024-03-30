#include <Message.hpp>
#include <miam_utils/trajectory/SampledTrajectory.h>
#include <cmath>

#define MESSAGE_PAYLOAD_START 1

#define TRAJECTORY_SERIALIZATION_DELTAT 0.1

using namespace miam::trajectory;

std::shared_ptr<Message > Message::parse(std::vector<float > message, uint8_t senderId)
{
    // Message type is the first float casted to int
    int message_type = (int)message.at(0);

    // // Sender is the second float casted to int
    // int sender_id = (int) message.at(1);

    if (message_type == MessageType::CONFIGURATION)
    {
        // Check size of payload
        if (message.size() - MESSAGE_PAYLOAD_START == 1)
        {
            // First byte of payload is configuration
            if ((bool)message.at(MESSAGE_PAYLOAD_START) == PlayingSide::BLUE_SIDE)
            {
                return std::make_shared<ConfigurationMessage >(
                    PlayingSide::BLUE_SIDE,
                    senderId
                );
            }
            else if ((bool)message.at(MESSAGE_PAYLOAD_START) == PlayingSide::YELLOW_SIDE)
            {
                return std::make_shared<ConfigurationMessage >(
                    PlayingSide::YELLOW_SIDE,
                    senderId
                );
            }
        }
    }
    else if (message_type == MessageType::MATCH_STATE)
    {
        // Check size of payload
        if (message.size() - MESSAGE_PAYLOAD_START == 2)
        {
            // First byte of payload is match started
            // Second byte of payload is match time
            return std::make_shared<MatchStateMessage >(
                (bool) message.at(MESSAGE_PAYLOAD_START),
                (float) message.at(MESSAGE_PAYLOAD_START + 1),
                senderId
            );
        }
    }
    else if (message_type == MessageType::NEW_TRAJECTORY)
    {
        if (message.size() - MESSAGE_PAYLOAD_START >= 2)
        {
            // Message should be size >= 2
            int size_of_trajectory = (int)message.at(MESSAGE_PAYLOAD_START);
            float duration_of_trajectory = (float)message.at(MESSAGE_PAYLOAD_START + 1);

            int expected_size = size_of_trajectory * 5;

            // Payload size is initial size - header - 2 first floats
            int trajectory_payload_start = MESSAGE_PAYLOAD_START + 2;

            if (expected_size != (message.size() - trajectory_payload_start))
            {
                std::vector<TrajectoryPoint > trajectoryPoints;

                for (int i = 0; i < size_of_trajectory; i++)
                {
                    TrajectoryPoint tp;
                    tp.position.x = message.at(trajectory_payload_start + 5*i);
                    tp.position.y = message.at(trajectory_payload_start + 5*i + 1);
                    tp.position.theta = message.at(trajectory_payload_start + 5*i + 2);
                    tp.linearVelocity = message.at(trajectory_payload_start + 5*i + 3);
                    tp.angularVelocity = message.at(trajectory_payload_start + 5*i + 4);
                    trajectoryPoints.push_back(tp);
                }

                TrajectoryConfig tc;
                std::shared_ptr<SampledTrajectory > traj(new SampledTrajectory(tc, trajectoryPoints, duration_of_trajectory));
                TrajectoryVector newTrajectory;
                newTrajectory.push_back(traj);

                return std::make_shared<NewTrajectoryMessage >(
                    newTrajectory,
                    senderId
                );
            }
        }
    }
    else if (message_type == MessageType::PAMI_REPORT)
    {
        // Check size of payload
        if (message.size() - MESSAGE_PAYLOAD_START == 3)
        {
            // First byte of payload is match started
            // Second byte of payload is match time
            // 3rd byte is side
            if ((bool)message.at(MESSAGE_PAYLOAD_START+2) == PlayingSide::BLUE_SIDE)
            {
                return std::make_shared<PamiReportMessage >(
                    (bool) message.at(MESSAGE_PAYLOAD_START),
                    (float) message.at(MESSAGE_PAYLOAD_START + 1),
                    PlayingSide::BLUE_SIDE,
                    senderId
                );
            }
            else if ((bool)message.at(MESSAGE_PAYLOAD_START+2) == PlayingSide::YELLOW_SIDE)
            {
                // First byte of payload is match started
                // Second byte of payload is match time
                return std::make_shared<PamiReportMessage >(
                    (bool) message.at(MESSAGE_PAYLOAD_START),
                    (float) message.at(MESSAGE_PAYLOAD_START + 1),
                    PlayingSide::YELLOW_SIDE,
                    senderId
                );
            }
        }
    }
    return std::make_shared<ErrorMessage >(senderId);
}

// function definitions
VecFloat ConfigurationMessage::serialize()
{
    VecFloat res;
    // First byte is message type
    res.push_back((float)get_message_type());
    // Second byte is side
    res.push_back((float)playingSide_);
    return res;
}

VecFloat NewTrajectoryMessage::serialize()
{
    VecFloat res;
    // First byte is message type
    res.push_back((float)get_message_type());

    // Serializing the trajectory:
    // N+1 = Number of points is duration / TRAJECTORY_SERIALIZATION_DELTAT + 1
    // N = number of time intervals
    int N = std::ceil(newTrajectory_.getDuration() / TRAJECTORY_SERIALIZATION_DELTAT);
    int deltat = newTrajectory_.getDuration() / N;

    // Second byte is size of trajectory in number of points
    res.push_back((float)(N+1));
    // Third byte is duration
    res.push_back((float)newTrajectory_.getDuration());
    // Following bytes are trajectory points
    for (int i = 0; i < N+1; i++)
    {
        TrajectoryPoint pt;
        if (i < N)
        {
            pt = newTrajectory_.getCurrentPoint(deltat * i);
        } else {
            pt = newTrajectory_.getEndPoint();
        }
        res.push_back((float)pt.position.x);
        res.push_back((float)pt.position.y);
        res.push_back((float)pt.position.theta);
        res.push_back((float)pt.linearVelocity);
        res.push_back((float)pt.angularVelocity);
    }
    return res;
}

VecFloat MatchStateMessage::serialize()
{
    VecFloat res;
    // First byte is message type
    res.push_back((float)get_message_type());
    // Second byte is match started
    res.push_back((float)matchStarted_);
    // Third byte is match time
    res.push_back((float)matchTime_);
    return res;
}

VecFloat ErrorMessage::serialize()
{
    VecFloat res;
    // First byte is message type
    res.push_back((float)get_message_type());
    return res;
}

VecFloat PamiReportMessage::serialize()
{
    VecFloat res;
    // First byte is message type
    res.push_back((float)get_message_type());
    // Second byte is match started
    res.push_back((float)matchStarted_);
    // Third byte is match time
    res.push_back((float)matchTime_);
    // 4th byte is playing side
    res.push_back((float)playingSide_);
    return res;
}