// Testing of kinematics functions.
#include <cstdlib>
#include <cmath>

#include "gtest/gtest.h"
#include "miam_utils/trajectory/ThreeWheelsKinematics.hpp"
#include "miam_utils/trajectory/DrivetrainKinematics.h"

double randf()
{
    return ((double) rand() / (RAND_MAX));
}

TEST(ThreeWheelsKinematicsTest, InverseSanityCheck)
{
    // Test ThreeWheelsKinematics inverse kinematics on simple contexts.
    double robotRadius = randf();
    double wheelRadius = randf();
    omni::ThreeWheelsKinematics kinematics(robotRadius, wheelRadius);

    omni::BaseSpeed baseSpeed(0.0, 0.0, 0.0);
    // Pure rotation: all wheel velocities should be equal to - robotRadius / wheelRadius * baseSpeed.omega_.
    baseSpeed.omega_ = randf();
    omni::WheelSpeed wheelSpeed = kinematics.inverseKinematics(baseSpeed);
    for (int i = 0; i < 3; i++)
        ASSERT_FLOAT_EQ(wheelSpeed.w_[i], robotRadius / wheelRadius * baseSpeed.omega_);
    // Y motion: wheel 0 should give -baseSpeed.vy_ / wheelRadius, other two motors should be equal and sum be zero.
    baseSpeed.vy_ = randf();
    baseSpeed.omega_ = 0;
    wheelSpeed = kinematics.inverseKinematics(baseSpeed);
    ASSERT_FLOAT_EQ(wheelSpeed.w_[0], baseSpeed.vy_ / wheelRadius);
    ASSERT_FLOAT_EQ(wheelSpeed.w_[1], - wheelSpeed.w_[0] / 2.0);
    ASSERT_FLOAT_EQ(wheelSpeed.w_[2], - wheelSpeed.w_[0] / 2.0);
    // X motion: wheel 0 does not move, other two wheels are opposite of each other.
    baseSpeed.vx_ = randf();
    baseSpeed.vy_ = 0;
    wheelSpeed = kinematics.inverseKinematics(baseSpeed);
    ASSERT_FLOAT_EQ(wheelSpeed.w_[0], 0);
    ASSERT_FLOAT_EQ(wheelSpeed.w_[1], - wheelSpeed.w_[2]);
    ASSERT_FLOAT_EQ(wheelSpeed.w_[2], 2.0 / sqrt(3.0)  * baseSpeed.vx_ / wheelRadius);
}


TEST(ThreeWheelsKinematicsTest, ForwardInverse)
{
    // Test ThreeWheelsKinematics class, making sure forward and inverse kinematics are the reciprocal of each other.
    omni::ThreeWheelsKinematics kinematics(randf(), randf());

    omni::WheelSpeed wheelSpeed(randf(), randf(), randf());
    omni::WheelSpeed convertedWheelSpeed = kinematics.inverseKinematics(kinematics.forwardKinematics(wheelSpeed));
    for (int i = 0; i < 3; i++)
        ASSERT_FLOAT_EQ(wheelSpeed.w_[i], convertedWheelSpeed.w_[i]);

    omni::BaseSpeed baseSpeed(randf(), randf(), randf());
    omni::BaseSpeed convertedBaseSpeed = kinematics.forwardKinematics(kinematics.inverseKinematics(baseSpeed));
    ASSERT_FLOAT_EQ(baseSpeed.vx_, convertedBaseSpeed.vx_);
    ASSERT_FLOAT_EQ(baseSpeed.vy_, convertedBaseSpeed.vy_);
    ASSERT_FLOAT_EQ(baseSpeed.omega_, convertedBaseSpeed.omega_);
}


TEST(DrivetrainKinematicsTest, ForwardInverse)
{
    // Test ThreeWheelsKinematics class, making sure forward and inverse kinematics are the reciprocal of each other.
    for (int i = 0; i < 50; i++)
    {
        DrivetrainKinematics kinematics(randf(), randf(), randf(), randf(), randf(), randf());

        WheelSpeed wheelSpeed(randf(), randf());
        WheelSpeed convertedWheelSpeed = kinematics.inverseKinematics(kinematics.forwardKinematics(wheelSpeed));
        ASSERT_FLOAT_EQ(wheelSpeed.right, convertedWheelSpeed.right);
        ASSERT_FLOAT_EQ(wheelSpeed.left, convertedWheelSpeed.left);

        BaseSpeed baseSpeed(randf(), randf());
        BaseSpeed convertedBaseSpeed = kinematics.forwardKinematics(kinematics.inverseKinematics(baseSpeed));
        ASSERT_FLOAT_EQ(baseSpeed.linear, convertedBaseSpeed.linear);
        ASSERT_FLOAT_EQ(baseSpeed.angular, convertedBaseSpeed.angular);
    }
}
