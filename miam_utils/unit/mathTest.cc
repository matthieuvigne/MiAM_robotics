// Testing of kinematics functions.
#include <cstdlib>
#include <cmath>

#include "gtest/gtest.h"
#include "miam_utils/MathUtils.h"


TEST(MathUtilsTest, unwrap)
{
    double angle = 0.0;
    double increment = 0.1;
    for (int i = 0; i < 100; i++)
    {
        double next = std::fmod(angle + increment, 2 * M_PI);
        double uangle = unwrap(angle, next, 2 * M_PI);
        angle += increment;
        ASSERT_FLOAT_EQ(angle, uangle);
    }
    angle = 1.5;
    for (int i = 0; i < 100; i++)
    {
        double next = std::fmod(angle - increment, 2 * M_PI);
        double uangle = unwrap(angle, next, 2 * M_PI);
        angle -= increment;
        ASSERT_FLOAT_EQ(angle, uangle);
    }

    // Test unwrap without option: 2pi by default
    angle = 0.055;
    for (int i = 0; i < 100; i++)
    {
        double next = std::fmod(angle - increment, 2 * M_PI);
        double uangle = unwrap(angle, next);
        angle -= increment;
        ASSERT_FLOAT_EQ(angle, uangle);
    }

}
