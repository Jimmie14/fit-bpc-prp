// Minimal GTest example for a line estimator
#include <cstdint>
#include <gtest/gtest.h>

#include "LineEstimator.hpp"

TEST(LineEstimator, BasicDiscreteEstimation)
{
    uint16_t left_value = 0;
    uint16_t right_value = 1024;
    auto result = LineEstimator::EstimateDiscrete(left_value, right_value);
    EXPECT_EQ(result, DiscreteLinePose::LineOnRight);
}

TEST(LineEstimator, BasicContinuousEstimation)
{
    uint16_t left_value = 0;
    uint16_t right_value = 1024;
    auto result = LineEstimator::EstimateContinuousLinePose(left_value, right_value);
    EXPECT_EQ(result, 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
