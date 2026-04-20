// Minimal GTest example for a line estimator
#include <cstdint>
#include <gtest/gtest.h>

#include "LineEstimator.hpp"

TEST(LineEstimator, BasicDiscreteEstimation)
{
    // Arrange
    uint16_t left_value = 0;
    uint16_t right_value = 1024;

    LineEstimator estimator(1024, 0);

    // Act
    auto result = estimator.EstimateDiscrete(left_value, right_value);

    // Assert
    EXPECT_EQ(result, DiscreteLinePose::LineOnRight);
}

TEST(LineEstimator, BasicContinuousEstimation)
{
    // Arrange
    uint16_t left_value = 0;
    uint16_t right_value = 1024;
    LineEstimator estimator(1024, 0);

    // Act
    auto result = estimator.EstimateContinuousLinePose(left_value, right_value);

    // Assert
    EXPECT_EQ(result, 0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
