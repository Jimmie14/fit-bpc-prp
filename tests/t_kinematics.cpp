#include <gtest/gtest.h>
#include "Kinematics.hpp"
#include <cmath>

constexpr float ERROR = 0.001;
constexpr float WHEEL_BASE = 0.12;
constexpr float WHEEL_RADIUS = 0.033;
constexpr float WHEEL_CIRCUMFERENCE = 2 * M_PI * WHEEL_RADIUS;
constexpr int32_t PULSES_PER_ROTATION = 550;

TEST(KinematicsTest, BackwardZeroVelocitySI) {
    // Arrange
    const float linear = 0;
    const float angular = 0;
    const float expected_l = 0;
    const float expected_r = 0;

    auto kinematics = Kinematics(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);

    // Act
    auto result = kinematics.inverse(RobotSpeed {linear, angular});

    // Assert
    EXPECT_NEAR(result.left, expected_l, ERROR);
    EXPECT_NEAR(result.right, expected_r, ERROR);
}

TEST(KinematicsTest, BackwardPositiveLinearVelocitySI) {
    // Arrange
    const auto robotSpeed = RobotSpeed(1.0, 0.0);
    const float expected_l = 1.0 / WHEEL_CIRCUMFERENCE * 2 * M_PI;
    const float expected_r = 1.0 / WHEEL_CIRCUMFERENCE * 2 * M_PI;

    auto kinematics = Kinematics(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);

    // Act
    auto result = kinematics.inverse(robotSpeed);

    // Assert
    EXPECT_NEAR(result.left, expected_l, ERROR);
    EXPECT_NEAR(result.right, expected_r, ERROR);
}

TEST(KinematicsTest, BackwardPositiveAngularVelocitySI) {
    // Arrange
    constexpr auto robotSpeed = RobotSpeed(0, 1.0);
    constexpr float expectedLeft = -(0.5 * WHEEL_BASE) / WHEEL_CIRCUMFERENCE * (2 * M_PI);
    constexpr float expectedRight = +(0.5 * WHEEL_BASE) / WHEEL_CIRCUMFERENCE * (2 * M_PI);

    auto kinematics = Kinematics(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);

    // Act
    auto result = kinematics.inverse(robotSpeed);

    // Assert
    EXPECT_NEAR(result.left, expectedLeft, ERROR);
    EXPECT_NEAR(result.right, expectedRight, ERROR);
}

TEST(KinematicsTest, ForwardZeroWheelSpeedSI) {
    // Arrange
    constexpr auto wheelSpeed = WheelSpeed(0, 0);
    constexpr auto expected_linear = 0.0;
    constexpr auto expected_angular = 0.0;

    auto kinematics = Kinematics(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);

    // Act
    auto result = kinematics.forward(wheelSpeed);

    // Assert
    EXPECT_NEAR(result.linear, expected_linear, ERROR);
    EXPECT_NEAR(result.angular, expected_angular, ERROR);
}

TEST(KinematicsTest, ForwardEqualWheelSpeedsSI) {
    // Arrange
    constexpr auto wheelSpeed = WheelSpeed(1, 1);
    constexpr auto expected_l = WHEEL_RADIUS;
    constexpr auto expected_a= 0.0;

    auto kinematics = Kinematics(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);

    // Act
    auto result = kinematics.forward(wheelSpeed);

    // Assert
    EXPECT_NEAR(result.linear, expected_l, ERROR);
    EXPECT_NEAR(result.angular, expected_a, ERROR);
}

TEST(KinematicsTest, ForwardOppositeWheelSpeedsSI) {
    // Arrange
    constexpr auto wheelSpeed = WheelSpeed(-1, 1);
    constexpr auto expectedLinear = 0;
    constexpr auto expectedAngular= (WHEEL_RADIUS / (0.5 * WHEEL_BASE));

    auto kinematics = Kinematics(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);

    // Act
    auto result = kinematics.forward(wheelSpeed);

    // Assert
    EXPECT_NEAR(result.linear, expectedLinear, ERROR);
    EXPECT_NEAR(result.angular, expectedAngular, ERROR);;
}

TEST(KinematicsTest, ForwardAndBackwardSI) {
    // Arrange
    constexpr auto wheelSpeed = WheelSpeed(1, -0.5);

    const auto kinematics = Kinematics(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);

    // Act
    auto robotSpeed = kinematics.forward(wheelSpeed);
    auto result = kinematics.inverse(robotSpeed);

    // Assert
    EXPECT_NEAR(result.left, wheelSpeed.left, ERROR);
    EXPECT_NEAR(result.right, wheelSpeed.right, ERROR);
}


TEST(KinematicsTest, ForwardAndBackwardEncoderDiff) {
    // Arrange
    constexpr auto encoders = Encoders(0, 550);

    const auto kinematics = Kinematics(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);

    // Act
    auto coord = kinematics.forward(encoders);
    auto result = kinematics.inverse(coord);

    // Assert
    EXPECT_NEAR(result.left, encoders.left, 1);
    EXPECT_NEAR(result.right, encoders.right, 1);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
