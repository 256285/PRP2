// Minimal GTest example for a line estimator
#include <cstdint>
#include <gtest/gtest.h>

#include "nodes/line_sens.hpp"
#include "line_pos.hpp"
#include <gtest/gtest.h>

TEST(LineEstimator, BasicDiscreteEstimationR) {
    const auto expected = DiscreteLinePose::LineOnRight;
    uint16_t left_value = 0;
    uint16_t right_value = 1024;
    auto result = LineEstimator::estimate_discrete(left_value, right_value);
    EXPECT_EQ(result, expected);
}

TEST(LineEstimator, BasicDiscreteEstimationL) {
    const auto expected = DiscreteLinePose::LineOnLeft;
    uint16_t left_value = 1024;
    uint16_t right_value = 0;
    auto result = LineEstimator::estimate_discrete(left_value, right_value);
    EXPECT_EQ(result, expected);
}
TEST(LineEstimator, BasicDiscreteEstimationN) {
    const auto expected = DiscreteLinePose::LineNone;
    uint16_t left_value = 0;
    uint16_t right_value = 0;
    auto result = LineEstimator::estimate_discrete(left_value, right_value);
    EXPECT_EQ(result, expected);
}

TEST(LineEstimator, BasicDiscreteEstimationB) {
    const auto expected = DiscreteLinePose::LineBoth;
    uint16_t left_value = 1024;
    uint16_t right_value = 1024;
    auto result = LineEstimator::estimate_discrete(left_value, right_value);
    EXPECT_EQ(result, expected);
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
