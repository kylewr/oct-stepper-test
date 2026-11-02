#include <unity.h>

#include "MotionPlanner.hpp"

#include <cstddef>
#include <cstdint>

namespace {

constexpr std::uint32_t kTicksPerSecond = 50000;

void runUntilIdle(MotionPlanner &planner, std::size_t axis,
                  std::uint32_t maximumTicks = 1000000) {
    for (std::uint32_t tick = 0; tick < maximumTicks; ++tick) {
        planner.tick(axis);
        if (planner.snapshot(axis).mode == MotionPlanner::Mode::Idle) return;
    }
    TEST_FAIL_MESSAGE("motion did not become idle before test timeout");
}

void test_signed_finite_moves_finish_exactly() {
    MotionPlanner planner(kTicksPerSecond, 1);
    TEST_ASSERT_TRUE(planner.moveBy(0, 137, 1200, 4000));
    runUntilIdle(planner, 0);
    TEST_ASSERT_EQUAL_INT64(137, planner.snapshot(0).position);

    TEST_ASSERT_TRUE(planner.moveBy(0, -289, 1600, 5000));
    runUntilIdle(planner, 0);
    TEST_ASSERT_EQUAL_INT64(-152, planner.snapshot(0).position);

    TEST_ASSERT_TRUE(planner.moveTo(0, 42, 800, 3000));
    runUntilIdle(planner, 0);
    TEST_ASSERT_EQUAL_INT64(42, planner.snapshot(0).position);
}

void test_four_axes_are_independent() {
    MotionPlanner planner(kTicksPerSecond, 4);
    TEST_ASSERT_TRUE(planner.moveTo(0, 100, 500, 2000));
    TEST_ASSERT_TRUE(planner.moveTo(1, -240, 1000, 3500));
    TEST_ASSERT_TRUE(planner.moveTo(2, 17, 150, 1000));
    TEST_ASSERT_TRUE(planner.moveTo(3, -83, 700, 2400));

    for (std::uint32_t tick = 0; tick < 1000000; ++tick) {
        planner.tick(0);
        planner.tick(1);
        planner.tick(2);
        planner.tick(3);
        if (planner.snapshot(0).mode == MotionPlanner::Mode::Idle &&
            planner.snapshot(1).mode == MotionPlanner::Mode::Idle &&
            planner.snapshot(2).mode == MotionPlanner::Mode::Idle &&
            planner.snapshot(3).mode == MotionPlanner::Mode::Idle) break;
    }

    TEST_ASSERT_EQUAL_INT64(100, planner.snapshot(0).position);
    TEST_ASSERT_EQUAL_INT64(-240, planner.snapshot(1).position);
    TEST_ASSERT_EQUAL_INT64(17, planner.snapshot(2).position);
    TEST_ASSERT_EQUAL_INT64(-83, planner.snapshot(3).position);
}

void test_jog_reverses_and_stops_under_control() {
    MotionPlanner planner(kTicksPerSecond, 1);
    TEST_ASSERT_TRUE(planner.jog(0, 1, 1000, 5000));
    for (std::uint32_t tick = 0; tick < kTicksPerSecond / 2; ++tick) {
        planner.tick(0);
    }
    const auto positivePosition = planner.snapshot(0).position;
    TEST_ASSERT_GREATER_THAN_INT64(0, positivePosition);

    TEST_ASSERT_TRUE(planner.jog(0, -1, 1000, 5000));
    for (std::uint32_t tick = 0; tick < kTicksPerSecond; ++tick) {
        planner.tick(0);
    }
    TEST_ASSERT_LESS_THAN_INT64(positivePosition, planner.snapshot(0).position);

    TEST_ASSERT_TRUE(planner.stop(0));
    runUntilIdle(planner, 0);
    TEST_ASSERT_EQUAL_UINT32(0, planner.snapshot(0).speed);
}

void test_zero_and_command_validation() {
    MotionPlanner planner(kTicksPerSecond, 1);
    TEST_ASSERT_FALSE(planner.moveBy(1, 10, 100, 100));
    TEST_ASSERT_FALSE(planner.moveBy(0, 10, 0, 100));
    TEST_ASSERT_FALSE(planner.moveBy(0, 10, 26000, 100));
    TEST_ASSERT_FALSE(planner.moveBy(0, 10, 100, 0));

    TEST_ASSERT_TRUE(planner.moveBy(0, 20, 200, 1000));
    TEST_ASSERT_FALSE(planner.zero(0));
    runUntilIdle(planner, 0);
    TEST_ASSERT_TRUE(planner.zero(0));
    TEST_ASSERT_EQUAL_INT64(0, planner.snapshot(0).position);
}

void test_maximum_rate_never_emits_more_than_requested() {
    MotionPlanner planner(kTicksPerSecond, 1);
    TEST_ASSERT_TRUE(planner.jog(0, 1, 10000, 1000000));
    std::uint32_t pulses = 0;
    for (std::uint32_t tick = 0; tick < kTicksPerSecond; ++tick) {
        if (planner.tick(0).step) ++pulses;
    }
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(10000, pulses);
    TEST_ASSERT_GREATER_THAN_UINT32(9000, pulses);
}

void test_progress_total_is_stable() {
    MotionPlanner planner(kTicksPerSecond, 1);
    TEST_ASSERT_TRUE(planner.moveTo(0, 500, 1000, 4000));
    for (int tick = 0; tick < 2000; ++tick) planner.tick(0);
    const auto state = planner.snapshot(0);
    TEST_ASSERT_EQUAL_UINT64(500, state.moveTotal);
    TEST_ASSERT_EQUAL_UINT64(500 - state.position, state.remaining);
}

} // namespace

void setUp() {}
void tearDown() {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_signed_finite_moves_finish_exactly);
    RUN_TEST(test_four_axes_are_independent);
    RUN_TEST(test_jog_reverses_and_stops_under_control);
    RUN_TEST(test_zero_and_command_validation);
    RUN_TEST(test_maximum_rate_never_emits_more_than_requested);
    RUN_TEST(test_progress_total_is_stable);
    return UNITY_END();
}
