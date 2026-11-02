#pragma once

#include "AppConfig.hpp"
#include "MotionEngine.hpp"

#include <Arduino.h>
#include <array>
#include <cstdint>

class CycleController {
    public:
        enum class Phase : std::uint8_t {
            Idle,
            MovingOut,
            DwellingOut,
            MovingBack,
            DwellingBack,
        };

        struct CycleSnapshot {
                bool active = false;
                bool infinite = false;
                Phase phase = Phase::Idle;
                std::uint32_t completed = 0;
                std::uint32_t repeats = 0;
                std::uint32_t elapsedMs = 0;
        };

        explicit CycleController(MotionEngine& engine) : engine_(engine) {}

        bool start(std::size_t axis, std::uint64_t travelSteps, std::int8_t initialDirection,
                   std::uint32_t speed, std::uint32_t acceleration, std::uint32_t dwellMs,
                   std::uint32_t repeats, bool infinite, std::uint32_t now) {
            if (axis >= engine_.axisCount() || travelSteps == 0 ||
                travelSteps > static_cast<std::uint64_t>(INT64_MAX) || speed == 0 ||
                acceleration == 0 || (!infinite && repeats == 0)) {
                return false;
            }

            stop(axis, true);
            auto& cycle = cycles_[axis];
            const auto motion = engine_.snapshot(axis);
            const std::int64_t signedTravel = initialDirection >= 0
                                                  ? static_cast<std::int64_t>(travelSteps)
                                                  : -static_cast<std::int64_t>(travelSteps);
            if ((signedTravel > 0 && motion.position > INT64_MAX - signedTravel) ||
                (signedTravel < 0 && motion.position < INT64_MIN - signedTravel)) {
                return false;
            }

            cycle.active = true;
            cycle.infinite = infinite;
            cycle.phase = Phase::MovingOut;
            cycle.startPosition = motion.position;
            cycle.outPosition = motion.position + signedTravel;
            cycle.speed = speed;
            cycle.acceleration = acceleration;
            cycle.dwellMs = dwellMs;
            cycle.repeats = repeats;
            cycle.completed = 0;
            cycle.startedAt = now;
            cycle.deadline = 0;
            return engine_.moveTo(axis, cycle.outPosition, speed, acceleration);
        }

        void stop(std::size_t axis, bool immediate = false) {
            if (axis >= engine_.axisCount()) {
                return;
            }
            cycles_[axis] = Cycle {};
            if (immediate) {
                // Replacing a cycle with another command should not leave its old
                // target active. A controlled user stop is handled by the caller.
                engine_.stop(axis);
            }
        }

        void stopAll() {
            for (std::size_t axis = 0; axis < engine_.axisCount(); ++axis) {
                cycles_[axis] = Cycle {};
            }
        }

        bool active(std::size_t axis) const {
            return axis < engine_.axisCount() && cycles_[axis].active;
        }

        void update(std::uint32_t now) {
            for (std::size_t axis = 0; axis < engine_.axisCount(); ++axis) {
                updateAxis(axis, now);
            }
        }

        CycleSnapshot snapshot(std::size_t axis, std::uint32_t now) const {
            CycleSnapshot value;
            if (axis >= engine_.axisCount()) {
                return value;
            }
            const auto& cycle = cycles_[axis];
            value.active = cycle.active;
            value.infinite = cycle.infinite;
            value.phase = cycle.phase;
            value.completed = cycle.completed;
            value.repeats = cycle.repeats;
            value.elapsedMs = cycle.active ? now - cycle.startedAt : 0;
            return value;
        }

    private:
        struct Cycle {
                bool active = false;
                bool infinite = false;
                Phase phase = Phase::Idle;
                std::int64_t startPosition = 0;
                std::int64_t outPosition = 0;
                std::uint32_t speed = 0;
                std::uint32_t acceleration = 0;
                std::uint32_t dwellMs = 0;
                std::uint32_t repeats = 0;
                std::uint32_t completed = 0;
                std::uint32_t startedAt = 0;
                std::uint32_t deadline = 0;
        };

        MotionEngine& engine_;
        std::array<Cycle, AppConfig::kMaxAxes> cycles_ {};

        static bool due(std::uint32_t now, std::uint32_t deadline) {
            return static_cast<std::int32_t>(now - deadline) >= 0;
        }

        void updateAxis(std::size_t axis, std::uint32_t now) {
            auto& cycle = cycles_[axis];
            if (!cycle.active) {
                return;
            }

            switch (cycle.phase) {
                case Phase::MovingOut:
                    if (engine_.isIdle(axis)) {
                        cycle.phase = Phase::DwellingOut;
                        cycle.deadline = now + cycle.dwellMs;
                    }
                    break;
                case Phase::DwellingOut:
                    if (due(now, cycle.deadline)) {
                        cycle.phase = Phase::MovingBack;
                        engine_.moveTo(axis, cycle.startPosition, cycle.speed, cycle.acceleration);
                    }
                    break;
                case Phase::MovingBack:
                    if (engine_.isIdle(axis)) {
                        ++cycle.completed;
                        if (!cycle.infinite && cycle.completed >= cycle.repeats) {
                            cycle.active = false;
                            cycle.phase = Phase::Idle;
                        } else {
                            cycle.phase = Phase::DwellingBack;
                            cycle.deadline = now + cycle.dwellMs;
                        }
                    }
                    break;
                case Phase::DwellingBack:
                    if (due(now, cycle.deadline)) {
                        cycle.phase = Phase::MovingOut;
                        engine_.moveTo(axis, cycle.outPosition, cycle.speed, cycle.acceleration);
                    }
                    break;
                case Phase::Idle:
                    break;
            }
        }
};
