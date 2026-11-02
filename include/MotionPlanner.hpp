#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

class MotionPlanner {
    public:
        static constexpr std::size_t kMaxAxes = 4;
        static constexpr std::uint32_t kRateScale = 1U << 16;

        enum class Mode : std::uint8_t {
            Idle,
            Finite,
            Continuous,
            Stopping,
        };

        enum class Phase : std::uint8_t {
            Stopped,
            Accelerating,
            Running,
            Decelerating,
        };

        struct AxisSnapshot {
                Mode mode = Mode::Idle;
                Phase phase = Phase::Stopped;
                std::int64_t position = 0;
                std::int64_t target = 0;
                std::uint32_t speed = 0;
                std::uint32_t commandSpeed = 0;
                std::uint32_t acceleration = 0;
                std::uint64_t remaining = 0;
                std::uint64_t moveTotal = 0;
                std::int8_t direction = 1;
        };

        struct TickResult {
                bool step = false;
                bool directionChanged = false;
                std::int8_t direction = 1;
        };

        explicit MotionPlanner(std::uint32_t tickRateHz = 50000, std::size_t axisCount = 1)
            : tickRateHz_(tickRateHz), axisCount_(std::min(axisCount, kMaxAxes)) {}

        std::size_t axisCount() const {
            return axisCount_;
        }

        bool moveTo(std::size_t axis, std::int64_t target, std::uint32_t speed,
                    std::uint32_t acceleration) {
            if (!validCommand(axis, speed, acceleration)) {
                return false;
            }
            Axis& a = axes_[axis];
            a.target = target;
            a.moveStart = a.position;
            a.commandSpeed = speed;
            a.acceleration = acceleration;
            if (a.position == target && a.speedQ == 0) {
                a.mode = Mode::Idle;
                a.phase = Phase::Stopped;
                return true;
            }
            a.mode = Mode::Finite;
            return true;
        }

        bool moveBy(std::size_t axis, std::int64_t delta, std::uint32_t speed,
                    std::uint32_t acceleration) {
            if (axis >= axisCount_) {
                return false;
            }
            const std::int64_t position = axes_[axis].position;
            if ((delta > 0 && position > std::numeric_limits<std::int64_t>::max() - delta) ||
                (delta < 0 && position < std::numeric_limits<std::int64_t>::min() - delta)) {
                return false;
            }
            return moveTo(axis, position + delta, speed, acceleration);
        }

        bool jog(std::size_t axis, std::int8_t direction, std::uint32_t speed,
                 std::uint32_t acceleration) {
            if (!validCommand(axis, speed, acceleration) || direction == 0) {
                return false;
            }
            Axis& a = axes_[axis];
            a.requestedDirection = direction > 0 ? 1 : -1;
            a.commandSpeed = speed;
            a.acceleration = acceleration;
            a.mode = Mode::Continuous;
            return true;
        }

        bool stop(std::size_t axis) {
            if (axis >= axisCount_) {
                return false;
            }
            Axis& a = axes_[axis];
            if (a.mode != Mode::Idle) {
                a.mode = Mode::Stopping;
            }
            return true;
        }

        void immediateStop(std::size_t axis) {
            if (axis >= axisCount_) {
                return;
            }
            Axis& a = axes_[axis];
            a.mode = Mode::Idle;
            a.phase = Phase::Stopped;
            a.speedQ = 0;
            a.phaseAccumulator = 0;
            a.target = a.position;
        }

        void immediateStopAll() {
            for (std::size_t axis = 0; axis < axisCount_; ++axis) {
                immediateStop(axis);
            }
        }

        bool zero(std::size_t axis) {
            if (axis >= axisCount_ || axes_[axis].mode != Mode::Idle) {
                return false;
            }
            axes_[axis].position = 0;
            axes_[axis].target = 0;
            return true;
        }

        TickResult tick(std::size_t axis) {
            TickResult result;
            if (axis >= axisCount_) {
                return result;
            }

            Axis& a = axes_[axis];
            result.direction = a.direction;
            if (a.mode == Mode::Idle) {
                a.phase = Phase::Stopped;
                return result;
            }

            std::int8_t desiredDirection = a.direction;
            std::uint32_t desiredSpeed = 0;

            if (a.mode == Mode::Finite) {
                if (a.target == a.position) {
                    immediateStop(axis);
                    return result;
                }
                desiredDirection = a.target > a.position ? 1 : -1;
                const std::uint64_t remaining = distance(a.position, a.target);
                const std::uint64_t accelerationTwice =
                    2ULL * static_cast<std::uint64_t>(a.acceleration);
                const std::uint64_t brakingRateSquared =
                    remaining > std::numeric_limits<std::uint64_t>::max() / accelerationTwice
                        ? std::numeric_limits<std::uint64_t>::max()
                        : accelerationTwice * remaining;
                desiredSpeed = static_cast<std::uint32_t>(integerSqrt(brakingRateSquared));
                desiredSpeed = std::max<std::uint32_t>(1, std::min(a.commandSpeed, desiredSpeed));
            } else if (a.mode == Mode::Continuous) {
                desiredDirection = a.requestedDirection;
                desiredSpeed = a.commandSpeed;
            }

            const std::uint32_t increment = accelerationIncrement(a.acceleration);
            if (desiredDirection != a.direction && a.speedQ > 0) {
                a.speedQ = subtractFloor(a.speedQ, increment);
                a.phase = Phase::Decelerating;
            } else {
                if (desiredDirection != a.direction) {
                    a.direction = desiredDirection;
                    a.directionSettleTicks = 1;
                    a.phaseAccumulator = 0;
                    result.directionChanged = true;
                    result.direction = a.direction;
                }

                const std::uint64_t desiredQ =
                    static_cast<std::uint64_t>(desiredSpeed) * kRateScale;
                if (a.speedQ < desiredQ) {
                    a.speedQ = std::min<std::uint64_t>(desiredQ, a.speedQ + increment);
                    a.phase = Phase::Accelerating;
                } else if (a.speedQ > desiredQ) {
                    a.speedQ = subtractFloor(a.speedQ, increment);
                    if (a.speedQ < desiredQ) {
                        a.speedQ = desiredQ;
                    }
                    a.phase = Phase::Decelerating;
                } else {
                    a.phase = desiredSpeed == 0 ? Phase::Stopped : Phase::Running;
                }
            }

            if (a.mode == Mode::Stopping && a.speedQ == 0) {
                immediateStop(axis);
                return result;
            }
            if (a.directionSettleTicks > 0) {
                --a.directionSettleTicks;
                return result;
            }

            a.phaseAccumulator += a.speedQ;
            const std::uint64_t threshold = static_cast<std::uint64_t>(tickRateHz_) * kRateScale;
            if (a.phaseAccumulator >= threshold && a.speedQ > 0) {
                a.phaseAccumulator -= threshold;
                a.position += a.direction;
                result.step = true;
                result.direction = a.direction;
                if (a.mode == Mode::Finite && a.position == a.target) {
                    immediateStop(axis);
                }
            }

            return result;
        }

        AxisSnapshot snapshot(std::size_t axis) const {
            AxisSnapshot out;
            if (axis >= axisCount_) {
                return out;
            }
            const Axis& a = axes_[axis];
            out.mode = a.mode;
            out.phase = a.phase;
            out.position = a.position;
            out.target = a.target;
            out.speed = static_cast<std::uint32_t>((a.speedQ + kRateScale / 2) / kRateScale);
            out.commandSpeed = a.commandSpeed;
            out.acceleration = a.acceleration;
            out.direction = a.direction;
            out.remaining = a.mode == Mode::Finite ? distance(a.position, a.target) : 0;
            out.moveTotal = a.mode == Mode::Finite ? distance(a.moveStart, a.target) : 0;
            return out;
        }

    private:
        struct Axis {
                Mode mode = Mode::Idle;
                Phase phase = Phase::Stopped;
                std::int64_t position = 0;
                std::int64_t target = 0;
                std::int64_t moveStart = 0;
                std::uint64_t speedQ = 0;
                std::uint64_t phaseAccumulator = 0;
                std::uint32_t commandSpeed = 0;
                std::uint32_t acceleration = 1;
                std::int8_t direction = 1;
                std::int8_t requestedDirection = 1;
                std::uint8_t directionSettleTicks = 0;
        };

        std::uint32_t tickRateHz_;
        std::size_t axisCount_;
        std::array<Axis, kMaxAxes> axes_ {};

        bool validCommand(std::size_t axis, std::uint32_t speed, std::uint32_t acceleration) const {
            return axis < axisCount_ && speed > 0 && speed <= tickRateHz_ / 2 && acceleration > 0;
        }

        std::uint32_t accelerationIncrement(std::uint32_t acceleration) const {
            const std::uint64_t scaled = static_cast<std::uint64_t>(acceleration) * kRateScale;
            return static_cast<std::uint32_t>(std::max<std::uint64_t>(1, scaled / tickRateHz_));
        }

        static std::uint64_t subtractFloor(std::uint64_t value, std::uint64_t amount) {
            return value > amount ? value - amount : 0;
        }

        static std::uint64_t distance(std::int64_t from, std::int64_t to) {
            return to >= from ? static_cast<std::uint64_t>(to) - static_cast<std::uint64_t>(from)
                              : static_cast<std::uint64_t>(from) - static_cast<std::uint64_t>(to);
        }

        static std::uint64_t integerSqrt(std::uint64_t value) {
            std::uint64_t result = 0;
            std::uint64_t bit = 1ULL << 62;
            while (bit > value) {
                bit >>= 2;
            }
            while (bit != 0) {
                if (value >= result + bit) {
                    value -= result + bit;
                    result = (result >> 1) + bit;
                } else {
                    result >>= 1;
                }
                bit >>= 2;
            }
            return result;
        }
};
