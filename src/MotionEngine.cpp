#include "MotionEngine.hpp"

#if MOTION_BACKEND_UART

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <limits>

MotionEngine::MotionEngine() = default;

bool MotionEngine::begin(const HardwareConfig &hardware) {
    hardware_ = hardware;
    bridgeSerial_.begin(AppConfig::kBridgeUartBaud, SERIAL_8N1,
                        AppConfig::kBridgeRxPin, AppConfig::kBridgeTxPin);
    delay(20);
    sendCommand("E 0\n");
    sendCommand("Q\n");
    return true;
}

void MotionEngine::update() {
    while (bridgeSerial_.available() > 0) {
        const char value = static_cast<char>(bridgeSerial_.read());
        if (value == '\n') {
            bridgeLine_[bridgeLineLength_] = '\0';
            processBridgeLine(bridgeLine_);
            bridgeLineLength_ = 0;
        } else if (value != '\r') {
            if (bridgeLineLength_ + 1 < sizeof(bridgeLine_)) {
                bridgeLine_[bridgeLineLength_++] = value;
            } else {
                bridgeLineLength_ = 0;
            }
        }
    }
}

std::size_t MotionEngine::axisCount() const {
    return hardware_.axisCount;
}

const HardwareConfig &MotionEngine::hardwareConfig() const {
    return hardware_;
}

bool MotionEngine::moveTo(std::size_t axis, std::int64_t target,
                          std::uint32_t speed, std::uint32_t acceleration) {
    if (axis >= axisCount() || target < std::numeric_limits<std::int32_t>::min() ||
        target > std::numeric_limits<std::int32_t>::max() || speed == 0) {
        return false;
    }
    enableDrivers();
    auto &state = remoteStates_[axis];
    state.mode = MotionPlanner::Mode::Finite;
    state.phase = MotionPlanner::Phase::Running;
    state.target = target;
    state.moveTotal = target >= state.position
                          ? static_cast<std::uint64_t>(target - state.position)
                          : static_cast<std::uint64_t>(state.position - target);
    state.remaining = state.moveTotal;
    state.commandSpeed = speed;
    state.acceleration = acceleration;
    sendCommand("M %u %ld %lu %lu\n", static_cast<unsigned>(axis),
                static_cast<long>(target), static_cast<unsigned long>(speed),
                static_cast<unsigned long>(acceleration));
    return true;
}

bool MotionEngine::moveBy(std::size_t axis, std::int64_t delta,
                          std::uint32_t speed, std::uint32_t acceleration) {
    if (axis >= axisCount()) {
        return false;
    }
    const std::int64_t position = remoteStates_[axis].position;
    if ((delta > 0 && position > std::numeric_limits<std::int64_t>::max() - delta) ||
        (delta < 0 && position < std::numeric_limits<std::int64_t>::min() - delta)) {
        return false;
    }
    return moveTo(axis, position + delta, speed, acceleration);
}

bool MotionEngine::jog(std::size_t axis, std::int8_t direction,
                       std::uint32_t speed, std::uint32_t acceleration) {
    if (axis >= axisCount() || (direction != -1 && direction != 1) || speed == 0) {
        return false;
    }
    enableDrivers();
    auto &state = remoteStates_[axis];
    state.mode = MotionPlanner::Mode::Continuous;
    state.phase = MotionPlanner::Phase::Running;
    state.direction = direction;
    state.commandSpeed = speed;
    state.acceleration = acceleration;
    sendCommand("J %u %d %lu %lu\n", static_cast<unsigned>(axis),
                static_cast<int>(direction), static_cast<unsigned long>(speed),
                static_cast<unsigned long>(acceleration));
    return true;
}

bool MotionEngine::stop(std::size_t axis) {
    if (axis >= axisCount()) {
        return false;
    }
    sendCommand("S %u\n", static_cast<unsigned>(axis));
    auto &state = remoteStates_[axis];
    state.mode = MotionPlanner::Mode::Idle;
    state.phase = MotionPlanner::Phase::Stopped;
    state.speed = 0;
    state.target = state.position;
    state.remaining = 0;
    return true;
}

void MotionEngine::immediateStopAll() {
    for (std::size_t axis = 0; axis < axisCount(); ++axis) {
        stop(axis);
    }
}

bool MotionEngine::zero(std::size_t axis) {
    if (axis >= axisCount() || !isIdle(axis)) {
        return false;
    }
    sendCommand("Z %u\n", static_cast<unsigned>(axis));
    remoteStates_[axis].position = 0;
    remoteStates_[axis].target = 0;
    referenced_[axis] = true;
    return true;
}

MotionPlanner::AxisSnapshot MotionEngine::snapshot(std::size_t axis) const {
    return axis < axisCount() ? remoteStates_[axis] : MotionPlanner::AxisSnapshot{};
}

bool MotionEngine::isIdle(std::size_t axis) const {
    return snapshot(axis).mode == MotionPlanner::Mode::Idle;
}

bool MotionEngine::isContinuous(std::size_t axis) const {
    return snapshot(axis).mode == MotionPlanner::Mode::Continuous;
}

void MotionEngine::enableDrivers() {
    if (!driversEnabled_) {
        sendCommand("E 1\n");
        driversEnabled_ = true;
    }
}

void MotionEngine::disableDrivers() {
    sendCommand("E 0\n");
    driversEnabled_ = false;
    for (std::size_t axis = 0; axis < axisCount(); ++axis) {
        auto &state = remoteStates_[axis];
        state.mode = MotionPlanner::Mode::Idle;
        state.phase = MotionPlanner::Phase::Stopped;
        state.speed = 0;
        state.target = state.position;
        state.remaining = 0;
    }
    referenced_.fill(false);
}

bool MotionEngine::driversEnabled() const {
    return driversEnabled_;
}

void MotionEngine::setDirectionInverted(std::size_t axis, bool inverted) {
    if (axis >= axisCount()) {
        return;
    }
    directionInverted_[axis] = inverted;
    sendCommand("I %u %u\n", static_cast<unsigned>(axis), inverted ? 1U : 0U);
}

bool MotionEngine::positionReferenced(std::size_t axis) const {
    return axis < axisCount() && referenced_[axis];
}

void MotionEngine::sendCommand(const char *format, ...) {
    char command[128];
    va_list arguments;
    va_start(arguments, format);
    const int length = vsnprintf(command, sizeof(command), format, arguments);
    va_end(arguments);
    if (length > 0) {
        bridgeSerial_.write(reinterpret_cast<const std::uint8_t *>(command),
                            static_cast<std::size_t>(std::min<int>(length, sizeof(command) - 1)));
    }
}

void MotionEngine::processBridgeLine(const char *line) {
    if (line[0] == 'H') {
        for (std::size_t axis = 0; axis < axisCount(); ++axis) {
            sendCommand("I %u %u\n", static_cast<unsigned>(axis),
                        directionInverted_[axis] ? 1U : 0U);
        }
        sendCommand("E %u\n", driversEnabled_ ? 1U : 0U);
        sendCommand("Q\n");
        return;
    }
    if (line[0] != 'T') {
        return;
    }

    unsigned axis = 0;
    unsigned enabled = 0;
    unsigned mode = 0;
    unsigned phase = 0;
    long position = 0;
    long target = 0;
    unsigned long speed = 0;
    unsigned long remaining = 0;
    unsigned long total = 0;
    int direction = 1;
    if (sscanf(line, "T %u %u %u %u %ld %ld %lu %lu %lu %d", &axis, &enabled,
               &mode, &phase, &position, &target, &speed, &remaining, &total,
               &direction) != 10 || axis >= axisCount() || mode > 3 || phase > 3) {
        return;
    }

    auto &state = remoteStates_[axis];
    state.mode = static_cast<MotionPlanner::Mode>(mode);
    state.phase = static_cast<MotionPlanner::Phase>(phase);
    state.position = position;
    state.target = target;
    state.speed = speed;
    state.remaining = remaining;
    state.moveTotal = total;
    state.direction = direction >= 0 ? 1 : -1;
    driversEnabled_ = enabled != 0;
}

#else

MotionEngine::MotionEngine()
    : planner_(AppConfig::kTimerFrequencyHz, AppConfig::kMaxAxes) {}

bool MotionEngine::begin(const HardwareConfig &hardware) {
    if (!HardwareConfigStore::validate(hardware)) return false;
    hardware_ = hardware;
    pinMode(hardware_.enablePin, OUTPUT);
    setEnablePin(false);
    for (std::size_t axis = 0; axis < hardware_.axisCount; ++axis) {
        pinMode(hardware_.axes[axis].step, OUTPUT);
        pinMode(hardware_.axes[axis].direction, OUTPUT);
        digitalWrite(hardware_.axes[axis].step, LOW);
        digitalWrite(hardware_.axes[axis].direction, HIGH);
    }

    timer_ = timerBegin(AppConfig::kTimerFrequencyHz);
    if (timer_ == nullptr) {
        return false;
    }
    timerAttachInterruptArg(timer_, timerThunk, this);
    timerAlarm(timer_, 1, true, 0);
    return true;
}

void MotionEngine::update() {}

std::size_t MotionEngine::axisCount() const {
    return hardware_.axisCount;
}

const HardwareConfig &MotionEngine::hardwareConfig() const {
    return hardware_;
}

bool MotionEngine::moveTo(std::size_t axis, std::int64_t target,
                          std::uint32_t speed, std::uint32_t acceleration) {
    if (axis >= axisCount()) return false;
    enableDrivers();
    portENTER_CRITICAL(&mux_);
    const bool accepted = planner_.moveTo(axis, target, speed, acceleration);
    portEXIT_CRITICAL(&mux_);
    return accepted;
}

bool MotionEngine::moveBy(std::size_t axis, std::int64_t delta,
                          std::uint32_t speed, std::uint32_t acceleration) {
    if (axis >= axisCount()) return false;
    enableDrivers();
    portENTER_CRITICAL(&mux_);
    const bool accepted = planner_.moveBy(axis, delta, speed, acceleration);
    portEXIT_CRITICAL(&mux_);
    return accepted;
}

bool MotionEngine::jog(std::size_t axis, std::int8_t direction,
                       std::uint32_t speed, std::uint32_t acceleration) {
    if (axis >= axisCount()) return false;
    enableDrivers();
    portENTER_CRITICAL(&mux_);
    const bool accepted = planner_.jog(axis, direction, speed, acceleration);
    portEXIT_CRITICAL(&mux_);
    return accepted;
}

bool MotionEngine::stop(std::size_t axis) {
    if (axis >= axisCount()) return false;
    portENTER_CRITICAL(&mux_);
    const bool accepted = planner_.stop(axis);
    portEXIT_CRITICAL(&mux_);
    return accepted;
}

void MotionEngine::immediateStopAll() {
    portENTER_CRITICAL(&mux_);
    planner_.immediateStopAll();
    portEXIT_CRITICAL(&mux_);
}

bool MotionEngine::zero(std::size_t axis) {
    if (axis >= axisCount()) return false;
    portENTER_CRITICAL(&mux_);
    const bool accepted = planner_.zero(axis);
    if (accepted && axis < referenced_.size()) {
        referenced_[axis] = true;
    }
    portEXIT_CRITICAL(&mux_);
    return accepted;
}

MotionPlanner::AxisSnapshot MotionEngine::snapshot(std::size_t axis) const {
    if (axis >= axisCount()) return {};
    portENTER_CRITICAL(&mux_);
    const auto value = planner_.snapshot(axis);
    portEXIT_CRITICAL(&mux_);
    return value;
}

bool MotionEngine::isIdle(std::size_t axis) const {
    return snapshot(axis).mode == MotionPlanner::Mode::Idle;
}

bool MotionEngine::isContinuous(std::size_t axis) const {
    return snapshot(axis).mode == MotionPlanner::Mode::Continuous;
}

void MotionEngine::enableDrivers() {
    setEnablePin(true);
}

void MotionEngine::disableDrivers() {
    immediateStopAll();
    setEnablePin(false);
    portENTER_CRITICAL(&mux_);
    referenced_.fill(false);
    portEXIT_CRITICAL(&mux_);
}

bool MotionEngine::driversEnabled() const {
    return driversEnabled_;
}

void MotionEngine::setDirectionInverted(std::size_t axis, bool inverted) {
    if (axis >= axisCount()) {
        return;
    }
    portENTER_CRITICAL(&mux_);
    directionInverted_[axis] = inverted;
    const bool positive = planner_.snapshot(axis).direction > 0;
    digitalWrite(hardware_.axes[axis].direction,
                 positive != inverted ? HIGH : LOW);
    portEXIT_CRITICAL(&mux_);
}

bool MotionEngine::positionReferenced(std::size_t axis) const {
    if (axis >= axisCount()) {
        return false;
    }
    portENTER_CRITICAL(&mux_);
    const bool referenced = referenced_[axis];
    portEXIT_CRITICAL(&mux_);
    return referenced;
}

void ARDUINO_ISR_ATTR MotionEngine::timerThunk(void *argument) {
    static_cast<MotionEngine *>(argument)->onTimer();
}

void ARDUINO_ISR_ATTR MotionEngine::onTimer() {
    portENTER_CRITICAL_ISR(&mux_);
    for (std::size_t axis = 0; axis < hardware_.axisCount; ++axis) {
        if (pulseHigh_[axis]) {
            digitalWrite(hardware_.axes[axis].step, LOW);
            pulseHigh_[axis] = false;
        }

        const auto tick = planner_.tick(axis);
        if (tick.directionChanged) {
            const bool positive = tick.direction > 0;
            digitalWrite(hardware_.axes[axis].direction,
                         positive != directionInverted_[axis] ? HIGH : LOW);
        }
        if (tick.step && driversEnabled_) {
            digitalWrite(hardware_.axes[axis].step, HIGH);
            pulseHigh_[axis] = true;
        }
    }
    portEXIT_CRITICAL_ISR(&mux_);
}

void MotionEngine::setEnablePin(bool enabled) {
    driversEnabled_ = enabled;
    const bool level = hardware_.enableActiveLow ? !enabled : enabled;
    digitalWrite(hardware_.enablePin, level ? HIGH : LOW);
}

#endif
