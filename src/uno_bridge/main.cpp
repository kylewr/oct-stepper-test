#include <Arduino.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

namespace {

#ifndef BRIDGE_MAX_STEP_RATE
#define BRIDGE_MAX_STEP_RATE 5000
#endif

constexpr uint8_t kAxisCount = 4;
constexpr uint8_t kEnablePin = 8;
constexpr uint8_t kStepPins[kAxisCount] = {2, 3, 4, 12};
constexpr uint8_t kDirectionPins[kAxisCount] = {5, 6, 7, 13};
constexpr uint32_t kTelemetryPeriodMs = 50;
constexpr uint16_t kPulseWidthUs = 10;

enum class Mode : uint8_t { Idle = 0, Finite = 1, Continuous = 2 };

struct Axis {
    Mode mode = Mode::Idle;
    int32_t position = 0;
    int32_t target = 0;
    int32_t moveStart = 0;
    uint32_t speed = 0;
    uint32_t commandSpeed = 0;
    uint32_t acceleration = 1;
    uint32_t speedQ = 0;
    uint32_t intervalUs = 1000;
    uint32_t lastStepAt = 0;
    uint32_t lastSpeedAt = 0;
    int8_t direction = 1;
    bool inverted = false;
};

Axis axes[kAxisCount];
bool driversEnabled = false;
char commandLine[96]{};
uint8_t commandLength = 0;
uint32_t lastTelemetryAt = 0;
uint8_t nextTelemetryAxis = 0;

uint32_t distance(int32_t from, int32_t to) {
    return to >= from ? static_cast<uint32_t>(to - from)
                      : static_cast<uint32_t>(from - to);
}

void setDrivers(bool enabled) {
    driversEnabled = enabled;
    digitalWrite(kEnablePin, enabled ? LOW : HIGH);
    if (!enabled) {
        for (auto &axis : axes) {
            axis.mode = Mode::Idle;
            axis.speed = 0;
            axis.commandSpeed = 0;
            axis.speedQ = 0;
            axis.target = axis.position;
        }
    }
}

void setDirection(uint8_t index, int8_t direction) {
    Axis &axis = axes[index];
    axis.direction = direction >= 0 ? 1 : -1;
    bool high = axis.direction < 0;
    if (axis.inverted) {
        high = !high;
    }
    // Matches the original AsyncStepper behavior: positive/CW is DIR low.
    digitalWrite(kDirectionPins[index], high ? HIGH : LOW);
}

void configureMotion(Axis &axis, uint32_t speed, uint32_t acceleration) {
    axis.commandSpeed = constrain(speed, 1UL, static_cast<unsigned long>(BRIDGE_MAX_STEP_RATE));
    axis.acceleration = constrain(acceleration, 1UL, 1000000UL);
    axis.speed = 1;
    axis.speedQ = 1UL << 16;
    axis.intervalUs = 1000000UL;
    axis.lastSpeedAt = micros();
}

void stopAxis(uint8_t index) {
    if (index >= kAxisCount) return;
    Axis &axis = axes[index];
    axis.mode = Mode::Idle;
    axis.speed = 0;
    axis.commandSpeed = 0;
    axis.speedQ = 0;
    axis.target = axis.position;
}

void startMove(uint8_t index, int32_t target, uint32_t speed, uint32_t acceleration) {
    if (index >= kAxisCount) return;
    setDrivers(true);
    Axis &axis = axes[index];
    axis.moveStart = axis.position;
    axis.target = target;
    if (target == axis.position) {
        stopAxis(index);
        return;
    }
    setDirection(index, target > axis.position ? 1 : -1);
    configureMotion(axis, speed, acceleration);
    axis.lastStepAt = micros();
    axis.mode = Mode::Finite;
}

void startJog(uint8_t index, int8_t direction, uint32_t speed, uint32_t acceleration) {
    if (index >= kAxisCount || (direction != -1 && direction != 1)) return;
    setDrivers(true);
    Axis &axis = axes[index];
    if (axis.mode == Mode::Continuous && axis.direction == direction) {
        axis.commandSpeed =
            constrain(speed, 1UL, static_cast<unsigned long>(BRIDGE_MAX_STEP_RATE));
        axis.acceleration = constrain(acceleration, 1UL, 1000000UL);
        return;
    }
    axis.moveStart = axis.position;
    axis.target = axis.position;
    setDirection(index, direction);
    configureMotion(axis, speed, acceleration);
    axis.lastStepAt = micros();
    axis.mode = Mode::Continuous;
}

void updateSpeed(Axis &axis, uint32_t now) {
    const uint32_t elapsedUs = static_cast<uint32_t>(now - axis.lastSpeedAt);
    const uint64_t deltaQ =
        static_cast<uint64_t>(axis.acceleration) * elapsedUs * (1UL << 16) / 1000000UL;
    if (deltaQ == 0) return;
    axis.lastSpeedAt = now;

    uint32_t desiredSpeed = axis.commandSpeed;
    if (axis.mode == Mode::Finite) {
        const uint32_t remaining = distance(axis.position, axis.target);
        const uint64_t stoppingDistance =
            static_cast<uint64_t>(axis.speed) * axis.speed / (2UL * axis.acceleration);
        if (remaining <= stoppingDistance + 1) desiredSpeed = 1;
    }

    const uint32_t desiredQ = desiredSpeed << 16;
    if (axis.speedQ < desiredQ) {
        const uint64_t next = static_cast<uint64_t>(axis.speedQ) + deltaQ;
        axis.speedQ = next >= desiredQ ? desiredQ : static_cast<uint32_t>(next);
    } else if (axis.speedQ > desiredQ) {
        axis.speedQ = deltaQ >= axis.speedQ - desiredQ
                          ? desiredQ
                          : axis.speedQ - static_cast<uint32_t>(deltaQ);
    }
    axis.speed = max(1UL, axis.speedQ >> 16);
    axis.intervalUs = 1000000UL / axis.speed;
}

void updateMotion() {
    const uint32_t now = micros();
    for (uint8_t index = 0; index < kAxisCount; ++index) {
        Axis &axis = axes[index];
        if (axis.mode == Mode::Idle ||
            (updateSpeed(axis, now),
             static_cast<uint32_t>(now - axis.lastStepAt) < axis.intervalUs)) {
            continue;
        }

        axis.lastStepAt += axis.intervalUs;
        digitalWrite(kStepPins[index], HIGH);
        delayMicroseconds(kPulseWidthUs);
        digitalWrite(kStepPins[index], LOW);
        axis.position += axis.direction;

        if (axis.mode == Mode::Finite && axis.position == axis.target) {
            axis.mode = Mode::Idle;
            axis.speed = 0;
        }
    }
}

void sendTelemetry(uint8_t index) {
    const Axis &axis = axes[index];
    const uint8_t mode = static_cast<uint8_t>(axis.mode);
    const uint8_t phase = axis.mode == Mode::Idle ? 0 : 2;
    const uint32_t remaining = axis.mode == Mode::Finite
                                   ? distance(axis.position, axis.target)
                                   : 0;
    const uint32_t total = axis.mode == Mode::Finite
                               ? distance(axis.moveStart, axis.target)
                               : 0;

    Serial.print(F("T "));
    Serial.print(index);
    Serial.print(' ');
    Serial.print(driversEnabled ? 1 : 0);
    Serial.print(' ');
    Serial.print(mode);
    Serial.print(' ');
    Serial.print(phase);
    Serial.print(' ');
    Serial.print(axis.position);
    Serial.print(' ');
    Serial.print(axis.target);
    Serial.print(' ');
    Serial.print(axis.mode == Mode::Idle ? 0 : axis.speed);
    Serial.print(' ');
    Serial.print(remaining);
    Serial.print(' ');
    Serial.print(total);
    Serial.print(' ');
    Serial.println(axis.direction);
}

void sendAllTelemetry() {
    for (uint8_t index = 0; index < kAxisCount; ++index) {
        sendTelemetry(index);
    }
}

void handleCommand(char *line) {
    char command = 0;
    unsigned axis = 0;
    long value = 0;
    unsigned long speed = 0;
    unsigned long acceleration = 0;

    if (sscanf(line, "%c", &command) != 1) return;
    switch (command) {
        case 'E':
            if (sscanf(line, "E %ld", &value) == 1) setDrivers(value != 0);
            break;
        case 'M':
            if (sscanf(line, "M %u %ld %lu %lu", &axis, &value, &speed,
                       &acceleration) == 4 && axis < kAxisCount) {
                startMove(static_cast<uint8_t>(axis), static_cast<int32_t>(value), speed,
                          acceleration);
            }
            break;
        case 'J':
            if (sscanf(line, "J %u %ld %lu %lu", &axis, &value, &speed,
                       &acceleration) == 4 && axis < kAxisCount) {
                startJog(static_cast<uint8_t>(axis), static_cast<int8_t>(value), speed,
                         acceleration);
            }
            break;
        case 'S':
            if (sscanf(line, "S %u", &axis) == 1) stopAxis(static_cast<uint8_t>(axis));
            break;
        case 'Z':
            if (sscanf(line, "Z %u", &axis) == 1 && axis < kAxisCount &&
                axes[axis].mode == Mode::Idle) {
                axes[axis].position = 0;
                axes[axis].target = 0;
            }
            break;
        case 'I':
            if (sscanf(line, "I %u %ld", &axis, &value) == 2 && axis < kAxisCount) {
                axes[axis].inverted = value != 0;
            }
            break;
        case 'Q':
            sendAllTelemetry();
            return;
        default:
            return;
    }
    sendAllTelemetry();
}

void readCommands() {
    while (Serial.available() > 0) {
        const char value = static_cast<char>(Serial.read());
        if (value == '\n') {
            commandLine[commandLength] = '\0';
            handleCommand(commandLine);
            commandLength = 0;
        } else if (value != '\r') {
            if (static_cast<size_t>(commandLength) + 1U < sizeof(commandLine)) {
                commandLine[commandLength++] = value;
            } else {
                commandLength = 0;
            }
        }
    }
}

} // namespace

void setup() {
    pinMode(kEnablePin, OUTPUT);
    digitalWrite(kEnablePin, HIGH);
    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        pinMode(kStepPins[axis], OUTPUT);
        pinMode(kDirectionPins[axis], OUTPUT);
        digitalWrite(kStepPins[axis], LOW);
        digitalWrite(kDirectionPins[axis], HIGH);
    }
    Serial.begin(115200);
    setDrivers(false);
    Serial.println(F("H UNO-CNC-BRIDGE 1"));
}

void loop() {
    readCommands();
    updateMotion();
    const uint32_t now = millis();
    if (static_cast<uint32_t>(now - lastTelemetryAt) >= kTelemetryPeriodMs) {
        lastTelemetryAt = now;
        // Keep routine status below the AVR serial buffer capacity so motion
        // is not stalled by a burst of blocking Serial.print sequences.
        sendTelemetry(nextTelemetryAxis);
        nextTelemetryAxis = static_cast<uint8_t>((nextTelemetryAxis + 1) % kAxisCount);
    }
}
