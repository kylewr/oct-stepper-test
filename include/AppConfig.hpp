#pragma once

#include <cstddef>
#include <cstdint>

namespace AppConfig {

    constexpr std::size_t kMaxAxes = 4;

#ifndef DEFAULT_AXIS_COUNT
#define DEFAULT_AXIS_COUNT 4
#endif
#ifndef DEFAULT_ENABLE_PIN
#define DEFAULT_ENABLE_PIN 10
#endif
#ifndef DEFAULT_ENABLE_ACTIVE_LOW
#define DEFAULT_ENABLE_ACTIVE_LOW 1
#endif
#ifndef DEFAULT_AXIS_1_STEP_PIN
#define DEFAULT_AXIS_1_STEP_PIN 3
#endif
#ifndef DEFAULT_AXIS_1_DIR_PIN
#define DEFAULT_AXIS_1_DIR_PIN 4
#endif
#ifndef DEFAULT_AXIS_2_STEP_PIN
#define DEFAULT_AXIS_2_STEP_PIN 5
#endif
#ifndef DEFAULT_AXIS_2_DIR_PIN
#define DEFAULT_AXIS_2_DIR_PIN 6
#endif
#ifndef DEFAULT_AXIS_3_STEP_PIN
#define DEFAULT_AXIS_3_STEP_PIN 7
#endif
#ifndef DEFAULT_AXIS_3_DIR_PIN
#define DEFAULT_AXIS_3_DIR_PIN 10
#endif
#ifndef DEFAULT_AXIS_4_STEP_PIN
#define DEFAULT_AXIS_4_STEP_PIN 0
#endif
#ifndef DEFAULT_AXIS_4_DIR_PIN
#define DEFAULT_AXIS_4_DIR_PIN 1
#endif
#ifndef MOTION_TIMER_HZ
#define MOTION_TIMER_HZ 50000
#endif
#ifndef MAX_STEP_RATE
#define MAX_STEP_RATE 5000
#endif
#ifndef TELEMETRY_PERIOD_MS
#define TELEMETRY_PERIOD_MS 100
#endif
#ifndef WIFI_RETRY_MS
#define WIFI_RETRY_MS 10000
#endif
#ifndef DEFAULT_STEPS_PER_REV
#define DEFAULT_STEPS_PER_REV 200
#endif
#ifndef DEFAULT_MAX_ACCELERATION
#define DEFAULT_MAX_ACCELERATION 10000
#endif
#ifndef DEFAULT_MAX_SPEED
#define DEFAULT_MAX_SPEED 1200
#endif
#ifndef DEFAULT_SPEED
#define DEFAULT_SPEED 400
#endif
#ifndef DEFAULT_ACCELERATION
#define DEFAULT_ACCELERATION 800
#endif
#ifndef DEFAULT_MOVE_STEPS
#define DEFAULT_MOVE_STEPS 20
#endif
#ifndef DEFAULT_CYCLE_STEPS
#define DEFAULT_CYCLE_STEPS 200
#endif
#ifndef DEFAULT_DWELL_MS
#define DEFAULT_DWELL_MS 250
#endif
#ifndef BRIDGE_UART_BAUD
#define BRIDGE_UART_BAUD 115200
#endif
#ifndef BRIDGE_TX_PIN
#define BRIDGE_TX_PIN 4
#endif
#ifndef BRIDGE_RX_PIN
#define BRIDGE_RX_PIN 5
#endif

    static_assert(DEFAULT_AXIS_COUNT >= 1 && DEFAULT_AXIS_COUNT <= kMaxAxes,
                  "DEFAULT_AXIS_COUNT must be between 1 and 4");
    static_assert(DEFAULT_MAX_SPEED >= 1 && DEFAULT_MAX_SPEED <= MAX_STEP_RATE,
                  "DEFAULT_MAX_SPEED must be between 1 and MAX_STEP_RATE");

    constexpr std::size_t kAxisCount = DEFAULT_AXIS_COUNT;
    constexpr std::uint8_t kEnablePin = DEFAULT_ENABLE_PIN;
    constexpr bool kEnableActiveLow = DEFAULT_ENABLE_ACTIVE_LOW != 0;
    constexpr std::uint32_t kTimerFrequencyHz = MOTION_TIMER_HZ;
    constexpr std::uint32_t kAbsoluteMaxStepRate = MAX_STEP_RATE;
    constexpr std::uint32_t kTelemetryPeriodMs = TELEMETRY_PERIOD_MS;
    constexpr std::uint32_t kWifiRetryMs = WIFI_RETRY_MS;
    constexpr std::uint32_t kDefaultStepsPerRevolution = DEFAULT_STEPS_PER_REV;
    constexpr std::uint32_t kDefaultMaxSpeed = DEFAULT_MAX_SPEED;
    constexpr std::uint32_t kDefaultMaxAcceleration = DEFAULT_MAX_ACCELERATION;
    constexpr std::uint32_t kDefaultSpeed = DEFAULT_SPEED;
    constexpr std::uint32_t kDefaultAcceleration = DEFAULT_ACCELERATION;
    constexpr std::uint32_t kDefaultMoveSteps = DEFAULT_MOVE_STEPS;
    constexpr std::uint32_t kDefaultCycleSteps = DEFAULT_CYCLE_STEPS;
    constexpr std::uint32_t kDefaultDwellMs = DEFAULT_DWELL_MS;
    constexpr std::uint32_t kBridgeUartBaud = BRIDGE_UART_BAUD;
    constexpr std::uint8_t kBridgeTxPin = BRIDGE_TX_PIN;
    constexpr std::uint8_t kBridgeRxPin = BRIDGE_RX_PIN;

    struct AxisPins {
            std::uint8_t step;
            std::uint8_t direction;
    };

    constexpr AxisPins kAxisPins[kMaxAxes] = {
        {DEFAULT_AXIS_1_STEP_PIN, DEFAULT_AXIS_1_DIR_PIN},
        {DEFAULT_AXIS_2_STEP_PIN, DEFAULT_AXIS_2_DIR_PIN},
        {DEFAULT_AXIS_3_STEP_PIN, DEFAULT_AXIS_3_DIR_PIN},
        {DEFAULT_AXIS_4_STEP_PIN, DEFAULT_AXIS_4_DIR_PIN},
    };

} // namespace AppConfig
