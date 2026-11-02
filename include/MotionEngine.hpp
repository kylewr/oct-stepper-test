#pragma once

#include "AppConfig.hpp"
#include "HardwareConfig.hpp"
#include "MotionPlanner.hpp"

#include <Arduino.h>
#include <array>

class MotionEngine {
    public:
        MotionEngine();

        bool begin(const HardwareConfig& hardware);
        void update();
        std::size_t axisCount() const;
        const HardwareConfig& hardwareConfig() const;
        bool moveTo(std::size_t axis, std::int64_t target, std::uint32_t speed,
                    std::uint32_t acceleration);
        bool moveBy(std::size_t axis, std::int64_t delta, std::uint32_t speed,
                    std::uint32_t acceleration);
        bool jog(std::size_t axis, std::int8_t direction, std::uint32_t speed,
                 std::uint32_t acceleration);
        bool stop(std::size_t axis);
        void immediateStopAll();
        bool zero(std::size_t axis);
        MotionPlanner::AxisSnapshot snapshot(std::size_t axis) const;
        bool isIdle(std::size_t axis) const;
        bool isContinuous(std::size_t axis) const;

        void enableDrivers();
        void disableDrivers();
        bool driversEnabled() const;
        void setDirectionInverted(std::size_t axis, bool inverted);
        bool positionReferenced(std::size_t axis) const;

    private:
#if MOTION_BACKEND_UART
        void sendCommand(const char* format, ...);
        void processBridgeLine(const char* line);

        HardwareConfig hardware_{};
        HardwareSerial bridgeSerial_{1};
        std::array<MotionPlanner::AxisSnapshot, AppConfig::kMaxAxes> remoteStates_{};
        std::array<bool, AppConfig::kMaxAxes> directionInverted_{};
        std::array<bool, AppConfig::kMaxAxes> referenced_{};
        char bridgeLine_[160]{};
        std::size_t bridgeLineLength_ = 0;
        bool driversEnabled_ = false;
#else
        static void ARDUINO_ISR_ATTR timerThunk(void* argument);
        void ARDUINO_ISR_ATTR onTimer();
        void setEnablePin(bool enabled);

        mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
        MotionPlanner planner_;
        HardwareConfig hardware_{};
        hw_timer_t* timer_ = nullptr;
        volatile bool driversEnabled_ = false;
        std::array<bool, AppConfig::kMaxAxes> pulseHigh_ {};
        std::array<bool, AppConfig::kMaxAxes> directionInverted_ {};
        std::array<bool, AppConfig::kMaxAxes> referenced_ {};
#endif
};
