#pragma once

#include "AppConfig.hpp"

#include <Preferences.h>
#include <array>
#include <cstddef>
#include <cstdint>

struct HardwareConfig {
    std::uint8_t axisCount = AppConfig::kAxisCount;
    std::uint8_t enablePin = AppConfig::kEnablePin;
    bool enableActiveLow = AppConfig::kEnableActiveLow;
    std::array<AppConfig::AxisPins, AppConfig::kMaxAxes> axes = {{
        AppConfig::kAxisPins[0],
        AppConfig::kAxisPins[1],
        AppConfig::kAxisPins[2],
        AppConfig::kAxisPins[3],
    }};
};

class HardwareConfigStore {
public:
    bool begin() {
        config_ = HardwareConfig{};
        if (!preferences_.begin("stepper-hw", false)) return false;

        StoredConfig stored{};
        if (preferences_.getBytesLength("config") == sizeof(stored) &&
            preferences_.getBytes("config", &stored, sizeof(stored)) == sizeof(stored) &&
            stored.magic == kMagic && stored.version == kVersion &&
            validate(stored.config)) {
            config_ = stored.config;
            return true;
        }
        return save(config_);
    }

    const HardwareConfig &get() const { return config_; }

    bool update(const HardwareConfig &config) {
        if (!validate(config)) return false;
        config_ = config;
        return save(config_);
    }

    bool reset() {
        config_ = HardwareConfig{};
        return save(config_);
    }

    static bool isSafeOutputPin(std::uint8_t pin) {
        // ESP32-C3 DevKitC-02: keep flash, RGB/strap, USB/JTAG and UART pins
        // out of a remotely configurable motor mapping.
        return pin <= 7 || pin == 10;
    }

    static bool validate(const HardwareConfig &config) {
        if (config.axisCount < 1 || config.axisCount > AppConfig::kMaxAxes) {
            return false;
        }

#if MOTION_BACKEND_UART
        // The Arduino CNC shield owns the physical step/dir/enable pins.
        return true;
#else
        if (!isSafeOutputPin(config.enablePin)) return false;

        std::array<bool, 22> used{};
        used[config.enablePin] = true;
        for (std::size_t axis = 0; axis < config.axisCount; ++axis) {
            const auto pins = config.axes[axis];
            if (!isSafeOutputPin(pins.step) || !isSafeOutputPin(pins.direction) ||
                used[pins.step] || used[pins.direction] || pins.step == pins.direction) {
                return false;
            }
            used[pins.step] = true;
            used[pins.direction] = true;
        }
        return true;
#endif
    }

private:
    static constexpr std::uint32_t kMagic = 0x48575049; // HWPI
    static constexpr std::uint16_t kVersion = 2;

    struct StoredConfig {
        std::uint32_t magic = kMagic;
        std::uint16_t version = kVersion;
        std::uint16_t reserved = 0;
        HardwareConfig config{};
    };

    Preferences preferences_;
    HardwareConfig config_{};

    bool save(const HardwareConfig &config) {
        StoredConfig stored;
        stored.config = config;
        return preferences_.putBytes("config", &stored, sizeof(stored)) == sizeof(stored);
    }
};
