#pragma once

#include "AppConfig.hpp"

#include <Arduino.h>
#include <Preferences.h>
#include <array>
#include <cstdint>
#include <cstring>

struct AxisProfile {
        char label[17] {};
        std::uint32_t stepsPerRevolution = AppConfig::kDefaultStepsPerRevolution;
        std::uint32_t maxSpeed = AppConfig::kDefaultMaxSpeed;
        std::uint32_t maxAcceleration = AppConfig::kDefaultMaxAcceleration;
        std::uint32_t defaultSpeed = AppConfig::kDefaultSpeed;
        std::uint32_t defaultAcceleration = AppConfig::kDefaultAcceleration;
        std::uint32_t defaultJogSteps = AppConfig::kDefaultMoveSteps;
        std::uint32_t defaultCycleSteps = AppConfig::kDefaultCycleSteps;
        std::uint32_t defaultDwellMs = AppConfig::kDefaultDwellMs;
        bool directionInverted = false;
};

class ProfileStore {
    public:
        bool begin() {
            setDefaults();
            if (!preferences_.begin("stepper-test", false)) {
                return false;
            }

            StoredProfiles stored {};
            if (preferences_.getBytesLength("profiles") == sizeof(stored) &&
                preferences_.getBytes("profiles", &stored, sizeof(stored)) == sizeof(stored) &&
                stored.magic == kMagic && stored.version == kVersion &&
                validateAll(stored.profiles)) {
                profiles_ = stored.profiles;
                return true;
            }
            save();
            return true;
        }

        const AxisProfile& get(std::size_t axis) const {
            return profiles_[axis];
        }
        AxisProfile& get(std::size_t axis) {
            return profiles_[axis];
        }

        bool update(std::size_t axis, const AxisProfile& profile) {
        if (axis >= AppConfig::kMaxAxes || !validate(profile)) {
                return false;
            }
            profiles_[axis] = profile;
            profiles_[axis].label[sizeof(profiles_[axis].label) - 1] = '\0';
            return save();
        }

        bool reset() {
            setDefaults();
            return save();
        }

        static bool validate(const AxisProfile& profile) {
            return profile.label[0] != '\0' &&
                   std::memchr(profile.label, '\0', sizeof(profile.label)) != nullptr &&
                   profile.stepsPerRevolution >= 1 && profile.stepsPerRevolution <= 1000000 &&
                   profile.maxSpeed >= 1 && profile.maxSpeed <= AppConfig::kAbsoluteMaxStepRate &&
                   profile.maxAcceleration >= 1 && profile.maxAcceleration <= 1000000 &&
                   profile.defaultSpeed >= 1 && profile.defaultSpeed <= profile.maxSpeed &&
                   profile.defaultAcceleration >= 1 &&
                   profile.defaultAcceleration <= profile.maxAcceleration &&
                   profile.defaultJogSteps >= 1 && profile.defaultJogSteps <= 100000000 &&
                   profile.defaultCycleSteps >= 1 && profile.defaultCycleSteps <= 100000000 &&
                   profile.defaultDwellMs <= 3600000;
        }

    private:
        static constexpr std::uint32_t kMagic = 0x53544550; // STEP
        static constexpr std::uint16_t kVersion = 3;

        struct StoredProfiles {
                std::uint32_t magic = kMagic;
                std::uint16_t version = kVersion;
                std::uint16_t reserved = 0;
                std::array<AxisProfile, AppConfig::kMaxAxes> profiles {};
        };

        Preferences preferences_;
        std::array<AxisProfile, AppConfig::kMaxAxes> profiles_ {};

        void setDefaults() {
            static constexpr const char* kAxisNames[AppConfig::kMaxAxes] = {
                "X axis", "Y axis", "Z axis", "A axis"
            };
            for (std::size_t axis = 0; axis < AppConfig::kMaxAxes; ++axis) {
                profiles_[axis] = AxisProfile {};
                snprintf(profiles_[axis].label, sizeof(profiles_[axis].label), "%s",
                         kAxisNames[axis]);
            }
        }

        bool save() {
            StoredProfiles stored;
            stored.profiles = profiles_;
            return preferences_.putBytes("profiles", &stored, sizeof(stored)) == sizeof(stored);
        }

        static bool validateAll(const std::array<AxisProfile, AppConfig::kMaxAxes>& profiles) {
        for (std::size_t axis = 0; axis < AppConfig::kMaxAxes; ++axis) {
                if (!validate(profiles[axis])) {
                    return false;
                }
            }
            return true;
        }
};
