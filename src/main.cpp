#include "AppConfig.hpp"
#include "AxisProfile.hpp"
#include "CycleController.hpp"
#include "MotionEngine.hpp"
#include "WebUi.hpp"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include <cstdint>
#include <cstring>

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

namespace {

    constexpr std::size_t kCommandBytes = 1024;
    constexpr std::size_t kCommandQueueLength = 12;

    enum class NetworkEventType : std::uint8_t { Connected, Message };

    struct NetworkEvent {
            NetworkEventType type = NetworkEventType::Message;
            std::uint32_t clientId = 0;
            char json[kCommandBytes] {};
    };

    MotionEngine motion;
    HardwareConfigStore hardwareStore;
    ProfileStore profiles;
    CycleController cycles(motion);
    AsyncWebServer server(80);
    AsyncWebSocket websocket("/ws");
    QueueHandle_t commandQueue = nullptr;

    std::uint32_t lastTelemetryAt = 0;
    std::uint32_t lastWifiAttemptAt = 0;
    std::uint32_t restartAt = 0;
    bool mdnsStarted = false;

    const char* modeName(MotionPlanner::Mode mode) {
        switch (mode) {
            case MotionPlanner::Mode::Idle:
                return "stopped";
            case MotionPlanner::Mode::Finite:
                return "finite";
            case MotionPlanner::Mode::Continuous:
                return "continuous";
            case MotionPlanner::Mode::Stopping:
                return "stopping";
        }
        return "unknown";
    }

    const char* phaseName(MotionPlanner::Phase phase) {
        switch (phase) {
            case MotionPlanner::Phase::Stopped:
                return "stopped";
            case MotionPlanner::Phase::Accelerating:
                return "accelerating";
            case MotionPlanner::Phase::Running:
                return "running";
            case MotionPlanner::Phase::Decelerating:
                return "decelerating";
        }
        return "unknown";
    }

    void sendDocument(std::uint32_t clientId, JsonDocument& document) {
        AsyncWebSocketClient* client = websocket.client(clientId);
        if (client == nullptr || !client->canSend())
            return;
        String output;
        serializeJson(document, output);
        client->text(output);
    }

    void broadcastDocument(JsonDocument& document) {
        String output;
        serializeJson(document, output);
        websocket.textAll(output);
    }

    void sendError(std::uint32_t clientId, const String& requestId, const char* code,
                   const char* message) {
        JsonDocument document;
        document["v"] = 1;
        document["type"] = "error";
        document["id"] = requestId;
        document["code"] = code;
        document["message"] = message;
        sendDocument(clientId, document);
    }

    void sendAck(std::uint32_t clientId, const String& requestId) {
        JsonDocument document;
        document["v"] = 1;
        document["type"] = "ack";
        document["id"] = requestId;
        sendDocument(clientId, document);
    }

    void broadcastEvent(const char* message) {
        JsonDocument document;
        document["v"] = 1;
        document["type"] = "event";
        document["message"] = message;
        broadcastDocument(document);
    }

    void addAxisState(JsonObject axis, std::size_t index, std::uint32_t now) {
        const auto state = motion.snapshot(index);
        const auto cycle = cycles.snapshot(index, now);
        axis["index"] = index;
        axis["mode"] = static_cast<std::uint8_t>(state.mode);
        axis["modeName"] = modeName(state.mode);
        axis["phase"] = static_cast<std::uint8_t>(state.phase);
        axis["phaseName"] = phaseName(state.phase);
        axis["position"] = state.position;
        axis["target"] = state.target;
        axis["speed"] = state.speed;
        axis["commandSpeed"] = state.commandSpeed;
        axis["acceleration"] = state.acceleration;
        axis["remaining"] = state.remaining;
        axis["moveTotal"] = state.moveTotal;
        axis["direction"] = state.direction;
        axis["referenced"] = motion.positionReferenced(index);
        JsonObject cycleJson = axis["cycle"].to<JsonObject>();
        cycleJson["active"] = cycle.active;
        cycleJson["infinite"] = cycle.infinite;
        cycleJson["phase"] = static_cast<std::uint8_t>(cycle.phase);
        cycleJson["completed"] = cycle.completed;
        cycleJson["repeats"] = cycle.repeats;
        cycleJson["elapsedMs"] = cycle.elapsedMs;
    }

    void addProfile(JsonObject object, const AxisProfile& profile, std::size_t index) {
        object["index"] = index;
        object["label"] = profile.label;
        object["stepsPerRevolution"] = profile.stepsPerRevolution;
        object["directionInverted"] = profile.directionInverted;
        object["maxSpeed"] = profile.maxSpeed;
        object["maxAcceleration"] = profile.maxAcceleration;
        object["defaultSpeed"] = profile.defaultSpeed;
        object["defaultAcceleration"] = profile.defaultAcceleration;
        object["defaultJogSteps"] = profile.defaultJogSteps;
        object["defaultCycleSteps"] = profile.defaultCycleSteps;
        object["defaultDwellMs"] = profile.defaultDwellMs;
    }

    void addHardwareConfig(JsonDocument& document) {
        const auto& hardware = hardwareStore.get();
        JsonObject object = document["hardware"].to<JsonObject>();
        object["axisCount"] = hardware.axisCount;
        object["enablePin"] = hardware.enablePin;
        object["enableActiveLow"] = hardware.enableActiveLow;
        JsonArray axes = object["axes"].to<JsonArray>();
        for (std::size_t axis = 0; axis < AppConfig::kMaxAxes; ++axis) {
            JsonObject pins = axes.add<JsonObject>();
            pins["stepPin"] = hardware.axes[axis].step;
            pins["directionPin"] = hardware.axes[axis].direction;
        }
    }

    void addCommonState(JsonDocument& document, std::uint32_t now) {
#if MOTION_BACKEND_UART
        document["backend"] = "arduino-uart";
#else
        document["backend"] = "local-gpio";
#endif
        document["driversEnabled"] = motion.driversEnabled();
        document["axisCount"] = motion.axisCount();
        document["ip"] =
            WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("offline");
        document["rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
        JsonArray axesJson = document["axes"].to<JsonArray>();
        for (std::size_t axis = 0; axis < motion.axisCount(); ++axis) {
            addAxisState(axesJson.add<JsonObject>(), axis, now);
        }
    }

    void sendSnapshot(std::uint32_t clientId) {
        JsonDocument document;
        document["v"] = 1;
        document["type"] = "snapshot";
        addCommonState(document, millis());
        addHardwareConfig(document);
        JsonArray profileJson = document["profiles"].to<JsonArray>();
        for (std::size_t axis = 0; axis < motion.axisCount(); ++axis) {
            addProfile(profileJson.add<JsonObject>(), profiles.get(axis), axis);
        }
        sendDocument(clientId, document);
    }

    void broadcastSnapshot() {
        JsonDocument document;
        document["v"] = 1;
        document["type"] = "snapshot";
        addCommonState(document, millis());
        addHardwareConfig(document);
        JsonArray profileJson = document["profiles"].to<JsonArray>();
        for (std::size_t axis = 0; axis < motion.axisCount(); ++axis) {
            addProfile(profileJson.add<JsonObject>(), profiles.get(axis), axis);
        }
        broadcastDocument(document);
    }

    void sendHello(std::uint32_t clientId) {
        JsonDocument document;
        document["v"] = 1;
        document["type"] = "hello";
        document["clientId"] = clientId;
        document["axisCount"] = motion.axisCount();
        document["maxStepRate"] = AppConfig::kAbsoluteMaxStepRate;
        sendDocument(clientId, document);
    }

    void broadcastTelemetry(std::uint32_t now) {
        JsonDocument document;
        document["v"] = 1;
        document["type"] = "telemetry";
        addCommonState(document, now);
        broadcastDocument(document);
    }

    void enqueueNetworkEvent(NetworkEventType type, std::uint32_t clientId) {
        if (commandQueue == nullptr)
            return;
        NetworkEvent event;
        event.type = type;
        event.clientId = clientId;
        xQueueSend(commandQueue, &event, 0);
    }

    void onWebSocketEvent(AsyncWebSocket*, AsyncWebSocketClient* client, AwsEventType type,
                          void* argument, std::uint8_t* data, std::size_t length) {
        if (type == WS_EVT_CONNECT) {
            enqueueNetworkEvent(NetworkEventType::Connected, client->id());
            return;
        }
        if (type == WS_EVT_DISCONNECT)
            return;
        if (type != WS_EVT_DATA)
            return;

        auto* frame = static_cast<AwsFrameInfo*>(argument);
        if (!frame->final || frame->index != 0 || frame->len != length ||
            frame->opcode != WS_TEXT || length == 0 || length >= kCommandBytes) {
            client->text("{\"v\":1,\"type\":\"error\",\"code\":\"invalid_frame\",\"message\":"
                         "\"Command must be one complete text frame under 1024 bytes\"}");
            return;
        }

        NetworkEvent event;
        event.type = NetworkEventType::Message;
        event.clientId = client->id();
        std::memcpy(event.json, data, length);
        event.json[length] = '\0';
        if (xQueueSend(commandQueue, &event, 0) != pdTRUE) {
            client->text("{\"v\":1,\"type\":\"error\",\"code\":\"queue_full\",\"message\":"
                         "\"Controller is busy; retry the command\"}");
        }
    }

    bool allAxesIdle() {
        for (std::size_t axis = 0; axis < motion.axisCount(); ++axis) {
            if (!motion.isIdle(axis) || cycles.active(axis))
                return false;
        }
        return true;
    }

    bool parseMotion(JsonObjectConst payload, const AxisProfile& profile, std::uint32_t& speed,
                     std::uint32_t& acceleration) {
        if (!payload["speed"].is<std::uint32_t>() || !payload["acceleration"].is<std::uint32_t>())
            return false;
        speed = payload["speed"].as<std::uint32_t>();
        acceleration = payload["acceleration"].as<std::uint32_t>();
        return speed >= 1 && speed <= profile.maxSpeed &&
               speed <= AppConfig::kAbsoluteMaxStepRate && acceleration >= 1 &&
               acceleration <= profile.maxAcceleration;
    }

    void cancelCycle(std::size_t axis) {
        if (cycles.active(axis))
            cycles.stop(axis);
    }

    void handleProfileUpdate(std::uint32_t clientId, const String& requestId, std::size_t axis,
                             JsonObjectConst payload) {
        if (!motion.isIdle(axis) || cycles.active(axis)) {
            sendError(clientId, requestId, "axis_busy",
                      "Stop the axis before changing its profile");
            return;
        }
        if (!payload["label"].is<const char*>() ||
            !payload["stepsPerRevolution"].is<std::uint32_t>() ||
            !payload["directionInverted"].is<bool>() || !payload["maxSpeed"].is<std::uint32_t>() ||
            !payload["maxAcceleration"].is<std::uint32_t>() ||
            !payload["defaultSpeed"].is<std::uint32_t>() ||
            !payload["defaultAcceleration"].is<std::uint32_t>() ||
            !payload["defaultJogSteps"].is<std::uint32_t>() ||
            !payload["defaultCycleSteps"].is<std::uint32_t>() ||
            !payload["defaultDwellMs"].is<std::uint32_t>()) {
            sendError(clientId, requestId, "invalid_profile",
                      "Profile fields are missing or have invalid types");
            return;
        }

        AxisProfile updated = profiles.get(axis);
        const char* label = payload["label"].as<const char*>();
        if (label == nullptr || label[0] == '\0' || std::strlen(label) > 16) {
            sendError(clientId, requestId, "invalid_profile",
                      "Label must contain 1 to 16 characters");
            return;
        }
        std::strncpy(updated.label, label, sizeof(updated.label) - 1);
        updated.label[sizeof(updated.label) - 1] = '\0';
        updated.stepsPerRevolution = payload["stepsPerRevolution"].as<std::uint32_t>();
        updated.directionInverted = payload["directionInverted"].as<bool>();
        updated.maxSpeed = payload["maxSpeed"].as<std::uint32_t>();
        updated.maxAcceleration = payload["maxAcceleration"].as<std::uint32_t>();
        updated.defaultSpeed = payload["defaultSpeed"].as<std::uint32_t>();
        updated.defaultAcceleration = payload["defaultAcceleration"].as<std::uint32_t>();
        updated.defaultJogSteps = payload["defaultJogSteps"].as<std::uint32_t>();
        updated.defaultCycleSteps = payload["defaultCycleSteps"].as<std::uint32_t>();
        updated.defaultDwellMs = payload["defaultDwellMs"].as<std::uint32_t>();

        if (!profiles.update(axis, updated)) {
            sendError(clientId, requestId, "invalid_profile",
                      "Profile values exceed the supported ranges");
            return;
        }
        motion.setDirectionInverted(axis, updated.directionInverted);
        sendAck(clientId, requestId);
        broadcastSnapshot();
    }

    void handleHardwareUpdate(std::uint32_t clientId, const String& requestId,
                              JsonObjectConst payload) {
        if (restartAt != 0) {
            sendError(clientId, requestId, "reboot_pending",
                      "A hardware configuration reboot is already pending");
            return;
        }
        if (!payload["axisCount"].is<std::uint8_t>() ||
            !payload["enablePin"].is<std::uint8_t>() ||
            !payload["enableActiveLow"].is<bool>() ||
            !payload["axes"].is<JsonArrayConst>()) {
            sendError(clientId, requestId, "invalid_hardware",
                      "Hardware configuration fields are missing or invalid");
            return;
        }

        const JsonArrayConst axes = payload["axes"].as<JsonArrayConst>();
        if (axes.size() != AppConfig::kMaxAxes) {
            sendError(clientId, requestId, "invalid_hardware",
                      "Pin mappings for all four axes are required");
            return;
        }

        HardwareConfig updated = hardwareStore.get();
        updated.axisCount = payload["axisCount"].as<std::uint8_t>();
        updated.enablePin = payload["enablePin"].as<std::uint8_t>();
        updated.enableActiveLow = payload["enableActiveLow"].as<bool>();
        for (std::size_t axis = 0; axis < AppConfig::kMaxAxes; ++axis) {
            const JsonObjectConst pins = axes[axis].as<JsonObjectConst>();
            if (!pins["stepPin"].is<std::uint8_t>() ||
                !pins["directionPin"].is<std::uint8_t>()) {
                sendError(clientId, requestId, "invalid_hardware",
                          "Every axis needs numeric STEP and DIR pins");
                return;
            }
            updated.axes[axis].step = pins["stepPin"].as<std::uint8_t>();
            updated.axes[axis].direction = pins["directionPin"].as<std::uint8_t>();
        }

        if (!HardwareConfigStore::validate(updated)) {
            sendError(clientId, requestId, "invalid_hardware",
                      "Use unique GPIOs from 0-7 or 10 for ENABLE and each active STEP/DIR pair");
            return;
        }

        cycles.stopAll();
        motion.disableDrivers();
        if (!hardwareStore.update(updated)) {
            sendError(clientId, requestId, "storage_error",
                      "Could not persist the hardware configuration");
            return;
        }
        sendAck(clientId, requestId);
        broadcastEvent("Hardware configuration saved; drivers disabled and controller rebooting");
        restartAt = millis() + 750;
    }

    void handleCommand(const NetworkEvent& event) {
        JsonDocument document;
        const DeserializationError parseError = deserializeJson(document, event.json);
        if (parseError) {
            sendError(event.clientId, "", "invalid_json", parseError.c_str());
            return;
        }

        const String requestId = document["id"] | "";
        const char* type = document["type"] | "";
        if ((document["v"] | 0) != 1 || requestId.isEmpty() || type[0] == '\0') {
            sendError(event.clientId, requestId, "invalid_envelope",
                      "Expected protocol v1 with id and type");
            return;
        }

        if (std::strcmp(type, "emergencyStop") == 0) {
            cycles.stopAll();
            motion.disableDrivers();
            sendAck(event.clientId, requestId);
            broadcastEvent("Emergency stop: all motion cancelled and drivers disabled");
            return;
        }
        const JsonObjectConst payload = document["payload"].as<JsonObjectConst>();
        if (std::strcmp(type, "setDrivers") == 0) {
            if (!payload["enabled"].is<bool>()) {
                sendError(event.clientId, requestId, "invalid_argument", "enabled must be boolean");
                return;
            }
            if (payload["enabled"].as<bool>()) {
                motion.enableDrivers();
            } else {
                cycles.stopAll();
                motion.disableDrivers();
            }
            sendAck(event.clientId, requestId);
            return;
        }

        if (std::strcmp(type, "updateHardware") == 0) {
            handleHardwareUpdate(event.clientId, requestId, payload);
            return;
        }

        if (std::strcmp(type, "resetHardware") == 0) {
            cycles.stopAll();
            motion.disableDrivers();
            if (!hardwareStore.reset()) {
                sendError(event.clientId, requestId, "storage_error",
                          "Could not restore the default hardware configuration");
                return;
            }
            sendAck(event.clientId, requestId);
            broadcastEvent("Default hardware configuration restored; controller rebooting");
            restartAt = millis() + 750;
            return;
        }

        if (std::strcmp(type, "resetProfiles") == 0) {
            if (!allAxesIdle()) {
                sendError(event.clientId, requestId, "axes_busy",
                          "Stop every axis before resetting profiles");
                return;
            }
            if (!profiles.reset()) {
                sendError(event.clientId, requestId, "storage_error",
                          "Could not save default profiles");
                return;
            }
            for (std::size_t axis = 0; axis < motion.axisCount(); ++axis) {
                motion.setDirectionInverted(axis, profiles.get(axis).directionInverted);
            }
            sendAck(event.clientId, requestId);
            broadcastSnapshot();
            return;
        }

        if (!document["axis"].is<std::uint32_t>()) {
            sendError(event.clientId, requestId, "invalid_axis", "axis is required");
            return;
        }
        const std::size_t axis = document["axis"].as<std::uint32_t>();
        if (axis >= motion.axisCount()) {
            sendError(event.clientId, requestId, "invalid_axis",
                      "axis is outside the configured range");
            return;
        }

        if (std::strcmp(type, "updateProfile") == 0) {
            handleProfileUpdate(event.clientId, requestId, axis, payload);
            return;
        }
        if (std::strcmp(type, "stopAxis") == 0 || std::strcmp(type, "stopCycle") == 0) {
            cancelCycle(axis);
            motion.stop(axis);
            sendAck(event.clientId, requestId);
            return;
        }
        if (std::strcmp(type, "zeroPosition") == 0) {
            if (!motion.zero(axis)) {
                sendError(event.clientId, requestId, "axis_busy",
                          "Position can only be zeroed while stopped");
                return;
            }
            sendAck(event.clientId, requestId);
            return;
        }
        if (std::strcmp(type, "jogStop") == 0) {
            if (motion.isContinuous(axis))
                motion.stop(axis);
            sendAck(event.clientId, requestId);
            return;
        }

        std::uint32_t speed = 0;
        std::uint32_t acceleration = 0;
        if (!parseMotion(payload, profiles.get(axis), speed, acceleration)) {
            sendError(event.clientId, requestId, "invalid_motion",
                      "Speed or acceleration is missing or outside the axis profile limits");
            return;
        }

        bool accepted = false;
        if (std::strcmp(type, "moveRelative") == 0) {
            if (!payload["steps"].is<std::int64_t>()) {
                sendError(event.clientId, requestId, "invalid_argument",
                          "steps must be a signed integer");
                return;
            }
            cancelCycle(axis);
            accepted =
                motion.moveBy(axis, payload["steps"].as<std::int64_t>(), speed, acceleration);
        } else if (std::strcmp(type, "moveAbsolute") == 0) {
            if (!payload["steps"].is<std::int64_t>()) {
                sendError(event.clientId, requestId, "invalid_argument",
                          "steps must be a signed integer");
                return;
            }
            cancelCycle(axis);
            accepted =
                motion.moveTo(axis, payload["steps"].as<std::int64_t>(), speed, acceleration);
        } else if (std::strcmp(type, "jogStart") == 0) {
            if (!payload["direction"].is<std::int8_t>()) {
                sendError(event.clientId, requestId, "invalid_argument",
                          "direction must be -1 or 1");
                return;
            }
            const std::int8_t direction = payload["direction"].as<std::int8_t>();
            if (direction != -1 && direction != 1) {
                sendError(event.clientId, requestId, "invalid_argument",
                          "direction must be -1 or 1");
                return;
            }
            cancelCycle(axis);
            accepted = motion.jog(axis, direction, speed, acceleration);
        } else if (std::strcmp(type, "startCycle") == 0) {
            if (!payload["travelSteps"].is<std::uint64_t>() ||
                !payload["initialDirection"].is<std::int8_t>() ||
                !payload["dwellMs"].is<std::uint32_t>() ||
                !payload["repeats"].is<std::uint32_t>() || !payload["infinite"].is<bool>()) {
                sendError(event.clientId, requestId, "invalid_cycle",
                          "Cycle fields are missing or invalid");
                return;
            }
            const std::int8_t direction = payload["initialDirection"].as<std::int8_t>();
            if (direction != -1 && direction != 1) {
                sendError(event.clientId, requestId, "invalid_cycle",
                          "initialDirection must be -1 or 1");
                return;
            }
            accepted = cycles.start(axis, payload["travelSteps"].as<std::uint64_t>(), direction,
                                    speed, acceleration, payload["dwellMs"].as<std::uint32_t>(),
                                    payload["repeats"].as<std::uint32_t>(),
                                    payload["infinite"].as<bool>(), millis());
        } else {
            sendError(event.clientId, requestId, "unknown_command",
                      "Command type is not supported");
            return;
        }

        if (!accepted) {
            sendError(event.clientId, requestId, "command_rejected",
                      "Motion command could not be accepted");
            return;
        }
        sendAck(event.clientId, requestId);
    }

    void processNetworkEvents() {
        NetworkEvent event;
        while (xQueueReceive(commandQueue, &event, 0) == pdTRUE) {
            if (event.type == NetworkEventType::Connected) {
                sendHello(event.clientId);
                sendSnapshot(event.clientId);
            } else {
                handleCommand(event);
            }
        }
    }

    void maintainWifi(std::uint32_t now) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!mdnsStarted) {
                mdnsStarted = MDNS.begin("stepper-test");
                if (mdnsStarted)
                    MDNS.addService("http", "tcp", 80);
            }
            return;
        }
        if (mdnsStarted) {
            MDNS.end();
            mdnsStarted = false;
        }
        if (WIFI_SSID[0] == '\0')
            return;
        if (lastWifiAttemptAt == 0 ||
            static_cast<std::uint32_t>(now - lastWifiAttemptAt) >= AppConfig::kWifiRetryMs) {
            lastWifiAttemptAt = now;
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            Serial.printf("Connecting to Wi-Fi SSID %s\n", WIFI_SSID);
        }
    }

} // namespace

void setup() {
    Serial.begin(115200);
    delay(50);
    if (!hardwareStore.begin()) {
        Serial.println("WARNING: hardware configuration could not be persisted");
    }
    if (!motion.begin(hardwareStore.get())) {
        Serial.println("FATAL: could not start the 50 kHz motion timer");
        return;
    }
    profiles.begin();
    for (std::size_t axis = 0; axis < motion.axisCount(); ++axis) {
        motion.setDirectionInverted(axis, profiles.get(axis).directionInverted);
    }

    commandQueue = xQueueCreate(kCommandQueueLength, sizeof(NetworkEvent));
    if (commandQueue == nullptr) {
        Serial.println("FATAL: could not create command queue");
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    maintainWifi(millis());

    websocket.onEvent(onWebSocketEvent);
    server.addHandler(&websocket);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        AsyncWebServerResponse* response =
            request->beginResponse(200, "text/html; charset=utf-8", WEB_UI);
        response->addHeader("Cache-Control", "no-store");
        response->addHeader("X-Content-Type-Options", "nosniff");
        request->send(response);
    });
    server.on("/health", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json", "{\"ok\":true}");
    });
    server.onNotFound([](AsyncWebServerRequest* request) {
        request->redirect("/");
    });
    server.begin();
    Serial.printf("Stepper test bench ready with %u axis/axes\n",
                  static_cast<unsigned>(motion.axisCount()));
}

void loop() {
    const std::uint32_t now = millis();
    motion.update();
    processNetworkEvents();
    cycles.update(now);
    maintainWifi(now);

    if (static_cast<std::uint32_t>(now - lastTelemetryAt) >= AppConfig::kTelemetryPeriodMs) {
        lastTelemetryAt = now;
        broadcastTelemetry(now);
    }
    static std::uint32_t lastCleanupAt = 0;
    if (static_cast<std::uint32_t>(now - lastCleanupAt) >= 2000) {
        lastCleanupAt = now;
        websocket.cleanupClients();
    }
    if (restartAt != 0 && static_cast<std::int32_t>(now - restartAt) >= 0) {
        ESP.restart();
    }
    delay(1);
}
