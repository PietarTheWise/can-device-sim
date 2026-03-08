#include "robotic_arm.hpp"
#include "can_protocol.hpp"
#include <cstdint>
#include <iostream>

namespace {
constexpr std::size_t ControllerHeartbeatTimeoutTicks = 3U;
constexpr std::uint8_t MessageCategoryStatus = 0x01U;
constexpr std::uint8_t MessageCategoryOperationResult = 0x02U;
constexpr std::uint8_t MessageCategoryError = 0x03U;
constexpr std::uint8_t MessageCategoryStateTransition = 0x04U;
constexpr std::uint8_t MessageCategoryOptional = 0x05U;

[[nodiscard]] auto toMessageString(const RoboticArmMessage message) -> const char* {
    switch (message) {
    case RoboticArmMessage::RobotIdle:
        return "RobotIdle";
    case RoboticArmMessage::RobotReady:
        return "RobotReady";
    case RoboticArmMessage::RobotBusy:
        return "RobotBusy";
    case RoboticArmMessage::RobotStopped:
        return "RobotStopped";
    case RoboticArmMessage::RobotFault:
        return "RobotFault";
    case RoboticArmMessage::PickStarted:
        return "PickStarted";
    case RoboticArmMessage::PickCompleted:
        return "PickCompleted";
    case RoboticArmMessage::PlaceCompleted:
        return "PlaceCompleted";
    case RoboticArmMessage::RejectRemoved:
        return "RejectRemoved";
    case RoboticArmMessage::PickFailed:
        return "PickFailed";
    case RoboticArmMessage::MotionFailed:
        return "MotionFailed";
    case RoboticArmMessage::RobotHoming:
        return "RobotHoming";
    case RoboticArmMessage::RobotHomed:
        return "RobotHomed";
    case RoboticArmMessage::RobotHeartbeat:
        return "RobotHeartbeat";
    }

    return "Unknown";
}

[[nodiscard]] auto toCategoryByte(const RoboticArmMessage message) -> std::uint8_t {
    switch (message) {
    case RoboticArmMessage::RobotIdle:
    case RoboticArmMessage::RobotReady:
    case RoboticArmMessage::RobotBusy:
    case RoboticArmMessage::RobotStopped:
    case RoboticArmMessage::RobotFault:
        return MessageCategoryStatus;
    case RoboticArmMessage::PickStarted:
    case RoboticArmMessage::PickCompleted:
    case RoboticArmMessage::PlaceCompleted:
    case RoboticArmMessage::RejectRemoved:
        return MessageCategoryOperationResult;
    case RoboticArmMessage::PickFailed:
    case RoboticArmMessage::MotionFailed:
        return MessageCategoryError;
    case RoboticArmMessage::RobotHoming:
    case RoboticArmMessage::RobotHomed:
        return MessageCategoryStateTransition;
    case RoboticArmMessage::RobotHeartbeat:
        return MessageCategoryOptional;
    }

    return MessageCategoryStatus;
}
} // namespace

RoboticArm::RoboticArm(const std::string_view name, const RandomEventConfig& randomConfig)
    : random_events_(randomConfig) {
    static_cast<void>(name);
}

void RoboticArm::receiveFrame(const CanFrame& frame) {
    handleCommand(frame);
}

void RoboticArm::sendFrame(const CanFrame& frame) {
    static_cast<void>(pushTxFrame(frame));
}

void RoboticArm::processTick() {
    if (shutdown_latched_) {
        return;
    }

    if (controller_heartbeat_seen_) {
        ++controller_heartbeat_misses_;
        if (controller_heartbeat_misses_ >= ControllerHeartbeatTimeoutTicks) {
            std::cout << "Controller heartbeat timeout, homing and shutting down robotic arm\n";
            shutdown_latched_ = true;
            home();
            stop();
            return;
        }
    }
    publishHeartbeat();
}

void RoboticArm::handleCommand(const CanFrame& frame) {
    if (frame.id == can_protocol::frame::ControllerHeartbeat) {
        controller_heartbeat_seen_ = true;
        controller_heartbeat_misses_ = 0U;
        return;
    }
    if (shutdown_latched_) {
        return;
    }

    switch (frame.id) {
    case can_protocol::command::robot::Home:
        home();
        break;
    case can_protocol::command::robot::Pick:
        pick();
        break;
    case can_protocol::command::robot::Place:
        place();
        break;
    case can_protocol::command::robot::RemoveReject:
        removeReject();
        break;
    case can_protocol::command::robot::Stop:
        stop();
        break;
    case can_protocol::command::robot::Reset:
        reset();
        break;
    case can_protocol::command::robot::Shutdown:
        shutdown_latched_ = true;
        home();
        stop();
        break;
    default:
        break;
    }
}

void RoboticArm::home() {
    std::cout << "Homing robotic arm\n";
    publishStatus(RoboticArmMessage::RobotBusy);
    publishStatus(RoboticArmMessage::RobotHoming);
    publishStatus(RoboticArmMessage::RobotHomed);
    publishStatus(RoboticArmMessage::RobotReady);
    publishHeartbeat();
}

void RoboticArm::pick() {
    if (shutdown_latched_) {
        return;
    }
    std::cout << "Picking up part\n";
    publishStatus(RoboticArmMessage::RobotBusy);
    publishStatus(RoboticArmMessage::PickStarted);
    if (random_events_.shouldInjectRobotPickFailure()) {
        publishStatus(RoboticArmMessage::PickFailed);
        publishStatus(RoboticArmMessage::RobotReady);
        publishHeartbeat();
        return;
    }
    publishStatus(RoboticArmMessage::PickCompleted);
    publishStatus(RoboticArmMessage::RobotReady);
    publishHeartbeat();
}

void RoboticArm::place() {
    if (shutdown_latched_) {
        return;
    }
    std::cout << "Placing part\n";
    publishStatus(RoboticArmMessage::RobotBusy);
    if (random_events_.shouldInjectRobotPlaceFailure()) {
        publishStatus(RoboticArmMessage::MotionFailed);
        publishStatus(RoboticArmMessage::RobotReady);
        publishHeartbeat();
        return;
    }
    publishStatus(RoboticArmMessage::PlaceCompleted);
    publishStatus(RoboticArmMessage::RobotReady);
    publishHeartbeat();
}

void RoboticArm::removeReject() {
    if (shutdown_latched_) {
        return;
    }
    std::cout << "Removing rejected part\n";
    publishStatus(RoboticArmMessage::RobotBusy);
    publishStatus(RoboticArmMessage::PickStarted);
    if (random_events_.shouldInjectRobotPickFailure()) {
        publishStatus(RoboticArmMessage::PickFailed);
        publishStatus(RoboticArmMessage::RobotReady);
        publishHeartbeat();
        return;
    }
    publishStatus(RoboticArmMessage::RejectRemoved);
    publishStatus(RoboticArmMessage::RobotReady);
    publishHeartbeat();
}

void RoboticArm::stop() {
    std::cout << "Stopping robotic arm\n";
    publishStatus(RoboticArmMessage::RobotStopped);
    publishHeartbeat();
}

void RoboticArm::reset() {
    if (shutdown_latched_) {
        return;
    }
    std::cout << "Resetting robotic arm\n";
    publishStatus(RoboticArmMessage::RobotIdle);
    publishHeartbeat();
}

void RoboticArm::publishStatus(const RoboticArmMessage message) {
    CanFrame statusFrame{};
    statusFrame.id = can_protocol::frame::RoboticArmStatus;
    statusFrame.dlc = 2U;
    statusFrame.data[0] = static_cast<std::uint8_t>(message);
    statusFrame.data[1] = toCategoryByte(message);
    sendFrame(statusFrame);
    std::cout << "Published robotic arm message: " << toMessageString(message) << '\n';
}

void RoboticArm::publishHeartbeat() {
    publishStatus(RoboticArmMessage::RobotHeartbeat);
}
