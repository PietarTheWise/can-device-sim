#include "conveyor_belt.hpp"
#include "can_protocol.hpp"
#include <cstdint>
#include <iostream>

namespace {
constexpr std::uint8_t DirectionForward = 0x00U;
constexpr std::uint8_t DirectionReverse = 0x01U;
constexpr std::size_t ControllerHeartbeatTimeoutTicks = 3U;

[[nodiscard]] auto toMessageString(const ConveyorMessage message) -> const char* {
    switch (message) {
    case ConveyorMessage::ConveyorRunning:
        return "ConveyorRunning";
    case ConveyorMessage::ConveyorStopped:
        return "ConveyorStopped";
    case ConveyorMessage::ConveyorFault:
        return "ConveyorFault";
    case ConveyorMessage::ConveyorSpeedReport:
        return "ConveyorSpeedReport";
    case ConveyorMessage::PartArrived:
        return "PartArrived";
    case ConveyorMessage::ConveyorHeartbeat:
        return "ConveyorHeartbeat";
    }

    return "Unknown";
}
} // namespace

ConveyorBelt::ConveyorBelt(const std::string_view name, const RandomEventConfig& randomConfig)
    : random_events_(randomConfig) {
    static_cast<void>(name);
}

void ConveyorBelt::receiveFrame(const CanFrame& frame) {
    handleCommand(frame);
}

void ConveyorBelt::sendFrame(const CanFrame& frame) {
    static_cast<void>(pushTxFrame(frame));
}

void ConveyorBelt::processTick() {
    if (shutdown_latched_) {
        return;
    }

    if (controller_heartbeat_seen_) {
        ++controller_heartbeat_misses_;
        if (controller_heartbeat_misses_ >= ControllerHeartbeatTimeoutTicks) {
            std::cout << "Controller heartbeat timeout, shutting down conveyor belt\n";
            shutdown();
            return;
        }
    }
    publishStatus(ConveyorMessage::ConveyorHeartbeat);
}

void ConveyorBelt::handleCommand(const CanFrame& frame) {
    if (frame.id == can_protocol::frame::ControllerHeartbeat) {
        controller_heartbeat_seen_ = true;
        controller_heartbeat_misses_ = 0U;
        return;
    }

    switch (frame.id) {
    case can_protocol::command::conveyor::Start:
        start();
        break;
    case can_protocol::command::conveyor::Stop:
        stop();
        break;
    case can_protocol::command::conveyor::ReportSpeed:
        publishSpeed();
        break;
    case can_protocol::command::conveyor::RequestPartArrived:
        if (random_events_.shouldInjectConveyorFaultOnPartRequest()) {
            std::cout << "Conveyor action failed while moving part\n";
            publishStatus(ConveyorMessage::ConveyorFault);
        } else {
            publishStatus(ConveyorMessage::PartArrived);
        }
        break;
    case can_protocol::command::conveyor::InjectFault:
        publishStatus(ConveyorMessage::ConveyorFault);
        break;
    case can_protocol::command::conveyor::Shutdown:
        shutdown();
        break;
    default:
        break;
    }
}

void ConveyorBelt::start() {
    if (shutdown_latched_) {
        return;
    }
    std::cout << "Starting conveyor belt\n";
    publishStatus(ConveyorMessage::ConveyorRunning);
}

void ConveyorBelt::stop() {
    std::cout << "Stopping conveyor belt\n";
    publishStatus(ConveyorMessage::ConveyorStopped);
}

void ConveyorBelt::shutdown() {
    if (shutdown_latched_) {
        return;
    }
    shutdown_latched_ = true;
    stop();
}

void ConveyorBelt::setSpeed(double speed) {
    if (shutdown_latched_) {
        return;
    }
    std::cout << "Setting conveyor speed: " << speed << '\n';
    publishSpeed();
}

void ConveyorBelt::setDirection(const Direction direction) {
    const auto directionByte = direction == Direction::Forward ? DirectionForward : DirectionReverse;
    std::cout << "Setting conveyor direction: " << static_cast<unsigned>(directionByte) << '\n';
}

void ConveyorBelt::publishStatus(const ConveyorMessage message) {
    CanFrame statusFrame{};
    statusFrame.id = can_protocol::frame::ConveyorStatus;
    statusFrame.dlc = 2U;
    statusFrame.data[0] = static_cast<std::uint8_t>(message);
    statusFrame.data[1] = 0U;
    sendFrame(statusFrame);
    std::cout << "Published conveyor message: " << toMessageString(message) << '\n';
}

void ConveyorBelt::publishSpeed() {
    publishStatus(ConveyorMessage::ConveyorSpeedReport);
}
