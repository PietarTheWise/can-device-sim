#include "controller.hpp"
#include "can_protocol.hpp"
#include "conveyor_belt.hpp"
#include "robotic_arm.hpp"
#include "vision_sensor.hpp"
#include <cstdint>
#include <iostream>

namespace {
constexpr std::size_t DeviceHeartbeatTimeoutTicks = 3U;
} // namespace

Controller::Controller(const std::string_view name, const std::size_t targetCycles)
    : target_cycles_(targetCycles) {
    static_cast<void>(name);
}

void Controller::receiveFrame(const CanFrame& frame) {
    if (frame.dlc == 0U) {
        return;
    }

    if (frame.id == can_protocol::frame::VisionStatus) {
        handleVisionStatus(frame.data[0]);
        return;
    }

    if (frame.id == can_protocol::frame::RoboticArmStatus) {
        handleRobotStatus(frame.data[0]);
        return;
    }

    if (frame.id == can_protocol::frame::ConveyorStatus) {
        handleConveyorStatus(frame.data[0]);
    }
}

void Controller::sendFrame(const CanFrame& frame) {
    static_cast<void>(pushTxFrame(frame));
}

void Controller::processTick() {
    if (phase_ == ControllerPhase::Finished || phase_ == ControllerPhase::Faulted) {
        return;
    }

    sendHeartbeat();
    monitorDeviceHeartbeats();
    if (phase_ == ControllerPhase::Faulted) {
        return;
    }

    if (!bootstrapped_) {
        bootstrapped_ = true;
        std::cout << "[controller] bootstrap: home robot + start conveyor\n";
        sendCommand(can_protocol::command::robot::Home);
        sendCommand(can_protocol::command::conveyor::Start);
        requestNextPart();
    }
}

auto Controller::isFinished() const -> bool {
    return phase_ == ControllerPhase::Finished;
}

auto Controller::hasFault() const -> bool {
    return phase_ == ControllerPhase::Faulted;
}

auto Controller::targetCycles() const -> std::size_t {
    return target_cycles_;
}

auto Controller::processedCycles() const -> std::size_t {
    return processed_cycles_;
}

auto Controller::acceptedCount() const -> std::size_t {
    return accepted_count_;
}

auto Controller::rejectedCount() const -> std::size_t {
    return rejected_count_;
}

auto Controller::phase() const -> ControllerPhase {
    return phase_;
}

void Controller::handleVisionStatus(const std::uint8_t statusByte) {
    const auto status = static_cast<VisionMessage>(statusByte);
    if (status == VisionMessage::VisionHeartbeat) {
        vision_heartbeat_misses_ = 0U;
        return;
    }
    if (status == VisionMessage::VisionFault || status == VisionMessage::VisionTimeout) {
        enterFault("vision status fault/timeout");
        return;
    }

    if (phase_ != ControllerPhase::WaitingInspection) {
        return;
    }

    if (status == VisionMessage::PartAccepted) {
        std::cout << "[controller] vision accepted part, requesting pick\n";
        sendCommand(can_protocol::command::robot::Pick);
        phase_ = ControllerPhase::WaitingPickComplete;
        return;
    }

    if (status == VisionMessage::RejectDetected) {
        std::cout << "[controller] vision rejected part, requesting reject removal\n";
        sendCommand(can_protocol::command::robot::RemoveReject);
        phase_ = ControllerPhase::WaitingRejectRemoval;
    }
}

void Controller::handleRobotStatus(const std::uint8_t statusByte) {
    const auto status = static_cast<RoboticArmMessage>(statusByte);
    if (status == RoboticArmMessage::RobotHeartbeat) {
        robot_heartbeat_misses_ = 0U;
        return;
    }
    if (status == RoboticArmMessage::RobotFault || status == RoboticArmMessage::MotionFailed ||
        status == RoboticArmMessage::PickFailed) {
        enterFault("robot reported failure");
        return;
    }

    if (phase_ == ControllerPhase::WaitingPickComplete &&
        status == RoboticArmMessage::PickCompleted) {
        std::cout << "[controller] pick complete, requesting place\n";
        sendCommand(can_protocol::command::robot::Place);
        phase_ = ControllerPhase::WaitingPlaceComplete;
        return;
    }

    if (phase_ == ControllerPhase::WaitingPlaceComplete &&
        status == RoboticArmMessage::PlaceCompleted) {
        ++accepted_count_;
        ++processed_cycles_;
        std::cout << "[controller] place complete, cycle done\n";
        if (processed_cycles_ >= target_cycles_) {
            finishRun();
            return;
        }
        requestNextPart();
        return;
    }

    if (phase_ == ControllerPhase::WaitingRejectRemoval &&
        status == RoboticArmMessage::RejectRemoved) {
        ++rejected_count_;
        ++processed_cycles_;
        std::cout << "[controller] reject removed, cycle done\n";
        if (processed_cycles_ >= target_cycles_) {
            finishRun();
            return;
        }
        requestNextPart();
    }
}

void Controller::handleConveyorStatus(const std::uint8_t statusByte) {
    const auto status = static_cast<ConveyorMessage>(statusByte);
    if (status == ConveyorMessage::ConveyorRunning || status == ConveyorMessage::ConveyorStopped ||
        status == ConveyorMessage::ConveyorSpeedReport || status == ConveyorMessage::PartArrived ||
        status == ConveyorMessage::ConveyorHeartbeat) {
        conveyor_heartbeat_misses_ = 0U;
    }
    if (status == ConveyorMessage::ConveyorFault) {
        enterFault("conveyor fault");
    }
}

void Controller::monitorDeviceHeartbeats() {
    ++conveyor_heartbeat_misses_;
    ++robot_heartbeat_misses_;
    ++vision_heartbeat_misses_;

    if (conveyor_heartbeat_misses_ >= DeviceHeartbeatTimeoutTicks) {
        enterFault("conveyor heartbeat timeout");
        return;
    }
    if (robot_heartbeat_misses_ >= DeviceHeartbeatTimeoutTicks) {
        enterFault("robot heartbeat timeout");
        return;
    }
    if (vision_heartbeat_misses_ >= DeviceHeartbeatTimeoutTicks) {
        enterFault("vision heartbeat timeout");
    }
}

void Controller::requestNextPart() {
    std::cout << "[controller] requesting next part\n";
    sendCommand(can_protocol::command::conveyor::RequestPartArrived);
    phase_ = ControllerPhase::WaitingInspection;
}

void Controller::sendCommand(const std::uint32_t frameId) {
    CanFrame frame{};
    frame.id = frameId;
    frame.dlc = 0U;
    sendFrame(frame);
}

void Controller::sendHeartbeat() {
    CanFrame frame{};
    frame.id = can_protocol::frame::ControllerHeartbeat;
    frame.dlc = 1U;
    frame.data[0] = 0xAAU;
    sendFrame(frame);
}

void Controller::sendSystemShutdown() {
    sendCommand(can_protocol::command::conveyor::Shutdown);
    sendCommand(can_protocol::command::robot::Shutdown);
    sendCommand(can_protocol::command::vision::Shutdown);
}

void Controller::finishRun() {
    sendCommand(can_protocol::command::conveyor::Stop);
    phase_ = ControllerPhase::Finished;
    std::cout << "[controller] finished run: processed=" << processed_cycles_
              << ", accepted=" << accepted_count_ << ", rejected=" << rejected_count_ << '\n';
}

void Controller::enterFault(const std::string_view reason) {
    if (!fault_latched_) {
        std::cout << "[controller] fault: " << reason << '\n';
        sendSystemShutdown();
        sendCommand(can_protocol::command::robot::Stop);
        sendCommand(can_protocol::command::conveyor::Stop);
        fault_latched_ = true;
    }
    phase_ = ControllerPhase::Faulted;
}
