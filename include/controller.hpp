#pragma once

#include "device.hpp"
#include <cstddef>
#include <cstdint>
#include <string_view>

enum class ControllerPhase : std::uint8_t {
    Boot = 0,
    WaitingInspection,
    WaitingPickComplete,
    WaitingPlaceComplete,
    WaitingRejectRemoval,
    Finished,
    Faulted
};

class Controller final : public Device {
  public:
    explicit Controller(std::string_view name, std::size_t targetCycles = 5U);

    void receiveFrame(const CanFrame& frame) override;
    void sendFrame(const CanFrame& frame) override;

    void processTick();

    [[nodiscard]] auto isFinished() const -> bool;
    [[nodiscard]] auto hasFault() const -> bool;
    [[nodiscard]] auto targetCycles() const -> std::size_t;
    [[nodiscard]] auto processedCycles() const -> std::size_t;
    [[nodiscard]] auto acceptedCount() const -> std::size_t;
    [[nodiscard]] auto rejectedCount() const -> std::size_t;
    [[nodiscard]] auto phase() const -> ControllerPhase;

  private:
    void handleVisionStatus(std::uint8_t statusByte);
    void handleRobotStatus(std::uint8_t statusByte);
    void handleConveyorStatus(std::uint8_t statusByte);
    void monitorDeviceHeartbeats();
    void requestNextPart();
    void sendCommand(std::uint32_t frameId);
    void sendHeartbeat();
    void sendSystemShutdown();
    void finishRun();
    void enterFault(std::string_view reason);

    std::size_t target_cycles_{0U};
    std::size_t processed_cycles_{0U};
    std::size_t accepted_count_{0U};
    std::size_t rejected_count_{0U};
    bool bootstrapped_{false};
    bool fault_latched_{false};
    std::size_t conveyor_heartbeat_misses_{0U};
    std::size_t robot_heartbeat_misses_{0U};
    std::size_t vision_heartbeat_misses_{0U};
    ControllerPhase phase_{ControllerPhase::Boot};
};
