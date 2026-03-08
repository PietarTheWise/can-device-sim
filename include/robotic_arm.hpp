#pragma once

#include "device.hpp"
#include "random_event.hpp"
#include <cstddef>
#include <cstdint>
#include <string_view>

enum class RoboticArmMessage : std::uint8_t {
    // Status
    RobotIdle = 0x00,
    RobotReady = 0x01,
    RobotBusy = 0x02,
    RobotStopped = 0x03,
    RobotFault = 0x04,
    // Operation results
    PickStarted = 0x10,
    PickCompleted = 0x11,
    PlaceCompleted = 0x12,
    RejectRemoved = 0x13,
    // Errors / failures
    PickFailed = 0x20,
    MotionFailed = 0x21,
    // State transitions
    RobotHoming = 0x30,
    RobotHomed = 0x31,
    // Optional
    RobotHeartbeat = 0x40
};

class RoboticArm final : public Device {
  public:
    explicit RoboticArm(std::string_view name, const RandomEventConfig& randomConfig = {});

    void receiveFrame(const CanFrame& frame) override;
    void sendFrame(const CanFrame& frame) override;
    void processTick();

  private:
    void handleCommand(const CanFrame& frame);

    void home();
    void pick();
    void place();
    void removeReject();
    void stop();
    void reset();

    void publishStatus(RoboticArmMessage message);
    void publishHeartbeat();

    bool controller_heartbeat_seen_{false};
    std::size_t controller_heartbeat_misses_{0U};
    bool shutdown_latched_{false};
    RandomEventSystem random_events_;
};
