#pragma once

#include "device.hpp"
#include "random_event.hpp"
#include <cstddef>
#include <cstdint>
#include <string_view>

enum class Direction : std::uint8_t { Forward, Reverse };
enum class ConveyorMessage : std::uint8_t {
    ConveyorRunning = 0x00,
    ConveyorStopped = 0x01,
    ConveyorFault = 0x02,
    ConveyorSpeedReport = 0x10,
    PartArrived = 0x11,
    ConveyorHeartbeat = 0x20
};

class ConveyorBelt final : public Device {
  public:
    explicit ConveyorBelt(std::string_view name, const RandomEventConfig& randomConfig = {});
    void receiveFrame(const CanFrame& frame) override;
    void sendFrame(const CanFrame& frame) override;
    void processTick();

  private:
    void handleCommand(const CanFrame& frame);
    void start();
    void stop();
    void shutdown();
    void setSpeed(double speed);
    static void setDirection(Direction direction);
    void publishStatus(ConveyorMessage message);
    void publishSpeed();

    bool controller_heartbeat_seen_{false};
    std::size_t controller_heartbeat_misses_{0U};
    bool shutdown_latched_{false};
    RandomEventSystem random_events_;
};
