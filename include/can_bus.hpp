#pragma once

#include "./can_frame.hpp"
#include "device.hpp"
#include "ring_buffer.hpp"
#include <array>
#include <cstddef>

constexpr std::size_t MaxDevices = 10;

namespace can_bus {
class CanBus {
  public:
    struct TxEntry {
        Device* source{nullptr};
        CanFrame frame{};
    };

    struct RxEntry {
        Device* destination{nullptr};
        CanFrame frame{};
    };

    CanBus() = default;
    ~CanBus() = default;

    CanBus(const CanBus&) = delete;
    auto operator=(const CanBus&) -> CanBus& = delete;
    CanBus(CanBus&&) = delete;
    auto operator=(CanBus&&) -> CanBus& = delete;

    auto registerDevice(Device* device) -> bool;

    auto unregisterDevice(Device* device) -> bool;

    void transmit(Device* source, const CanFrame& frame);
    auto processSinglePass() -> bool;
    void processFrames();

    void clearFrames();

  private:
    std::array<Device*, MaxDevices> devices_{};
    std::size_t deviceCount_{0};
    ring_buffer::RingBuffer<TxEntry, 1024> buffer_;
};
} // namespace can_bus
