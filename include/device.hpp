#pragma once

#include "./can_frame.hpp"
#include "ring_buffer.hpp"
#include <cstddef>

class Device {
  public:
    Device() = default;
    virtual ~Device() = default;

    Device(const Device&) = delete;
    auto operator=(const Device&) -> Device& = delete;
    Device(Device&&) = delete;
    auto operator=(Device&&) -> Device& = delete;

    virtual void receiveFrame(const CanFrame& frame) = 0;
    auto processRxFrames() -> std::size_t;
    [[nodiscard]] auto pushRxFrame(const CanFrame& frame) -> bool;
    [[nodiscard]] auto popRxFrame(CanFrame& frame) -> bool;
    [[nodiscard]] auto pushTxFrame(const CanFrame& frame) -> bool;
    [[nodiscard]] auto popTxFrame(CanFrame& frame) -> bool;

  private:
    virtual void sendFrame(const CanFrame& frame) = 0;
    ring_buffer::RingBuffer<CanFrame, 256> input_message_buffer_;
    ring_buffer::RingBuffer<CanFrame, 256> output_message_buffer_;
};
