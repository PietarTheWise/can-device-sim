#include "device.hpp"

auto Device::processRxFrames() -> std::size_t {
    std::size_t processed = 0;
    CanFrame frame{};
    while (popRxFrame(frame)) {
        receiveFrame(frame);
        processed++;
    }
    return processed;
}

auto Device::pushRxFrame(const CanFrame& frame) -> bool {
    return input_message_buffer_.push(frame);
}

auto Device::popRxFrame(CanFrame& frame) -> bool {
    return input_message_buffer_.pop(frame);
}

auto Device::pushTxFrame(const CanFrame& frame) -> bool {
    return output_message_buffer_.push(frame);
}

auto Device::popTxFrame(CanFrame& frame) -> bool {
    return output_message_buffer_.pop(frame);
}
