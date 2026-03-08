#include "can_bus.hpp"

namespace {
constexpr std::size_t MaxProcessingPasses = 64;

auto drainOutputBuffers(can_bus::CanBus& bus,
                        const std::array<Device*, MaxDevices>& devices,
                        const std::size_t deviceCount) -> bool {
    bool drained = false;
    for (std::size_t i = 0; i < deviceCount; i++) {
        Device* source = devices.at(i);
        if (source == nullptr) {
            continue;
        }

        CanFrame frame{};
        while (source->popTxFrame(frame)) {
            bus.transmit(source, frame);
            drained = true;
        }
    }
    return drained;
}

auto distributeFrames(ring_buffer::RingBuffer<can_bus::CanBus::TxEntry, 1024>& txBuffer,
                      const std::array<Device*, MaxDevices>& devices,
                      const std::size_t deviceCount) -> bool {
    bool distributed = false;
    can_bus::CanBus::TxEntry entry{};
    while (txBuffer.pop(entry)) {
        distributed = true;
        for (std::size_t i = 0; i < deviceCount; i++) {
            Device* destination = devices.at(i);
            if ((destination == nullptr) || (destination == entry.source)) {
                continue;
            }
            static_cast<void>(destination->pushRxFrame(entry.frame));
        }
    }
    return distributed;
}

auto processInputBuffers(const std::array<Device*, MaxDevices>& devices, const std::size_t deviceCount)
    -> bool {
    bool processed = false;
    for (std::size_t i = 0; i < deviceCount; i++) {
        Device* destination = devices.at(i);
        if (destination == nullptr) {
            continue;
        }
        if (destination->processRxFrames() > 0) {
            processed = true;
        }
    }
    return processed;
}
}

namespace can_bus {
auto CanBus::registerDevice(Device* device) -> bool {
    if (deviceCount_ >= MaxDevices) {
        return false;
    }
    devices_.at(deviceCount_) = device;
    deviceCount_++;
    return true;
}

auto CanBus::unregisterDevice(Device* device) -> bool {
    for (size_t i = 0; i < deviceCount_; i++) {
        if (devices_.at(i) == device) {
            const size_t lastIndex = deviceCount_ - 1;
            devices_.at(i) = devices_.at(lastIndex);
            devices_.at(lastIndex) = nullptr;
            deviceCount_--;
            return true;
        }
    }
    return false;
}

auto CanBus::transmit(Device* source, const CanFrame& frame) -> void {
    const TxEntry entry{.source = source, .frame = frame};
    if (!buffer_.push(entry)) {
        return;
    }
}

auto CanBus::processSinglePass() -> bool {
    bool progressed = false;
    progressed = drainOutputBuffers(*this, devices_, deviceCount_) || progressed;
    progressed = distributeFrames(buffer_, devices_, deviceCount_) || progressed;
    progressed = processInputBuffers(devices_, deviceCount_) || progressed;
    return progressed;
}

auto CanBus::processFrames() -> void {
    for (std::size_t pass = 0; pass < MaxProcessingPasses; pass++) {
        if (!processSinglePass()) {
            break;
        }
    }
}

auto CanBus::clearFrames() -> void {
    buffer_.clear();
}
} // namespace can_bus
