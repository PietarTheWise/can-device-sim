#pragma once

#include <cstdint>

namespace can_protocol {
namespace frame {
constexpr std::uint32_t ControllerHeartbeat = 0x100U;
constexpr std::uint32_t RoboticArmStatus = 0x201U;
constexpr std::uint32_t ConveyorStatus = 0x301U;
constexpr std::uint32_t VisionStatus = 0x401U;
} // namespace frame

namespace command {
namespace robot {
constexpr std::uint32_t Home = 0x01U;
constexpr std::uint32_t Pick = 0x02U;
constexpr std::uint32_t Place = 0x03U;
constexpr std::uint32_t Stop = 0x04U;
constexpr std::uint32_t Reset = 0x05U;
constexpr std::uint32_t Shutdown = 0x06U;
constexpr std::uint32_t RemoveReject = 0x07U;
} // namespace robot

namespace conveyor {
constexpr std::uint32_t Start = 0x10U;
constexpr std::uint32_t Stop = 0x11U;
constexpr std::uint32_t ReportSpeed = 0x12U;
constexpr std::uint32_t RequestPartArrived = 0x13U;
constexpr std::uint32_t InjectFault = 0x14U;
constexpr std::uint32_t Shutdown = 0x15U;
} // namespace conveyor

namespace vision {
constexpr std::uint32_t ProcessTick = 0x20U;
constexpr std::uint32_t PublishHeartbeat = 0x21U;
constexpr std::uint32_t InjectFault = 0x22U;
constexpr std::uint32_t InjectTimeout = 0x23U;
constexpr std::uint32_t Shutdown = 0x24U;
} // namespace vision
} // namespace command
} // namespace can_protocol
