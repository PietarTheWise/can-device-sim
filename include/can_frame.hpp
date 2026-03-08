#pragma once

#include <array>
#include <cstdint>

struct CanFrame {
    std::uint32_t id{0};
    std::uint8_t dlc{0};
    std::array<std::uint8_t, 8> data{};
};
