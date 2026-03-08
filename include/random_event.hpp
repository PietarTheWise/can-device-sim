#pragma once

#include <cstdint>
#include <random>

struct RandomEventConfig {
    double conveyor_fault_on_part_request{0.0};
    double vision_fault_on_part_request{0.0};
    double vision_timeout_on_part_request{0.0};
    double robot_pick_failure{0.0};
    double robot_place_failure{0.0};
    std::uint32_t seed{1337U};
};

class RandomEventSystem {
  public:
    explicit RandomEventSystem(const RandomEventConfig& config);

    [[nodiscard]] auto shouldInjectConveyorFaultOnPartRequest() -> bool;
    [[nodiscard]] auto shouldInjectVisionFaultOnPartRequest() -> bool;
    [[nodiscard]] auto shouldInjectVisionTimeoutOnPartRequest() -> bool;
    [[nodiscard]] auto shouldInjectRobotPickFailure() -> bool;
    [[nodiscard]] auto shouldInjectRobotPlaceFailure() -> bool;

  private:
    static auto clampProbability(double probability) -> double;
    auto draw(double probability) -> bool;

    RandomEventConfig config_{};
    std::mt19937 generator_;
};
