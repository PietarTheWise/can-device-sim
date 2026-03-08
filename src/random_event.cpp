#include "random_event.hpp"
#include <algorithm>

RandomEventSystem::RandomEventSystem(const RandomEventConfig& config)
    : config_(config), generator_(config.seed) {}

auto RandomEventSystem::shouldInjectConveyorFaultOnPartRequest() -> bool {
    return draw(config_.conveyor_fault_on_part_request);
}

auto RandomEventSystem::shouldInjectVisionFaultOnPartRequest() -> bool {
    return draw(config_.vision_fault_on_part_request);
}

auto RandomEventSystem::shouldInjectVisionTimeoutOnPartRequest() -> bool {
    return draw(config_.vision_timeout_on_part_request);
}

auto RandomEventSystem::shouldInjectRobotPickFailure() -> bool {
    return draw(config_.robot_pick_failure);
}

auto RandomEventSystem::shouldInjectRobotPlaceFailure() -> bool {
    return draw(config_.robot_place_failure);
}

auto RandomEventSystem::clampProbability(const double probability) -> double {
    return std::clamp(probability, 0.0, 1.0);
}

auto RandomEventSystem::draw(const double probability) -> bool {
    std::bernoulli_distribution distribution(clampProbability(probability));
    return distribution(generator_);
}
