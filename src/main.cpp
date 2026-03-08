#include "can_bus.hpp"
#include "controller.hpp"
#include "conveyor_belt.hpp"
#include "robotic_arm.hpp"
#include "vision_sensor.hpp"
#include <cctype>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>

namespace {
constexpr std::size_t DefaultSimulationMaxTicks = 100000U;
constexpr std::size_t SimulationTargetCycles = 5U;
constexpr std::uint32_t DefaultTickIntervalMs = 1000U;
constexpr std::uint32_t RestartSeedOffsetStep = 100U;

auto parseUnsignedValue(const std::string_view value) -> std::optional<std::uint32_t> {
    std::size_t parsedLength = 0U;
    try {
        const unsigned long parsed = std::stoul(std::string(value), &parsedLength, 10);
        if (parsedLength != value.size()) {
            return std::nullopt;
        }
        return static_cast<std::uint32_t>(parsed);
    } catch (...) {
        return std::nullopt;
    }
}

auto parseDoubleValue(const std::string_view value) -> std::optional<double> {
    std::size_t parsedLength = 0U;
    try {
        const double parsed = std::stod(std::string(value), &parsedLength);
        if (parsedLength != value.size()) {
            return std::nullopt;
        }
        return parsed;
    } catch (...) {
        return std::nullopt;
    }
}

auto parseOptionValue(const std::string_view arg, const std::string_view prefix)
    -> std::optional<std::string_view> {
    if (!arg.starts_with(prefix)) {
        return std::nullopt;
    }
    return arg.substr(prefix.size());
}

auto applyPreset(const std::string_view presetName, RandomEventConfig& config) -> bool {
    if (presetName == "smooth") {
        config.conveyor_fault_on_part_request = 0.0;
        config.vision_fault_on_part_request = 0.0;
        config.vision_timeout_on_part_request = 0.0;
        config.robot_pick_failure = 0.0;
        config.robot_place_failure = 0.0;
        return true;
    }

    if (presetName == "medium") {
        config.conveyor_fault_on_part_request = 0.03;
        config.vision_fault_on_part_request = 0.03;
        config.vision_timeout_on_part_request = 0.05;
        config.robot_pick_failure = 0.08;
        config.robot_place_failure = 0.08;
        return true;
    }

    if (presetName == "faulty") {
        config.conveyor_fault_on_part_request = 0.20;
        config.vision_fault_on_part_request = 0.20;
        config.vision_timeout_on_part_request = 0.25;
        config.robot_pick_failure = 0.30;
        config.robot_place_failure = 0.30;
        return true;
    }

    if (presetName == "realistic") {
        config.conveyor_fault_on_part_request = 0.05;
        config.vision_fault_on_part_request = 0.05;
        config.vision_timeout_on_part_request = 0.10;
        config.robot_pick_failure = 0.15;
        config.robot_place_failure = 0.15;
        return true;
    }

    if (presetName == "extreme") {
        config.conveyor_fault_on_part_request = 0.50;
        config.vision_fault_on_part_request = 0.50;
        config.vision_timeout_on_part_request = 0.50;
        config.robot_pick_failure = 0.50;
        config.robot_place_failure = 0.50;
        return true;
    }

    return false;
}

auto applyRandomEventOption(const std::string_view arg, RandomEventConfig& config) -> bool {
    if (const auto value = parseOptionValue(arg, "--seed=")) {
        const auto parsed = parseUnsignedValue(*value);
        if (!parsed.has_value()) {
            return false;
        }
        config.seed = *parsed;
        return true;
    }

    if (const auto value = parseOptionValue(arg, "--p-conveyor-fault=")) {
        const auto parsed = parseDoubleValue(*value);
        if (!parsed.has_value()) {
            return false;
        }
        config.conveyor_fault_on_part_request = *parsed;
        return true;
    }

    if (const auto value = parseOptionValue(arg, "--p-vision-fault=")) {
        const auto parsed = parseDoubleValue(*value);
        if (!parsed.has_value()) {
            return false;
        }
        config.vision_fault_on_part_request = *parsed;
        return true;
    }

    if (const auto value = parseOptionValue(arg, "--p-vision-timeout=")) {
        const auto parsed = parseDoubleValue(*value);
        if (!parsed.has_value()) {
            return false;
        }
        config.vision_timeout_on_part_request = *parsed;
        return true;
    }

    if (const auto value = parseOptionValue(arg, "--p-robot-pick-fail=")) {
        const auto parsed = parseDoubleValue(*value);
        if (!parsed.has_value()) {
            return false;
        }
        config.robot_pick_failure = *parsed;
        return true;
    }

    if (const auto value = parseOptionValue(arg, "--p-robot-place-fail=")) {
        const auto parsed = parseDoubleValue(*value);
        if (!parsed.has_value()) {
            return false;
        }
        config.robot_place_failure = *parsed;
        return true;
    }

    return false;
}

void printUsage() {
    std::cout << "Usage: ./build/main [options]\n"
              << "Options:\n"
              << "  --preset=<smooth|medium|faulty|realistic|extreme>\n"
              << "      smooth: no injected device failures\n"
              << "      medium: occasional failures for realism\n"
              << "      faulty: frequent failures for stress testing\n"
              << "      realistic: realistic failures for production-like testing\n"
              << "      extreme: extreme failures for stress testing\n"
              << "  --seed=<uint>\n"
              << "  --p-conveyor-fault=<0..1>\n"
              << "  --p-vision-fault=<0..1>\n"
              << "  --p-vision-timeout=<0..1>\n"
              << "  --p-robot-pick-fail=<0..1>\n"
              << "  --p-robot-place-fail=<0..1>\n"
              << "  --max-ticks=<uint>\n"
              << "  --tick-interval-ms=<uint>\n"
              << "      Each tick runs bounded single-pass bus processing.\n"
              << "  Note: explicit --p-* flags override preset values.\n";
}

auto withSeedOffset(RandomEventConfig config, const std::uint32_t offset) -> RandomEventConfig {
    config.seed += offset;
    return config;
}

class SimulationRuntime final {
  public:
    SimulationRuntime(const RandomEventConfig& baseConfig, const std::size_t targetCycles,
                      const std::uint32_t restartIndex)
        : conveyor("conveyor",
                   withSeedOffset(baseConfig, (restartIndex * RestartSeedOffsetStep) + 1U)),
          vision("vision", withSeedOffset(baseConfig, (restartIndex * RestartSeedOffsetStep) + 2U)),
          robot("robot", withSeedOffset(baseConfig, (restartIndex * RestartSeedOffsetStep) + 3U)),
          controller("controller", targetCycles) {}

    auto registerDevices() -> bool {
        return bus.registerDevice(&controller) && bus.registerDevice(&conveyor) &&
               bus.registerDevice(&vision) && bus.registerDevice(&robot);
    }

    void processTick() {
        controller.processTick();
        static_cast<void>(bus.processSinglePass());
        conveyor.processTick();
        vision.processTick();
        robot.processTick();
        static_cast<void>(bus.processSinglePass());
    }

    can_bus::CanBus bus;
    ConveyorBelt conveyor;
    VisionSensor vision;
    RoboticArm robot;
    Controller controller;
};

auto normalizeCommand(std::string command) -> std::string {
    std::string normalized;
    normalized.reserve(command.size());
    for (const unsigned char value : command) {
        if (std::isspace(value) == 0) {
            normalized.push_back(static_cast<char>(std::tolower(value)));
        }
    }
    command = std::move(normalized);
    return command;
}

auto waitForResumeCommand() -> bool {
    while (true) {
        std::cout << "\nController faulted. Type 'continue' to restart all devices "
                     "or 'quit' to exit: ";
        std::string command;
        if (!std::getline(std::cin, command)) {
            return false;
        }

        const std::string normalized = normalizeCommand(command);
        if (normalized == "continue" || normalized == "c" || normalized == "restart" ||
            normalized == "r") {
            return true;
        }
        if (normalized == "quit" || normalized == "q" || normalized == "exit") {
            return false;
        }

        std::cout << "Unknown command: '" << command << "'.\n";
    }
}
} // namespace

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
auto main(const int argc, char* argv[]) -> int {
    RandomEventConfig randomConfig{};
    std::string presetName = "custom";
    std::size_t simulationMaxTicks = DefaultSimulationMaxTicks;
    std::uint32_t tickIntervalMs = DefaultTickIntervalMs;
    const std::span<char*> args(argv, static_cast<std::size_t>(argc));

    for (const char* rawArg : args.subspan(1)) {
        const std::string_view arg(rawArg);
        if (arg == "--help") {
            printUsage();
            return 0;
        }

        if (const auto presetValue = parseOptionValue(arg, "--preset=")) {
            if (!applyPreset(*presetValue, randomConfig)) {
                std::cerr << "Invalid preset: " << *presetValue << '\n';
                printUsage();
                return 1;
            }
            presetName = std::string(*presetValue);
        }
    }

    for (const char* rawArg : args.subspan(1)) {
        const std::string_view arg(rawArg);
        if (parseOptionValue(arg, "--preset=").has_value()) {
            continue;
        }
        if (const auto maxTicksValue = parseOptionValue(arg, "--max-ticks=")) {
            const auto parsed = parseUnsignedValue(*maxTicksValue);
            if (!parsed.has_value()) {
                std::cerr << "Invalid --max-ticks value: " << *maxTicksValue << '\n';
                printUsage();
                return 1;
            }
            simulationMaxTicks = static_cast<std::size_t>(*parsed);
            continue;
        }
        if (const auto tickIntervalValue = parseOptionValue(arg, "--tick-interval-ms=")) {
            const auto parsed = parseUnsignedValue(*tickIntervalValue);
            if (!parsed.has_value()) {
                std::cerr << "Invalid --tick-interval-ms value: " << *tickIntervalValue << '\n';
                printUsage();
                return 1;
            }
            tickIntervalMs = *parsed;
            continue;
        }
        if (!applyRandomEventOption(arg, randomConfig)) {
            std::cerr << "Invalid option: " << arg << '\n';
            printUsage();
            return 1;
        }
    }

    auto createRuntime =
        [&](const std::uint32_t restartIndex) -> std::unique_ptr<SimulationRuntime> {
        auto runtime =
            std::make_unique<SimulationRuntime>(randomConfig, SimulationTargetCycles, restartIndex);
        if (!runtime->registerDevices()) {
            return nullptr;
        }
        return runtime;
    };

    std::uint32_t restartIndex = 0U;
    std::unique_ptr<SimulationRuntime> runtime = createRuntime(restartIndex);
    if (runtime == nullptr) {
        std::cerr << "Failed to register one or more devices.\n";
        return 1;
    }

    std::size_t globalTick = 0U;
    std::size_t completedRuns = 0U;
    std::size_t faultedRuns = 0U;
    std::size_t totalProcessed = 0U;
    std::size_t totalAccepted = 0U;
    std::size_t totalRejected = 0U;
    bool terminatedByUser = false;

    while (globalTick < simulationMaxTicks) {
        std::cout << "\n=== tick " << globalTick << " ===\n";
        runtime->processTick();
        ++globalTick;
        if (tickIntervalMs > 0U) {
            std::this_thread::sleep_for(std::chrono::milliseconds(tickIntervalMs));
        }

        if (runtime->controller.isFinished()) {
            totalProcessed += runtime->controller.processedCycles();
            totalAccepted += runtime->controller.acceptedCount();
            totalRejected += runtime->controller.rejectedCount();
            ++completedRuns;
            ++restartIndex;
            runtime = createRuntime(restartIndex);
            if (runtime == nullptr) {
                std::cerr << "Failed to restart runtime.\n";
                return 1;
            }
            std::cout << "[main] completed run, automatically continuing with restarted devices\n";
            continue;
        }

        if (runtime->controller.hasFault()) {
            totalProcessed += runtime->controller.processedCycles();
            totalAccepted += runtime->controller.acceptedCount();
            totalRejected += runtime->controller.rejectedCount();
            ++faultedRuns;

            if (!waitForResumeCommand()) {
                terminatedByUser = true;
                break;
            }

            ++restartIndex;
            runtime = createRuntime(restartIndex);
            if (runtime == nullptr) {
                std::cerr << "Failed to restart runtime.\n";
                return 1;
            }
            std::cout << "[main] restarted devices and controller, resuming simulation\n";
        }
    }

    std::cout << "\nSimulation summary:\n";
    std::cout << "- max ticks: " << simulationMaxTicks << '\n';
    std::cout << "- tick interval ms: " << tickIntervalMs << '\n';
    std::cout << "- ticks elapsed: " << globalTick << '\n';
    std::cout << "- target cycles per run: " << SimulationTargetCycles << '\n';
    std::cout << "- completed runs: " << completedRuns << '\n';
    std::cout << "- faulted runs: " << faultedRuns << '\n';
    std::cout << "- processed total: " << totalProcessed << '\n';
    std::cout << "- accepted total: " << totalAccepted << '\n';
    std::cout << "- rejected total: " << totalRejected << '\n';
    std::cout << "- preset: " << presetName << '\n';
    std::cout << "- seed: " << randomConfig.seed << '\n';
    std::cout << "- p-conveyor-fault: " << randomConfig.conveyor_fault_on_part_request << '\n';
    std::cout << "- p-vision-fault: " << randomConfig.vision_fault_on_part_request << '\n';
    std::cout << "- p-vision-timeout: " << randomConfig.vision_timeout_on_part_request << '\n';
    std::cout << "- p-robot-pick-fail: " << randomConfig.robot_pick_failure << '\n';
    std::cout << "- p-robot-place-fail: " << randomConfig.robot_place_failure << '\n';
    const char* resultLabel = "MAX_TICKS_REACHED";
    if (terminatedByUser) {
        resultLabel = "STOPPED_BY_USER";
    }
    std::cout << "- result: " << resultLabel << '\n';

    return 0;
}
