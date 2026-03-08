#include "../include/vision_sensor.hpp"
#include "../include/can_protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string_view>
#include <vector>

namespace {
auto expect(const bool condition, const char* message) -> bool {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

auto toLabel(const VisionSensor::Verdict verdict) -> const char* {
    switch (verdict) {
    case VisionSensor::Verdict::Valid:
        return "washer";
    case VisionSensor::Verdict::Invalid:
        return "disc";
    case VisionSensor::Verdict::Unknown:
        return "unknown";
    }
    return "unknown";
}

struct SampleCase {
    std::string_view imagePath;
    std::string_view expectedLabel;
    VisionSensor::Verdict expectedVerdict;
    bool strict;
};
} // namespace

auto main() -> int {
    bool ok = true;
    VisionSensor sensor("test-vision-sensor");

    const std::vector<SampleCase> samples{
        {.imagePath = "images/conveyor_image_1.jpg",
         .expectedLabel = "washer",
         .expectedVerdict = VisionSensor::Verdict::Valid,
         .strict = true},
        {.imagePath = "images/conveyor_image_2.jpg",
         .expectedLabel = "disc",
         .expectedVerdict = VisionSensor::Verdict::Invalid,
         .strict = true},
        {.imagePath = "images/conveyor_image_3.jpg",
         .expectedLabel = "disc",
         .expectedVerdict = VisionSensor::Verdict::Invalid,
         .strict = true},
        {.imagePath = "images/conveyor_image_4.jpg",
         .expectedLabel = "washer",
         .expectedVerdict = VisionSensor::Verdict::Valid,
         .strict = true},
        {.imagePath = "images/conveyor_image_5.jpg",
         .expectedLabel = "washer",
         .expectedVerdict = VisionSensor::Verdict::Valid,
         .strict = true},
        {.imagePath = "images/conveyor_image_6.jpg",
         .expectedLabel = "disc",
         .expectedVerdict = VisionSensor::Verdict::Invalid,
         .strict = true},
        // Edge-case sample from spec: known as "square disc", accepted as non-blocking report.
        {.imagePath = "images/conveyor_image_7.jpg",
         .expectedLabel = "square disc",
         .expectedVerdict = VisionSensor::Verdict::Invalid,
         .strict = false},
    };

    std::size_t strictPassed = 0U;
    std::size_t strictTotal = 0U;
    std::size_t exploratoryMatches = 0U;

    for (std::size_t index = 0; index < samples.size(); ++index) {
        const SampleCase& sample = samples[index];
        const auto id = static_cast<std::uint16_t>(index + 1U);
        const bool exists = std::filesystem::exists(sample.imagePath);
        ok &= expect(exists, "sample image must exist");
        if (!exists) {
            continue;
        }

        sensor.queueImageInput(sample.imagePath, id);
        CanFrame trigger{};
        trigger.id = can_protocol::command::vision::ProcessTick;
        trigger.dlc = 0U;
        sensor.receiveFrame(trigger);
        const auto result = sensor.lastResult();
        const bool matched = (result.verdict == sample.expectedVerdict);

        std::cout << "[vision] " << sample.imagePath << " expected=" << sample.expectedLabel
                  << " predicted=" << toLabel(result.verdict)
                  << " score=" << static_cast<int>(result.score)
                  << (sample.strict ? " [strict]" : " [exploratory]") << '\n';

        if (sample.strict) {
            ++strictTotal;
            if (matched) {
                ++strictPassed;
            } else {
                std::cerr << "FAIL: classifier mismatch for " << sample.imagePath << '\n';
                ok = false;
            }
        } else if (matched) {
            ++exploratoryMatches;
        }
    }

    std::cout << "[vision] strict accuracy: " << strictPassed << "/" << strictTotal << '\n';
    std::cout << "[vision] exploratory matches: " << exploratoryMatches << "/1\n";

    if (!ok) {
        return 1;
    }

    std::cout << "All vision sensor tests passed.\n";
    return 0;
}
