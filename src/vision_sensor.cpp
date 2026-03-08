#include "vision_sensor.hpp"
#include "can_protocol.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include "third_party/stb_image.h"
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "third_party/stb_image_resize2.h"

namespace {
constexpr int MaxImageDimension = 192;
constexpr std::uint8_t ConveyorPartArrivedByte = 0x11U;
constexpr std::size_t ControllerHeartbeatTimeoutTicks = 3U;
constexpr int MinComponentAreaPixels = 20;
constexpr int CompactComponentMinArea = 120;
constexpr int NonBorderFallbackMinArea = 80;
constexpr double CompactComponentMaxAspect = 1.35;
constexpr double CenterRegionRadius = 0.38;
constexpr double AnnulusInnerRadius = 0.50;
constexpr double AnnulusOuterRadius = 0.92;
constexpr double DarkMassCenterRadius = 0.52;
constexpr int MinSampleFloor = 10;
constexpr int MinBoxDimension = 10;
constexpr double MinRadiusPixels = 4.0;
constexpr double BrightThresholdStdDevScale = 0.35;
constexpr int BrightThresholdMin = 32;
constexpr int BrightThresholdMax = 230;
constexpr double DarkCutoffOffset = 8.0;
constexpr double RoundPartAspectMax = 1.72;
constexpr double WasherContrastMin = 4.0;
constexpr double WasherCenterDarkRatioMin = 0.18;
constexpr double WasherDarkMassRatioMin = 0.20;
constexpr double WasherStrengthMin = 17.0;

struct GrayImage {
    int width{0};
    int height{0};
    std::vector<std::uint8_t> pixels;
};

struct BoundingBox {
    int minX{0};
    int minY{0};
    int maxX{-1};
    int maxY{-1};
};

struct HoleFeatures {
    bool valid{false};
    double centerMean{0.0};
    double annulusMean{0.0};
    double centerDarkRatio{0.0};
    double darkMassRatio{0.0};
    double darkConcentration{0.0};
};

struct RingCenterSamples {
    std::uint64_t annulusSum{0U};
    std::uint64_t centerSum{0U};
    int annulusCount{0};
    int centerCount{0};
};

struct DarkStats {
    int centerDarkCount{0};
    int discPixelCount{0};
    int darkPixelCount{0};
    double darkWeight{0.0};
    double centeredDarkWeight{0.0};
};

struct CircleGeometry {
    double centerX{0.0};
    double centerY{0.0};
    double radius{1.0};
};

struct BrightComponent {
    BoundingBox box{};
    int area{0};
    bool touchesBorder{false};
};

struct PixelCoord {
    int x{0};
    int y{0};
};

// Temporary mock image source until real sensor image input is connected.
[[nodiscard]] auto sampleImageInputPath() -> std::string_view {
    static constexpr std::array<std::string_view, 7> MockImagePool{
        "images/conveyor_image_1.jpg", "images/conveyor_image_2.jpg", "images/conveyor_image_3.jpg",
        "images/conveyor_image_4.jpg", "images/conveyor_image_5.jpg", "images/conveyor_image_6.jpg",
        "images/conveyor_image_7.jpg"};

    static std::random_device randomDevice;
    static std::mt19937 generator(randomDevice());
    static std::uniform_int_distribution<std::size_t> distribution(0, MockImagePool.size() - 1U);
    return MockImagePool.at(distribution(generator));
}

[[nodiscard]] auto pixelIndex(const int x, const int y, const int width) -> std::size_t {
    return (static_cast<std::size_t>(y) * static_cast<std::size_t>(width)) +
           static_cast<std::size_t>(x);
}

[[nodiscard]] auto isInBounds(const GrayImage& image, const int x, const int y) -> bool {
    return x >= 0 && y >= 0 && x < image.width && y < image.height;
}

[[nodiscard]] auto canVisitBrightPixel(const GrayImage& image, const PixelCoord position,
                                       const int brightThreshold,
                                       const std::vector<std::uint8_t>& visited) -> bool {
    if (!isInBounds(image, position.x, position.y)) {
        return false;
    }
    const auto index = pixelIndex(position.x, position.y, image.width);
    return visited[index] == 0U &&
           image.pixels[index] >= static_cast<std::uint8_t>(brightThreshold);
}

[[nodiscard]] auto floodFillBrightComponent(const GrayImage& image, const PixelCoord start,
                                            const int brightThreshold,
                                            std::vector<std::uint8_t>& visited) -> BrightComponent {
    BrightComponent component{};
    component.box.minX = start.x;
    component.box.minY = start.y;
    component.box.maxX = start.x;
    component.box.maxY = start.y;

    std::vector<std::pair<int, int>> stack;
    stack.emplace_back(start.x, start.y);
    visited[pixelIndex(start.x, start.y, image.width)] = 1U;

    while (!stack.empty()) {
        const auto [x, y] = stack.back();
        stack.pop_back();
        ++component.area;

        component.box.minX = std::min(component.box.minX, x);
        component.box.minY = std::min(component.box.minY, y);
        component.box.maxX = std::max(component.box.maxX, x);
        component.box.maxY = std::max(component.box.maxY, y);

        if (x == 0 || y == 0 || x == (image.width - 1) || y == (image.height - 1)) {
            component.touchesBorder = true;
        }

        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const int nx = x + dx;
                const int ny = y + dy;
                if (!canVisitBrightPixel(image, PixelCoord{.x = nx, .y = ny}, brightThreshold,
                                         visited)) {
                    continue;
                }
                visited[pixelIndex(nx, ny, image.width)] = 1U;
                stack.emplace_back(nx, ny);
            }
        }
    }

    return component;
}

[[nodiscard]] auto shouldPreferCompactNonBorder(const BoundingBox& nonBorderBox,
                                                const int nonBorderArea) -> bool {
    if (nonBorderArea <= 0) {
        return false;
    }
    const int width = (nonBorderBox.maxX - nonBorderBox.minX) + 1;
    const int height = (nonBorderBox.maxY - nonBorderBox.minY) + 1;
    const double aspect = static_cast<double>(std::max(width, height)) /
                          static_cast<double>(std::max(1, std::min(width, height)));
    return nonBorderArea >= CompactComponentMinArea && aspect <= CompactComponentMaxAspect;
}

[[nodiscard]] auto loadAndDownscaleGrayscale(std::string_view imagePath) -> GrayImage {
    std::string path(imagePath);
    int sourceWidth = 0;
    int sourceHeight = 0;
    int sourceChannels = 0;
    stbi_uc* sourcePixels =
        stbi_load(path.c_str(), &sourceWidth, &sourceHeight, &sourceChannels, 1);
    if (sourcePixels == nullptr || sourceWidth <= 0 || sourceHeight <= 0) {
        if (sourcePixels != nullptr) {
            stbi_image_free(sourcePixels);
        }
        return {};
    }

    const int sourceMaxDimension = std::max(sourceWidth, sourceHeight);
    const double scale =
        (sourceMaxDimension > MaxImageDimension)
            ? static_cast<double>(MaxImageDimension) / static_cast<double>(sourceMaxDimension)
            : 1.0;
    const int targetWidth =
        std::max(1, static_cast<int>(std::lround(static_cast<double>(sourceWidth) * scale)));
    const int targetHeight =
        std::max(1, static_cast<int>(std::lround(static_cast<double>(sourceHeight) * scale)));

    GrayImage image{};
    image.width = targetWidth;
    image.height = targetHeight;
    const auto pixelCount =
        static_cast<std::size_t>(targetWidth) * static_cast<std::size_t>(targetHeight);
    image.pixels.resize(pixelCount);

    if (targetWidth == sourceWidth && targetHeight == sourceHeight) {
        std::copy_n(sourcePixels, pixelCount, image.pixels.begin());
        stbi_image_free(sourcePixels);
        return image;
    }

    const auto* resized =
        stbir_resize_uint8_linear(sourcePixels, sourceWidth, sourceHeight, 0, image.pixels.data(),
                                  targetWidth, targetHeight, 0, static_cast<stbir_pixel_layout>(1));
    stbi_image_free(sourcePixels);
    if (resized == nullptr) {
        return {};
    }

    return image;
}

[[nodiscard]] auto findBrightBoundingBox(const GrayImage& image, const int brightThreshold)
    -> BoundingBox {
    BoundingBox bestAnyBox{};
    BoundingBox bestNonBorderBox{};
    int bestAnyArea = 0;
    int bestNonBorderArea = 0;
    std::vector<std::uint8_t> visited(image.pixels.size(), 0U);

    for (int startY = 0; startY < image.height; ++startY) {
        for (int startX = 0; startX < image.width; ++startX) {
            const PixelCoord start{.x = startX, .y = startY};
            if (!canVisitBrightPixel(image, start, brightThreshold, visited)) {
                continue;
            }

            const BrightComponent component =
                floodFillBrightComponent(image, start, brightThreshold, visited);
            if (component.area < MinComponentAreaPixels) {
                continue;
            }

            if (component.area > bestAnyArea) {
                bestAnyArea = component.area;
                bestAnyBox = component.box;
            }

            if (!component.touchesBorder && component.area > bestNonBorderArea) {
                bestNonBorderArea = component.area;
                bestNonBorderBox = component.box;
            }
        }
    }

    if (shouldPreferCompactNonBorder(bestNonBorderBox, bestNonBorderArea)) {
        return bestNonBorderBox;
    }

    if (bestNonBorderArea > 0 &&
        bestNonBorderArea >= std::max(NonBorderFallbackMinArea, bestAnyArea / 3)) {
        return bestNonBorderBox;
    }
    return bestAnyBox;
}

[[nodiscard]] auto computeAdaptiveBrightThreshold(const GrayImage& image) -> int {
    std::uint64_t brightnessSum = 0U;
    for (const std::uint8_t value : image.pixels) {
        brightnessSum += value;
    }
    const double averageBrightness =
        static_cast<double>(brightnessSum) / static_cast<double>(image.pixels.size());

    double varianceSum = 0.0;
    for (const std::uint8_t value : image.pixels) {
        const double delta = static_cast<double>(value) - averageBrightness;
        varianceSum += delta * delta;
    }
    const double standardDeviation =
        std::sqrt(varianceSum / static_cast<double>(image.pixels.size()));

    return std::clamp(static_cast<int>(std::lround(
                          averageBrightness + (BrightThresholdStdDevScale * standardDeviation))),
                      BrightThresholdMin, BrightThresholdMax);
}

template <typename Func>
void forEachDiskPixel(const GrayImage& image, const BoundingBox& box,
                      const CircleGeometry& geometry, const Func& visitPixel) {
    for (int y = box.minY; y <= box.maxY; ++y) {
        for (int x = box.minX; x <= box.maxX; ++x) {
            const double dx = static_cast<double>(x) - geometry.centerX;
            const double dy = static_cast<double>(y) - geometry.centerY;
            const double normalizedDistance = std::sqrt((dx * dx) + (dy * dy)) / geometry.radius;
            if (normalizedDistance > 1.0) {
                continue;
            }

            const std::uint8_t value = image.pixels[pixelIndex(x, y, image.width)];
            visitPixel(normalizedDistance, value);
        }
    }
}

[[nodiscard]] auto collectRingCenterSamples(const GrayImage& image, const BoundingBox& box,
                                            const CircleGeometry& geometry) -> RingCenterSamples {
    RingCenterSamples samples{};
    forEachDiskPixel(image, box, geometry,
                     [&samples](const double normalizedDistance, const std::uint8_t value) {
                         if (normalizedDistance < CenterRegionRadius) {
                             samples.centerSum += value;
                             ++samples.centerCount;
                         } else if (normalizedDistance > AnnulusInnerRadius &&
                                    normalizedDistance < AnnulusOuterRadius) {
                             samples.annulusSum += value;
                             ++samples.annulusCount;
                         }
                     });
    return samples;
}

[[nodiscard]] auto collectDarkStats(const GrayImage& image, const BoundingBox& box,
                                    const CircleGeometry& geometry, const double darkCutoff)
    -> DarkStats {
    DarkStats stats{};
    forEachDiskPixel(
        image, box, geometry,
        [&stats, darkCutoff](const double normalizedDistance, const std::uint8_t value) {
            ++stats.discPixelCount;
            if (normalizedDistance < CenterRegionRadius &&
                static_cast<double>(value) <= darkCutoff) {
                ++stats.centerDarkCount;
            }
            if (static_cast<double>(value) <= darkCutoff) {
                ++stats.darkPixelCount;
                const double weight = (darkCutoff - static_cast<double>(value)) + 1.0;
                stats.darkWeight += weight;
                if (normalizedDistance < DarkMassCenterRadius) {
                    stats.centeredDarkWeight += weight;
                }
            }
        });
    return stats;
}

[[nodiscard]] auto computeHoleFeatures(const GrayImage& image, const BoundingBox& box)
    -> HoleFeatures {
    const int boxWidth = (box.maxX - box.minX) + 1;
    const int boxHeight = (box.maxY - box.minY) + 1;
    const CircleGeometry geometry{.centerX = static_cast<double>(box.minX + box.maxX) * 0.5,
                                  .centerY = static_cast<double>(box.minY + box.maxY) * 0.5,
                                  .radius =
                                      static_cast<double>(std::min(boxWidth, boxHeight)) * 0.5};

    const RingCenterSamples samples = collectRingCenterSamples(image, box, geometry);

    const int minSampleCount = std::max(MinSampleFloor, std::min(boxWidth, boxHeight) / 5);
    if (samples.annulusCount < minSampleCount || samples.centerCount < minSampleCount) {
        return {};
    }

    HoleFeatures features{};
    features.valid = true;
    features.centerMean =
        static_cast<double>(samples.centerSum) / static_cast<double>(samples.centerCount);
    features.annulusMean =
        static_cast<double>(samples.annulusSum) / static_cast<double>(samples.annulusCount);

    const double darkCutoff = features.annulusMean - DarkCutoffOffset;
    const DarkStats darkStats = collectDarkStats(image, box, geometry, darkCutoff);

    features.centerDarkRatio =
        static_cast<double>(darkStats.centerDarkCount) / static_cast<double>(samples.centerCount);
    if (darkStats.discPixelCount > 0) {
        features.darkMassRatio = static_cast<double>(darkStats.darkPixelCount) /
                                 static_cast<double>(darkStats.discPixelCount);
    }
    if (darkStats.darkWeight > 0.0) {
        features.darkConcentration = darkStats.centeredDarkWeight / darkStats.darkWeight;
    }
    return features;
}

[[nodiscard]] auto classifyWasher(const GrayImage& image, std::uint16_t part_id)
    -> VisionSensor::InspectionResult {
    VisionSensor::InspectionResult result{};
    result.part_id = part_id;

    if (image.width <= 0 || image.height <= 0 || image.pixels.empty()) {
        result.verdict = VisionSensor::Verdict::Unknown;
        result.score = 0U;
        return result;
    }

    const int brightThreshold = computeAdaptiveBrightThreshold(image);
    const BoundingBox box = findBrightBoundingBox(image, brightThreshold);
    if (box.maxX < 0 || box.maxY < 0) {
        result.verdict = VisionSensor::Verdict::Invalid;
        result.score = 15U;
        return result;
    }

    const int boxWidth = (box.maxX - box.minX) + 1;
    const int boxHeight = (box.maxY - box.minY) + 1;
    if (boxWidth < MinBoxDimension || boxHeight < MinBoxDimension) {
        result.verdict = VisionSensor::Verdict::Invalid;
        result.score = 20U;
        return result;
    }

    const double radius = static_cast<double>(std::min(boxWidth, boxHeight)) * 0.5;
    if (radius < MinRadiusPixels) {
        result.verdict = VisionSensor::Verdict::Invalid;
        result.score = 20U;
        return result;
    }

    const HoleFeatures features = computeHoleFeatures(image, box);
    if (!features.valid) {
        result.verdict = VisionSensor::Verdict::Unknown;
        result.score = 0U;
        return result;
    }

    const double annulusContrast = features.annulusMean - features.centerMean;
    const double holeStrength = (annulusContrast * 0.58) + (features.centerDarkRatio * 28.0) +
                                (features.darkConcentration * 18.0) +
                                (features.darkMassRatio * 12.0);
    const double aspectRatio = static_cast<double>(std::max(boxWidth, boxHeight)) /
                               static_cast<double>(std::max(1, std::min(boxWidth, boxHeight)));

    const bool likelyRoundPart = aspectRatio < RoundPartAspectMax;
    const bool washerLike =
        likelyRoundPart && (((annulusContrast > WasherContrastMin) &&
                             (features.centerDarkRatio > WasherCenterDarkRatioMin) &&
                             (features.darkMassRatio > WasherDarkMassRatioMin)) ||
                            (holeStrength > WasherStrengthMin));

    if (washerLike) {
        result.verdict = VisionSensor::Verdict::Valid;
        const int score = static_cast<int>(std::lround(
            std::clamp(35.0 + (annulusContrast * 1.2) + (features.centerDarkRatio * 22.0) +
                           (features.darkConcentration * 25.0),
                       50.0, 99.0)));
        result.score = static_cast<std::uint8_t>(score);
    } else {
        result.verdict = VisionSensor::Verdict::Invalid;
        const int score = static_cast<int>(std::lround(
            std::clamp(85.0 - (annulusContrast * 1.1) - (features.centerDarkRatio * 24.0) -
                           (features.darkConcentration * 20.0),
                       10.0, 90.0)));
        result.score = static_cast<std::uint8_t>(score);
    }

    return result;
}

[[nodiscard]] auto toMessageString(const VisionMessage message) -> const char* {
    switch (message) {
    case VisionMessage::PartDetected:
        return "PartDetected";
    case VisionMessage::RejectDetected:
        return "RejectDetected";
    case VisionMessage::PartAccepted:
        return "PartAccepted";
    case VisionMessage::InspectionStarted:
        return "InspectionStarted";
    case VisionMessage::InspectionComplete:
        return "InspectionComplete";
    case VisionMessage::VisionFault:
        return "VisionFault";
    case VisionMessage::VisionTimeout:
        return "VisionTimeout";
    case VisionMessage::VisionHeartbeat:
        return "VisionHeartbeat";
    }

    return "Unknown";
}
} // namespace

VisionSensor::VisionSensor(const std::string_view name, const RandomEventConfig& randomConfig)
    : random_events_(randomConfig) {
    static_cast<void>(name);
}

void VisionSensor::receiveFrame(const CanFrame& frame) {
    handleCommand(frame);
}

void VisionSensor::sendFrame(const CanFrame& frame) {
    static_cast<void>(pushTxFrame(frame));
}

void VisionSensor::processTick() {
    if (shutdown_latched_) {
        return;
    }

    if (controller_heartbeat_seen_) {
        ++controller_heartbeat_misses_;
        if (controller_heartbeat_misses_ >= ControllerHeartbeatTimeoutTicks) {
            std::cout << "Controller heartbeat timeout, shutting down vision sensor\n";
            shutdown_latched_ = true;
            return;
        }
    }
    publishHeartbeat();
}

void VisionSensor::runInspectionCycle() {
    if (random_events_.shouldInjectVisionFaultOnPartRequest()) {
        publishStatus(VisionMessage::VisionFault);
        publishHeartbeat();
        return;
    }

    if (random_events_.shouldInjectVisionTimeoutOnPartRequest()) {
        publishStatus(VisionMessage::VisionTimeout);
        publishHeartbeat();
        return;
    }

    if (!loadNextImage()) {
        publishStatus(VisionMessage::VisionTimeout);
        publishHeartbeat();
        return;
    }

    last_result_ = classifyCurrentImage(pending_image_path_, pending_part_id_);
    publishInspection(last_result_);
    publishHeartbeat();
}

void VisionSensor::queueImageInput(std::string_view image_path, std::uint16_t part_id) {
    pending_image_path_ = std::string(image_path);
    has_pending_image_ = true;
    pending_part_id_ = part_id;
}

auto VisionSensor::lastResult() const -> InspectionResult {
    return last_result_;
}

void VisionSensor::handleCommand(const CanFrame& frame) {
    if (frame.id == can_protocol::frame::ControllerHeartbeat) {
        controller_heartbeat_seen_ = true;
        controller_heartbeat_misses_ = 0U;
        return;
    }
    if (frame.id == can_protocol::command::vision::Shutdown) {
        shutdown_latched_ = true;
        std::cout << "Shutting down vision sensor\n";
        return;
    }
    if (shutdown_latched_) {
        return;
    }

    if (frame.id == can_protocol::frame::ConveyorStatus && frame.dlc > 0U &&
        frame.data[0] == ConveyorPartArrivedByte) {
        queueSampleImageInput();
        runInspectionCycle();
        return;
    }

    switch (frame.id) {
    case can_protocol::command::vision::ProcessTick:
        runInspectionCycle();
        break;
    case can_protocol::command::vision::PublishHeartbeat:
        publishHeartbeat();
        break;
    case can_protocol::command::vision::InjectFault:
        publishStatus(VisionMessage::VisionFault);
        break;
    case can_protocol::command::vision::InjectTimeout:
        publishStatus(VisionMessage::VisionTimeout);
        break;
    default:
        break;
    }
}

void VisionSensor::queueSampleImageInput() {
    ++input_part_counter_;
    if (input_part_counter_ == 0U) {
        input_part_counter_ = 1U;
    }
    queueImageInput(sampleImageInputPath(), input_part_counter_);
}

auto VisionSensor::loadNextImage() -> bool {
    if (!has_pending_image_ || pending_image_path_.empty()) {
        return false;
    }

    has_pending_image_ = false;
    return true;
}

auto VisionSensor::classifyCurrentImage(std::string_view image_path, std::uint16_t part_id)
    -> InspectionResult {
    const GrayImage image = loadAndDownscaleGrayscale(image_path);
    return classifyWasher(image, part_id);
}

void VisionSensor::publishInspection(const InspectionResult& result) {
    if (result.verdict == Verdict::Valid) {
        publishStatus(VisionMessage::PartAccepted);
        return;
    }

    // Invalid/Unknown are treated as "disregard part" for downstream control.
    publishStatus(VisionMessage::RejectDetected);
}

void VisionSensor::publishStatus(const VisionMessage message) {
    CanFrame statusFrame{};
    statusFrame.id = can_protocol::frame::VisionStatus;
    statusFrame.dlc = 1U;
    statusFrame.data[0] = static_cast<std::uint8_t>(message);
    sendFrame(statusFrame);
    std::cout << "Published vision message: " << toMessageString(message) << '\n';
}

void VisionSensor::publishHeartbeat() {
    publishStatus(VisionMessage::VisionHeartbeat);
}
