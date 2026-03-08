#pragma once

#include "device.hpp"
#include "random_event.hpp"
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

enum class VisionMessage : std::uint8_t {
    // Results
    PartDetected = 0x00,
    RejectDetected = 0x01,
    PartAccepted = 0x02,
    // Processing status
    InspectionStarted = 0x10,
    InspectionComplete = 0x11,
    // Errors
    VisionFault = 0x20,
    VisionTimeout = 0x21,
    // Health
    VisionHeartbeat = 0x30
};

class VisionSensor final : public Device {
  public:
    enum class Verdict : std::uint8_t { Unknown = 0, Valid = 1, Invalid = 2 };

    struct InspectionResult {
        std::uint16_t part_id{0};
        Verdict verdict{Verdict::Unknown};
        std::uint8_t score{0}; // 0-100 confidence-like score
    };

    explicit VisionSensor(std::string_view name, const RandomEventConfig& randomConfig = {});

    void receiveFrame(const CanFrame& frame) override;
    void sendFrame(const CanFrame& frame) override;

    void processTick();

    void queueImageInput(std::string_view image_path, std::uint16_t part_id);

    auto lastResult() const -> InspectionResult;

  private:
    void handleCommand(const CanFrame& frame);
    void runInspectionCycle();
    // Temporary mock source: picks one sample image from images/ folder.
    void queueSampleImageInput();
    auto loadNextImage() -> bool;
    static auto classifyCurrentImage(std::string_view image_path, std::uint16_t part_id)
        -> InspectionResult;
    void publishInspection(const InspectionResult& result);
    void publishStatus(VisionMessage message);
    void publishHeartbeat();

    bool has_pending_image_{false};
    std::string pending_image_path_;
    std::uint16_t pending_part_id_{0};
    std::uint16_t input_part_counter_{0};
    InspectionResult last_result_{};
    bool controller_heartbeat_seen_{false};
    std::size_t controller_heartbeat_misses_{0U};
    bool shutdown_latched_{false};
    RandomEventSystem random_events_;
};
