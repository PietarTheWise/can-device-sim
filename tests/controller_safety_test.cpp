#include "../include/can_protocol.hpp"
#include "../include/controller.hpp"
#include "../include/conveyor_belt.hpp"
#include "../include/robotic_arm.hpp"
#include "../include/vision_sensor.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {
auto expect(const bool condition, const char* message) -> bool {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

auto drainTxFrames(Controller& controller) -> std::vector<CanFrame> {
    std::vector<CanFrame> frames;
    CanFrame frame{};
    while (controller.popTxFrame(frame)) {
        frames.push_back(frame);
    }
    return frames;
}

auto containsCommand(const std::vector<CanFrame>& frames, const std::uint32_t commandId) -> bool {
    return std::ranges::any_of(frames,
                               [commandId](const CanFrame& frame) { return frame.id == commandId; });
}
} // namespace

auto main() -> int {
    bool ok = true;

    {
        Controller controller("controller-reject-test", /*targetCycles=*/1U);
        controller.processTick();
        static_cast<void>(drainTxFrames(controller)); // clear bootstrap commands

        CanFrame reject{};
        reject.id = can_protocol::frame::VisionStatus;
        reject.dlc = 1U;
        reject.data[0] = static_cast<std::uint8_t>(VisionMessage::RejectDetected);
        controller.receiveFrame(reject);

        auto afterRejectFrames = drainTxFrames(controller);
        ok &= expect(controller.phase() == ControllerPhase::WaitingRejectRemoval,
                     "reject should move controller to waiting reject removal");
        ok &= expect(containsCommand(afterRejectFrames, can_protocol::command::robot::RemoveReject),
                     "controller should command robot reject removal");

        CanFrame rejectRemoved{};
        rejectRemoved.id = can_protocol::frame::RoboticArmStatus;
        rejectRemoved.dlc = 1U;
        rejectRemoved.data[0] = static_cast<std::uint8_t>(RoboticArmMessage::RejectRemoved);
        controller.receiveFrame(rejectRemoved);

        const auto completionFrames = drainTxFrames(controller);
        ok &= expect(controller.isFinished(), "controller should finish after one reject cycle");
        ok &= expect(controller.processedCycles() == 1U,
                     "reject cycle should increase processed cycle count");
        ok &= expect(controller.rejectedCount() == 1U, "reject cycle should increase rejected count");
        ok &= expect(containsCommand(completionFrames, can_protocol::command::conveyor::Stop),
                     "finish should stop conveyor");
    }

    {
        Controller controller("controller-heartbeat-test", /*targetCycles=*/5U);
        static_cast<void>(drainTxFrames(controller));

        controller.processTick();
        controller.processTick();
        controller.processTick();

        const auto frames = drainTxFrames(controller);
        ok &= expect(controller.hasFault(), "missing device heartbeats should trigger fault");
        ok &= expect(containsCommand(frames, can_protocol::command::conveyor::Shutdown),
                     "heartbeat fault should fan out conveyor shutdown");
        ok &= expect(containsCommand(frames, can_protocol::command::robot::Shutdown),
                     "heartbeat fault should fan out robot shutdown");
        ok &= expect(containsCommand(frames, can_protocol::command::vision::Shutdown),
                     "heartbeat fault should fan out vision shutdown");
    }

    {
        Controller controller("controller-fanout-test", /*targetCycles=*/5U);
        controller.processTick();
        static_cast<void>(drainTxFrames(controller)); // clear bootstrap commands

        CanFrame conveyorFault{};
        conveyorFault.id = can_protocol::frame::ConveyorStatus;
        conveyorFault.dlc = 1U;
        conveyorFault.data[0] = static_cast<std::uint8_t>(ConveyorMessage::ConveyorFault);
        controller.receiveFrame(conveyorFault);

        const auto frames = drainTxFrames(controller);
        ok &= expect(controller.hasFault(), "conveyor fault should put controller into faulted state");
        ok &= expect(containsCommand(frames, can_protocol::command::conveyor::Shutdown),
                     "device fault should send conveyor shutdown command");
        ok &= expect(containsCommand(frames, can_protocol::command::robot::Shutdown),
                     "device fault should send robot shutdown command");
        ok &= expect(containsCommand(frames, can_protocol::command::vision::Shutdown),
                     "device fault should send vision shutdown command");
    }

    if (!ok) {
        return 1;
    }

    std::cout << "All controller safety tests passed.\n";
    return 0;
}
