#include "../include/can_bus.hpp"

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

class TestDevice final : public Device {
  public:
    explicit TestDevice(const bool autoReply = false, const std::uint32_t replyId = 0U)
        : autoReply_(autoReply), replyId_(replyId) {}

    void receiveFrame(const CanFrame& frame) override {
        receivedFrames_.push_back(frame);
        if (autoReply_) {
            CanFrame reply{};
            reply.id = replyId_;
            reply.dlc = 1U;
            reply.data[0] = 0xAAU;
            sendFrame(reply);
        }
    }

    void queueTxFrame(const CanFrame& frame) {
        sendFrame(frame);
    }

    [[nodiscard]] auto receivedCount() const -> std::size_t {
        return receivedFrames_.size();
    }

    [[nodiscard]] auto receivedAt(const std::size_t index) const -> const CanFrame& {
        return receivedFrames_.at(index);
    }

  private:
    void sendFrame(const CanFrame& frame) override {
        static_cast<void>(pushTxFrame(frame));
    }

    bool autoReply_{false};
    std::uint32_t replyId_{0};
    std::vector<CanFrame> receivedFrames_;
};
} // namespace

auto main() -> int {
    bool ok = true;

    {
        can_bus::CanBus bus;
        TestDevice source;
        TestDevice destinationA;
        TestDevice destinationB;

        ok &= expect(bus.registerDevice(&source), "register source should succeed");
        ok &= expect(bus.registerDevice(&destinationA), "register destinationA should succeed");
        ok &= expect(bus.registerDevice(&destinationB), "register destinationB should succeed");

        CanFrame frame{};
        frame.id = 0x123U;
        frame.dlc = 2U;
        frame.data[0] = 0xDEU;
        frame.data[1] = 0xADU;

        source.queueTxFrame(frame);
        bus.processFrames();

        ok &= expect(source.receivedCount() == 0U, "source should not receive its own frame");
        ok &= expect(destinationA.receivedCount() == 1U, "destinationA should receive one frame");
        ok &= expect(destinationB.receivedCount() == 1U, "destinationB should receive one frame");
        ok &=
            expect(destinationA.receivedAt(0).id == frame.id, "destinationA frame id should match");
        ok &=
            expect(destinationB.receivedAt(0).id == frame.id, "destinationB frame id should match");
    }

    {
        can_bus::CanBus bus;
        TestDevice commandSource;
        TestDevice responder(/*autoReply=*/true, /*replyId=*/0x456U);

        ok &= expect(bus.registerDevice(&commandSource), "register commandSource should succeed");
        ok &= expect(bus.registerDevice(&responder), "register responder should succeed");

        CanFrame command{};
        command.id = 0x111U;
        command.dlc = 1U;
        command.data[0] = 0x01U;
        commandSource.queueTxFrame(command);

        bus.processFrames();

        ok &= expect(responder.receivedCount() == 1U, "responder should receive command");
        ok &= expect(commandSource.receivedCount() == 1U, "source should receive responder reply");
        ok &=
            expect(commandSource.receivedAt(0).id == 0x456U, "reply id should match responder id");
    }

    {
        can_bus::CanBus bus;
        TestDevice commandSource;
        TestDevice responder(/*autoReply=*/true, /*replyId=*/0x888U);

        ok &= expect(bus.registerDevice(&commandSource), "register commandSource for single pass");
        ok &= expect(bus.registerDevice(&responder), "register responder for single pass");

        CanFrame command{};
        command.id = 0x222U;
        command.dlc = 1U;
        command.data[0] = 0x09U;
        commandSource.queueTxFrame(command);

        static_cast<void>(bus.processSinglePass());
        ok &= expect(responder.receivedCount() == 1U,
                     "responder should receive command in first pass");
        ok &= expect(commandSource.receivedCount() == 0U,
                     "source should not receive auto-reply in first pass");

        static_cast<void>(bus.processSinglePass());
        ok &= expect(commandSource.receivedCount() == 1U,
                     "source should receive auto-reply in second pass");
        ok &= expect(commandSource.receivedAt(0).id == 0x888U,
                     "single-pass auto-reply id should match");
    }

    {
        can_bus::CanBus bus;
        TestDevice source;
        TestDevice unregistered;

        ok &= expect(bus.registerDevice(&source), "register source should succeed");
        ok &= expect(bus.registerDevice(&unregistered), "register unregistered should succeed");
        ok &= expect(bus.unregisterDevice(&unregistered), "unregister should succeed");

        CanFrame frame{};
        frame.id = 0x77U;
        frame.dlc = 0U;
        source.queueTxFrame(frame);
        bus.processFrames();

        ok &= expect(unregistered.receivedCount() == 0U,
                     "unregistered device should not receive frames");
    }

    {
        can_bus::CanBus bus;
        TestDevice source;
        TestDevice destination;

        ok &= expect(bus.registerDevice(&source), "register source for transmit should succeed");
        ok &= expect(bus.registerDevice(&destination),
                     "register destination for transmit should succeed");

        CanFrame frame{};
        frame.id = 0xA0U;
        frame.dlc = 1U;
        frame.data[0] = 0xFEU;

        bus.transmit(&source, frame);
        bus.processFrames();

        ok &= expect(destination.receivedCount() == 1U,
                     "destination should receive transmit() frame");
        ok &= expect(destination.receivedAt(0).id == frame.id, "transmit() frame id should match");
    }

    if (!ok) {
        return 1;
    }

    std::cout << "All CAN bus tests passed.\n";
    return 0;
}
