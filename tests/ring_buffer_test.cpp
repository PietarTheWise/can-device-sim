#include "../include/ring_buffer.hpp"

#include <cstddef>
#include <iostream>

namespace {
auto expect(bool condition, const char* message) -> bool {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

auto enterCallCount() -> size_t& {
    static size_t count = 0;
    return count;
}

auto exitCallCount() -> size_t& {
    static size_t count = 0;
    return count;
}

void enterCriticalMock() {
    ++enterCallCount();
}

void exitCriticalMock() {
    ++exitCallCount();
}
} // namespace

auto main() -> int {
    bool ok = true;

    ring_buffer::RingBuffer<int, 4> buffer;

    ok &= expect(buffer.empty(), "buffer should start empty");
    ok &= expect(!buffer.full(), "buffer should not start full");
    ok &= expect(buffer.size() == 0, "initial size should be 0");
    ok &= expect(ring_buffer::RingBuffer<int, 4>::capacity() == 4, "capacity should be 4");

    ok &= expect(buffer.push(1), "push 1 should succeed");
    ok &= expect(buffer.push(2), "push 2 should succeed");
    ok &= expect(buffer.push(3), "push 3 should succeed");
    ok &= expect(buffer.push(4), "push 4 should succeed");
    ok &= expect(buffer.full(), "buffer should be full");
    ok &= expect(buffer.size() == 4, "size should be 4 after filling");

    ok &= expect(!buffer.push(99), "push should fail when full");
    ok &= expect(buffer.overflowCount() == 1, "overflow_count should increment");

    int value = -1;
    ok &= expect(buffer.pop(value), "pop should succeed");
    ok &= expect(value == 1, "first popped value should be 1");
    ok &= expect(buffer.pop(value), "second pop should succeed");
    ok &= expect(value == 2, "second popped value should be 2");

    ok &= expect(buffer.push(5), "push 5 should succeed after wrap");
    ok &= expect(buffer.push(6), "push 6 should succeed after wrap");
    ok &= expect(buffer.full(), "buffer should be full again");

    ok &= expect(buffer.pop(value), "pop should return 3");
    ok &= expect(value == 3, "third value should be 3");
    ok &= expect(buffer.pop(value), "pop should return 4");
    ok &= expect(value == 4, "fourth value should be 4");
    ok &= expect(buffer.pop(value), "pop should return 5");
    ok &= expect(value == 5, "fifth value should be 5");
    ok &= expect(buffer.pop(value), "pop should return 6");
    ok &= expect(value == 6, "sixth value should be 6");
    ok &= expect(buffer.empty(), "buffer should be empty after draining");

    ok &= expect(!buffer.pop(value), "pop should fail on empty");
    ok &= expect(!buffer.peek(value), "peek should fail on empty");
    ok &= expect(buffer.underflowCount() == 2, "underflow_count should increment");

    ok &= expect(buffer.push(7), "push should work after underflow");
    buffer.clear();
    ok &= expect(buffer.empty(), "clear should empty the buffer");
    ok &= expect(buffer.overflowCount() == 1, "clear should keep overflow_count by default");

    buffer.clear(true);
    ok &= expect(buffer.overflowCount() == 0, "clear(true) should reset overflow_count");
    ok &= expect(buffer.underflowCount() == 2, "clear should not reset underflow_count");

    buffer.resetCounters();
    ok &= expect(buffer.overflowCount() == 0, "resetCounters should clear overflow_count");
    ok &= expect(buffer.underflowCount() == 0, "resetCounters should clear underflow_count");

    ring_buffer::RingBuffer<int, 8> guarded_buffer;
    guarded_buffer.setCriticalSectionHooks({.enter = enterCriticalMock, .exit = exitCriticalMock});
    enterCallCount() = 0;
    exitCallCount() = 0;

    ok &= expect(guarded_buffer.push(42), "guarded push should succeed");
    ok &= expect(guarded_buffer.peek(value), "guarded peek should succeed");
    ok &= expect(value == 42, "peeked value should be 42");
    ok &= expect(guarded_buffer.pop(value), "guarded pop should succeed");
    ok &= expect(value == 42, "popped value should be 42");
    ok &= expect(enterCallCount() == exitCallCount(), "enter and exit calls should match");
    ok &= expect(enterCallCount() > 0, "critical hooks should be called");

    if (!ok) {
        return 1;
    }

    std::cout << "All ring buffer tests passed.\n";
    return 0;
}
