#pragma once

#include <array>
#include <cstddef>

namespace ring_buffer {
template <typename T, size_t Capacity> class RingBuffer {
  public:
    using CriticalSectionFn = void (*)();
    struct CriticalSectionHooks {
        CriticalSectionFn enter;
        CriticalSectionFn exit;
    };

    static_assert(Capacity > 0, "RingBuffer capacity N must be greater than 0");

    RingBuffer() = default;

    void setCriticalSectionHooks(CriticalSectionHooks hooks) {
        enter_critical = hooks.enter;
        exit_critical = hooks.exit;
    }

    [[nodiscard]] auto push(const T& value) -> bool {
        const CriticalSectionLock lock(*this);
        if (count == Capacity) {
            overflow_count++;
            return false;
        }
        buffer[head] = value;
        head = advanceIndex(head);
        count++;
        return true;
    }

    [[nodiscard]] auto pop(T& out) -> bool {
        const CriticalSectionLock lock(*this);
        if (count == 0) {
            underflow_count++;
            return false;
        }
        out = buffer[tail];
        tail = advanceIndex(tail);
        count--;
        return true;
    }

    [[nodiscard]] auto peek(T& out) const -> bool {
        const CriticalSectionLock lock(*this);
        if (count == 0) {
            underflow_count++;
            return false;
        }
        out = buffer[tail];
        return true;
    }

    [[nodiscard]] auto overflowCount() const -> size_t {
        const CriticalSectionLock lock(*this);
        return overflow_count;
    }

    [[nodiscard]] auto underflowCount() const -> size_t {
        const CriticalSectionLock lock(*this);
        return underflow_count;
    }

    [[nodiscard]] auto empty() const -> bool {
        const CriticalSectionLock lock(*this);
        return count == 0;
    }

    [[nodiscard]] auto full() const -> bool {
        const CriticalSectionLock lock(*this);
        return count == Capacity;
    }

    [[nodiscard]] auto size() const -> size_t {
        const CriticalSectionLock lock(*this);
        return count;
    }

    [[nodiscard]] static constexpr auto capacity() -> size_t {
        return Capacity;
    }

    void clear(bool resetOverflowCount = false) {
        const CriticalSectionLock lock(*this);
        head = 0;
        tail = 0;
        count = 0;
        if (resetOverflowCount) {
            overflow_count = 0;
        }
    }

    void resetCounters() {
        const CriticalSectionLock lock(*this);
        overflow_count = 0;
        underflow_count = 0;
    }

  private:
    class CriticalSectionLock {
      public:
        explicit CriticalSectionLock(const RingBuffer& ring_buffer) : ring_buffer_(&ring_buffer) {
            ring_buffer_->enterCriticalSection();
        }

        ~CriticalSectionLock() {
            ring_buffer_->exitCriticalSection();
        }

        CriticalSectionLock(const CriticalSectionLock&) = delete;
        auto operator=(const CriticalSectionLock&) -> CriticalSectionLock& = delete;

        CriticalSectionLock(CriticalSectionLock&&) = delete;
        auto operator=(CriticalSectionLock&&) -> CriticalSectionLock& = delete;

      private:
        const RingBuffer* ring_buffer_;
    };

    static constexpr bool capacityIsPowerOfTwo = (Capacity & (Capacity - 1)) == 0;

    static constexpr auto advanceIndex(size_t index) -> size_t {
        if constexpr (capacityIsPowerOfTwo) {
            return (index + 1) & (Capacity - 1);
        }
        return (index + 1) % Capacity;
    }

    void enterCriticalSection() const {
        if (enter_critical != nullptr) {
            enter_critical();
        }
    }

    void exitCriticalSection() const {
        if (exit_critical != nullptr) {
            exit_critical();
        }
    }

    std::array<T, Capacity> buffer{};
    size_t head{0};
    size_t tail{0};
    size_t count{0};
    mutable size_t underflow_count{0};
    size_t overflow_count{0};
    CriticalSectionFn enter_critical{nullptr};
    CriticalSectionFn exit_critical{nullptr};
};
} // namespace ring_buffer
