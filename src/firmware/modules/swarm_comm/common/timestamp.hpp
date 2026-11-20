#ifndef SWARM_TIMESTAMP_HPP
#define SWARM_TIMESTAMP_HPP

#include <cstdint>
#include <chrono>
#include <atomic>

namespace swarm::common {

// Monotonic timestamp in microseconds
// Thread-safe, lock-free
class Timestamp {
public:
    static uint64_t nowUs() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();
    }

    static uint64_t nowMs() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();
    }

    static uint64_t nowNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();
    }

    // Synchronize with external time source (GPS, NTP, PTP)
    static void setOffset(int64_t offset_us) {
        _offset_us.store(offset_us, std::memory_order_release);
    }

    static int64_t getOffset() {
        return _offset_us.load(std::memory_order_acquire);
    }

    // Synchronized timestamp (monotonic + offset)
    static uint64_t syncedUs() {
        return nowUs() + static_cast<uint64_t>(getOffset());
    }

    // Time since epoch in microseconds (for logging compatibility)
    static uint64_t epochUs() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
    }

private:
    static inline std::atomic<int64_t> _offset_us{0};
};

// Scoped timer for profiling
class ScopedTimer {
public:
    explicit ScopedTimer(const char* label) : _label(label), _start(Timestamp::nowUs()) {}
    ~ScopedTimer() {
        uint64_t elapsed = Timestamp::nowUs() - _start;
        // Log or store elapsed time
    }
    uint64_t elapsedUs() const { return Timestamp::nowUs() - _start; }

private:
    const char* _label;
    uint64_t _start;
};

// Rate limiter
class RateLimiter {
public:
    explicit RateLimiter(float rate_hz) : _period_us(static_cast<uint64_t>(1e6 / rate_hz)), _last(0) {}

    bool ready() {
        uint64_t now = Timestamp::nowUs();
        if (now - _last >= _period_us) {
            _last = now;
            return true;
        }
        return false;
    }

    void reset() { _last = Timestamp::nowUs(); }

private:
    uint64_t _period_us;
    uint64_t _last;
};

} // namespace swarm::common

#endif // SWARM_TIMESTAMP_HPP