#!/usr/bin/env python3
# Timestamp utilities for companion computer (Python)

import time
import threading
from typing import Optional

class Timestamp:
    """Monotonic timestamp in microseconds. Thread-safe."""
    
    _offset_us: int = 0
    _lock = threading.Lock()
    
    @staticmethod
    def now_us() -> int:
        return int(time.monotonic() * 1_000_000)
    
    @staticmethod
    def now_ms() -> int:
        return int(time.monotonic() * 1_000)
    
    @staticmethod
    def now_ns() -> int:
        return int(time.monotonic_ns())
    
    @classmethod
    def set_offset(cls, offset_us: int) -> None:
        with cls._lock:
            cls._offset_us = offset_us
    
    @classmethod
    def get_offset(cls) -> int:
        with cls._lock:
            return cls._offset_us
    
    @classmethod
    def synced_us(cls) -> int:
        return cls.now_us() + cls.get_offset()
    
    @staticmethod
    def epoch_us() -> int:
        return int(time.time() * 1_000_000)
    
    @staticmethod
    def epoch_ms() -> int:
        return int(time.time() * 1_000)


class ScopedTimer:
    """Context manager for profiling code blocks."""
    
    def __init__(self, label: str):
        self._label = label
        self._start = Timestamp.now_us()
        self._elapsed = 0
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self._elapsed = Timestamp.now_us() - self._start
        # In real implementation, would log to structured logger
        # logger.debug(f"{self._label}: {self._elapsed} us")
    
    @property
    def elapsed_us(self) -> int:
        return Timestamp.now_us() - self._start


class RateLimiter:
    """Rate limiter for fixed-frequency loops."""
    
    def __init__(self, rate_hz: float):
        self._period_us = int(1_000_000 / rate_hz)
        self._last = Timestamp.now_us()
    
    def ready(self) -> bool:
        now = Timestamp.now_us()
        if now - self._last >= self._period_us:
            self._last = now
            return True
        return False
    
    def sleep_until_ready(self) -> None:
        while not self.ready():
            time.sleep(0.001)  # 1ms sleep
    
    def reset(self) -> None:
        self._last = Timestamp.now_us()


class TimeSync:
    """Time synchronization with external source (GPS, NTP, PTP)."""
    
    def __init__(self):
        self._offset_us = 0
        self._lock = threading.Lock()
        self._last_sync = 0
        self._sync_interval_us = 1_000_000  # 1 second
    
    def update_offset(self, external_time_us: int) -> None:
        """Update offset from external time source."""
        local_time = Timestamp.now_us()
        with self._lock:
            self._offset_us = external_time_us - local_time
            self._last_sync = Timestamp.epoch_us()
    
    def get_offset(self) -> int:
        with self._lock:
            return self._offset_us
    
    def is_synced(self, max_age_us: int = 5_000_000) -> bool:
        with self._lock:
            return (Timestamp.epoch_us() - self._last_sync) < max_age_us
    
    def synced_us(self) -> int:
        return Timestamp.now_us() + self.get_offset()