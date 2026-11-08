#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace runtime_logging
{

/*
 * Single-producer / single-consumer bounded ring.
 *
 * try_push never blocks; when the ring is full it returns false so the
 * caller can count a drop instead of stalling a real-time producer.
 * The producer is expected to be externally serialised (e.g. one UART
 * read thread, or callers that already hold a write mutex).
 */
template <typename T>
class BoundedRing
{
public:
  explicit BoundedRing(size_t capacity)
  : buf_(capacity > 0 ? capacity + 1 : 2), capacity_(capacity > 0 ? capacity + 1 : 2)
  {
  }

  bool try_push(T item)
  {
    const size_t head = head_.load(std::memory_order_relaxed);
    const size_t next = advance(head);
    if (next == tail_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    buf_[head] = std::move(item);
    head_.store(next, std::memory_order_release);
    pushed_.fetch_add(1, std::memory_order_relaxed);
    return true;
  }

  std::optional<T> try_pop()
  {
    const size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      return std::nullopt;
    }
    T item = std::move(buf_[tail]);
    tail_.store(advance(tail), std::memory_order_release);
    popped_.fetch_add(1, std::memory_order_relaxed);
    return item;
  }

  bool empty() const
  {
    return tail_.load(std::memory_order_acquire) ==
           head_.load(std::memory_order_acquire);
  }

  size_t size() const
  {
    const size_t h = head_.load(std::memory_order_acquire);
    const size_t t = tail_.load(std::memory_order_acquire);
    return h >= t ? h - t : capacity_ - (t - h);
  }

  size_t capacity() const {return capacity_ - 1;}

  uint64_t pushed() const {return pushed_.load(std::memory_order_relaxed);}
  uint64_t popped() const {return popped_.load(std::memory_order_relaxed);}
  uint64_t dropped() const {return dropped_.load(std::memory_order_relaxed);}

private:
  size_t advance(size_t idx) const {return (idx + 1) % capacity_;}

  std::vector<T> buf_;
  size_t capacity_;
  std::atomic<size_t> head_{0};
  std::atomic<size_t> tail_{0};
  std::atomic<uint64_t> pushed_{0};
  std::atomic<uint64_t> popped_{0};
  std::atomic<uint64_t> dropped_{0};
};

}  // namespace runtime_logging
