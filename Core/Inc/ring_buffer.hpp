#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_fdcan.h"
#include <array>
#include <cstddef>
#include <cstdint>

// Template for a fixed-size ring buffer
template <typename T, size_t Size> class RingBuffer {
private:
  std::array<T, Size> buffer_;
  volatile size_t head_ = 0; // ISR writes here
  volatile size_t tail_ = 0; // Main loop reads here

public:
  // Push an item
  inline bool push(const T &item) {
    size_t nextHead = (head_ + 1) % Size;
    if (nextHead == tail_)
      return false; // Full (drop silently)
    buffer_[head_] = item;
    head_ = nextHead;
    return true;
  }

  // Pop an item
  inline bool pop(T &item) {
    if (head_ == tail_)
      return false; // Empty
    item = buffer_[tail_];
    tail_ = (tail_ + 1) % Size;
    return true;
  }

  // Utility checks
  inline bool isEmpty() const { return head_ == tail_; }
  inline size_t size() const { return (head_ >= tail_) ? (head_ - tail_) : (Size - tail_ + head_); }
};

// FDCAN message struct
struct FDCAN_RxMessage {
  FDCAN_RxHeaderTypeDef header;
  std::array<uint8_t, 64> data; // Fixed max size
};

#endif /* RING_BUFFER_HPP */
