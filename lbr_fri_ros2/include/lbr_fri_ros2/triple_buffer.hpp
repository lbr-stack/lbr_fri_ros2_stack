#ifndef LBR_FRI_ROS2__TRIPLE_BUFFER_HPP_
#define LBR_FRI_ROS2__TRIPLE_BUFFER_HPP_

#include <array>
#include <atomic>
#include <cstdint>

namespace lbr_fri_ros2 {
/**
 * @brief Wait-free single-producer single-consumer triple buffer. Neither side ever blocks, sleeps
 * or allocates, and the consumer always sees the latest published value.
 *
 * The producer owns one slot, the consumer owns one slot, and the third (middle) slot is exchanged
 * atomically. Publishing swaps the producer's slot with the middle slot, reading swaps the
 * consumer's slot with the middle slot if it holds unread data.
 *
 * Exactly one thread may call #write / #publish, and exactly one thread may call #update / #read
 * at a time.
 *
 * @tparam T Copy assignable type.
 */
template <typename T> class TripleBuffer {
public:
  TripleBuffer() = default;

  /**
   * @brief Construct a new TripleBuffer object with all slots set to #value.
   *
   * @param[in] value The initial value.
   */
  explicit TripleBuffer(const T &value) { buffers_.fill(value); }

  /**
   * @brief The producer's slot, to be filled before #publish. Producer only.
   *
   * @return T& The producer's slot.
   */
  inline T &write_buffer() { return buffers_[write_index_]; }

  /**
   * @brief Make the producer's slot visible to the consumer. Producer only.
   *
   */
  inline void publish() {
    write_index_ =
        middle_.exchange(write_index_ | DIRTY, std::memory_order_acq_rel) & INDEX_MASK;
  }

  /**
   * @brief Copy #value into the producer's slot and publish it. Producer only.
   *
   * @param[in] value The value to publish.
   */
  inline void write(const T &value) {
    write_buffer() = value;
    publish();
  }

  /**
   * @brief Fetch the latest published value, if any. Consumer only.
   *
   * @return true if a value was published since the last call.
   * @return false if nothing new was published, #read then returns the previous value.
   */
  inline bool update() {
    if (!(middle_.load(std::memory_order_relaxed) & DIRTY)) {
      return false;
    }
    read_index_ = middle_.exchange(read_index_, std::memory_order_acq_rel) & INDEX_MASK;
    return true;
  }

  /**
   * @brief The consumer's slot, valid until the next #update. Consumer only.
   *
   * @return const T& The latest value fetched by #update.
   */
  inline const T &read() const { return buffers_[read_index_]; }

protected:
  static constexpr std::uint8_t INDEX_MASK = 0b011; /**< Slot index bits of #middle_.*/
  static constexpr std::uint8_t DIRTY = 0b100;      /**< Set if #middle_ holds unread data.*/

  std::array<T, 3> buffers_{};
  std::uint8_t write_index_{0};          /**< Producer only.*/
  std::atomic<std::uint8_t> middle_{1};  /**< Exchanged between producer and consumer.*/
  std::uint8_t read_index_{2};           /**< Consumer only.*/
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__TRIPLE_BUFFER_HPP_
