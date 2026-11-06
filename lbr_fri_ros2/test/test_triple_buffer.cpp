#include <array>
#include <atomic>
#include <cstdint>
#include <thread>

#include "gtest/gtest.h"

#include "lbr_fri_ros2/triple_buffer.hpp"

TEST(TestTripleBuffer, InitialValue) {
  lbr_fri_ros2::TripleBuffer<int> buffer(42);
  EXPECT_FALSE(buffer.update());
  EXPECT_EQ(buffer.read(), 42);
}

TEST(TestTripleBuffer, UpdateOnlyOnNewData) {
  lbr_fri_ros2::TripleBuffer<int> buffer(0);
  buffer.write(1);
  EXPECT_TRUE(buffer.update());
  EXPECT_EQ(buffer.read(), 1);
  EXPECT_FALSE(buffer.update()); // nothing new, keeps previous value
  EXPECT_EQ(buffer.read(), 1);
}

TEST(TestTripleBuffer, LatestWins) {
  lbr_fri_ros2::TripleBuffer<int> buffer(0);
  for (int i = 1; i <= 5; ++i) {
    buffer.write(i);
  }
  EXPECT_TRUE(buffer.update());
  EXPECT_EQ(buffer.read(), 5);
}

TEST(TestTripleBuffer, ConcurrentReadsAreConsistent) {
  // every published array holds the same value in all elements, a torn read would mix values
  using array_t = std::array<std::uint64_t, 64>;
  lbr_fri_ros2::TripleBuffer<array_t> buffer(array_t{});
  constexpr std::uint64_t N = 200000;
  std::atomic_bool done{false};

  std::thread producer([&]() {
    for (std::uint64_t i = 1; i <= N; ++i) {
      buffer.write_buffer().fill(i);
      buffer.publish();
    }
    done = true;
  });

  std::uint64_t last = 0;
  while (!done) {
    buffer.update();
    const auto &value = buffer.read();
    for (const auto &v : value) {
      ASSERT_EQ(v, value[0]);
    }
    ASSERT_GE(value[0], last); // never goes back in time
    last = value[0];
  }
  producer.join();
  buffer.update();
  EXPECT_EQ(buffer.read()[0], N); // the last publish is never lost
}
