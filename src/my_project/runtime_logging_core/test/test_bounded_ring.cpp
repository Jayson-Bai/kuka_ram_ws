#include <gtest/gtest.h>
#include <thread>
#include <vector>
#include "runtime_logging_core/bounded_ring.hpp"

using runtime_logging::BoundedRing;

TEST(BoundedRing, EmptyFullWrap)
{
  BoundedRing<int> ring(4);
  EXPECT_EQ(ring.capacity(), 4u);
  EXPECT_TRUE(ring.try_push(1));
  EXPECT_TRUE(ring.try_push(2));
  EXPECT_TRUE(ring.try_push(3));
  EXPECT_TRUE(ring.try_push(4));
  EXPECT_FALSE(ring.try_push(5));
  EXPECT_EQ(ring.dropped(), 1u);
  auto v = ring.try_pop();
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ(*v, 1);
  EXPECT_TRUE(ring.try_push(5));
  EXPECT_EQ(ring.size(), 4u);
  for (int expected : {2, 3, 4, 5}) {
    auto item = ring.try_pop();
    ASSERT_TRUE(item.has_value());
    EXPECT_EQ(*item, expected);
  }
  EXPECT_FALSE(ring.try_pop().has_value());
}

TEST(BoundedRing, ProducerNeverBlocksWhenFull)
{
  BoundedRing<int> ring(8);
  for (int i = 0; i < 8; ++i) {
    ASSERT_TRUE(ring.try_push(i));
  }
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(ring.try_push(99));
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(
    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 5);
}

TEST(BoundedRing, SpscConcurrency)
{
  BoundedRing<int> ring(1024);
  constexpr int kItems = 20000;
  std::atomic<bool> done{false};
  std::vector<int> got;
  got.reserve(kItems);
  std::thread consumer([&]() {
    while (!done.load() || !ring.empty()) {
      auto item = ring.try_pop();
      if (item) {got.push_back(*item);}
    }
  });
  int produced = 0;
  for (int i = 0; i < kItems; ++i) {
    if (ring.try_push(i)) {++produced;}
  }
  done.store(true);
  consumer.join();
  EXPECT_EQ(static_cast<int>(got.size()), produced);
  EXPECT_EQ(ring.pushed(), static_cast<uint64_t>(produced));
  EXPECT_EQ(ring.popped(), static_cast<uint64_t>(produced));
  EXPECT_EQ(ring.dropped(), static_cast<uint64_t>(kItems - produced));
}
