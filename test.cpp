#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "boids.hpp"

TEST_CASE("Testing dist()")
{
  bds::boid b1{1, 2, 3, 4};
  bds::boid b2{3, 2, 0, 0};

  auto distance = bds::dist(b1, b2);
  CHECK(distance == 2);

  bds::boid b3{3, 5, 0, 0};
  distance = bds::dist(b1, b3);
  CHECK(distance == doctest::Approx(3.605).epsilon(0.01));
}
