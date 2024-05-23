#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"

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

TEST_CASE("Testing absoluteVelocity()")
{
  bds::boid b1{0, 0, 3, 4};
  bds::boid b2{0, 0, 0, 0};
  bds::boid b3{2, 4, -23, 43};

  CHECK(b1.absoluteVelocity() == 5);
  CHECK(b2.absoluteVelocity() == 0);
  CHECK(b3.absoluteVelocity() == doctest::Approx(48.7647).epsilon(0.0001));
}

TEST_CASE("Testing velocitylimit")
{
  double Vmax{3};
  bds::boid b{0, 0, 1, 1};
  bds::velocitylimit(b, Vmax);
  CHECK(b.velocity()[0] == 1);

  b.setVelocity({-3, 4});
  bds::velocitylimit(b, Vmax);
  CHECK(b.velocity()[0] == doctest::Approx(-1.8));
  CHECK(b.velocity()[1] == doctest::Approx(2.4));
  CHECK(b.absoluteVelocity() == 3);
}