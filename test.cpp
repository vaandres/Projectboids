#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "boids.hpp"

TEST_CASE("Testing dist function")
{
  bds::boid b1{1, 2, 3, 4};
  bds::boid b2{3, 2, 0, 0};

  auto distance = bds::dist(b1, b2);
  CHECK(distance == 2);

  bds::boid b3{3, 5, 0, 0};
  distance = bds::dist(b1, b3);
  CHECK(distance == doctest::Approx(3.605).epsilon(0.01));
}

TEST_CASE("Testing neighbours function")
{
  bds::boid b1{1, 2, 0, 0};
  bds::boid b2{3, 3, 0, 0};
  bds::boid b3{3, 2, 0, 0};
  bds::boid b4{5, 5, 0, 0};
  bds::boid b5{100, 60, 0, 0};
  bds::boid b6{40, 40, 0, 0};
  bds::boid b7{200, 50, 0, 0};

  std::vector<bds::boid> flock{b2, b3, b4, b5, b6, b7};
  SUBCASE("Testing neighbours function with d = 3")
  {
    double d{3};
    auto neighbours = bds::neighbours(b1, flock, d);
    CHECK(neighbours.size() == 2);
  }
  SUBCASE("Testing neighbours function with d = 1")
  {
    double d{2};
    auto neighbours = bds::neighbours(b1, flock, d);
    CHECK(neighbours.size() == 0);
  }
  SUBCASE("Testing neighbours function with d = 100")
  {
    double d{100};
    auto neighbours = bds::neighbours(b1, flock, d);
    CHECK(neighbours.size() == 4);
  }
}

TEST_CASE("Testing velocity method")
{
  bds::boid b1{0, 70, 4, 5};
  bds::boid b2{64, 21, -50, 100};
  bds::boid b3{0, 0, 0, 0};
  CHECK(b1.velocity()[0] == 4);
  CHECK(b1.velocity()[1] == 5);
  CHECK(b2.velocity()[0] == -50);
  CHECK(b2.velocity()[1] == 100);
  CHECK(b3.velocity()[0] == 0);
  CHECK(b3.velocity()[1] == 0);
}

TEST_CASE("Testing position method")
{
  bds::boid b1{0, 70, 4, 5};
  bds::boid b2{64, 21, -50, 100};
  bds::boid b3{0, 0, 0, 0};
  CHECK(b1.position()[0] == 0);
  CHECK(b1.position()[1] == 70);
  CHECK(b2.position()[0] == 64);
  CHECK(b2.position()[1] == 21);
  CHECK(b3.position()[0] == 0);
  CHECK(b3.position()[1] == 0);
  // ricordati di verificare che la posizione sia positiva(assert o exception?)
}
// testing alignment function
TEST_CASE("Testing alignment function")
{
  // per semplicià tutti i boid sono vicini
  bds::boid b1{0, 0, 0.5, 0.5};
  bds::boid b2{1, 0, -1.0, 1.0};
  bds::boid b3{1, 0, 1.5, 1.5};
  bds::boid b4{1, 0, 2.0, 2.0};
  bds::boid b5{1, 0, -2.5, 2.5};
  bds::boid b6{1, 0, 3.0, 3.0};
  bds::boid b7{1, 0, 3.5, 3.5};
  std::vector<bds::boid> flock{b2, b3, b4, b5, b6, b7};
  double d{10};
  double a{1};
  auto alignment_vel = bds::alignment(b1, flock, d, a);
  CHECK(alignment_vel[0] == doctest::Approx(0.58).epsilon(0.01));
  CHECK(alignment_vel[1] == doctest::Approx(1.75).epsilon(0.01));
}