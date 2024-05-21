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

/*TEST_CASE("Testing operator+")
{
  std::array<double, 2> v1{1, 2};
  std::array<double, 2> v2{3, 4};

  auto v3 = v1 + v2;
  CHECK(v3[0] == 4);
  CHECK(v3[1] == 6);
}

TEST_CASE("Testing operator*")
{
  std::array<double, 2> v1{1, 2};
  double k = 2;

  auto v2 = v1 * k;
  CHECK(v2[0] == 2);
  CHECK(v2[1] == 4);
}
*/

TEST_CASE("Testing separation()")
{
  bds::boid b1{1, 2, 3, 4};
  bds::boid b2{3, 2, 0, 0};
  bds::boid b3{3, 5, 0, 0};
  bds::boid b4{5, 5, 0, 0};

  std::vector<bds::boid> flock{b2, b3, b4};

  auto sep = bds::separation(b1, flock, 3, 1, 1);
  CHECK(sep[0] == 0.);
  CHECK(sep[1] == 0.);

}


TEST_CASE("Testing cohesion()")
{
  bds::boid b1{1, 2, 3, 4};
  bds::boid b2{3, 2, 0, 0};
  bds::boid b3{3, 5, 0, 0};
  bds::boid b4{5, 5, 0, 0};

  std::vector<bds::boid> flock{b2, b3, b4};

  auto coh = bds::cohesion(b1, flock, 3, 1);
  CHECK(coh[0] == doctest::Approx(3.0).epsilon(0.01));
  CHECK(coh[1] == doctest::Approx(4.0).epsilon(0.01));
}

