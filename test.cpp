#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
// #include "boids.hpp" //forse non serve
#include "doctest.h"
#include "operator.hpp"

TEST_CASE("Testing dist()")
{
  bds::Boid b1{1, 2, 3, 4};
  bds::Boid b2{3, 2, 0, 0};

  auto distance = bds::dist(b1, b2);
  CHECK(distance == 2);

  bds::Boid b3{3, 5, 0, 0};
  distance = bds::dist(b1, b3);
  CHECK(distance == doctest::Approx(3.605).epsilon(0.01));
}

TEST_CASE("Testing absoluteVelocity()")
{
  bds::Boid b1{0, 0, 3, 4};
  bds::Boid b2{0, 0, 0, 0};
  bds::Boid b3{2, 4, -23, 43};

  CHECK(b1.absoluteVelocity() == 5);
  CHECK(b2.absoluteVelocity() == 0);
  CHECK(b3.absoluteVelocity() == doctest::Approx(48.7647).epsilon(0.0001));
}

TEST_CASE("Testing velocityLimit")
{
  double Vmax{3};
  bds::Boid b{0, 0, 1, 1};
  bds::velocityLimit(b, Vmax);
  CHECK(b.velocity().vx == 1);
  CHECK(b.velocity().vy == 1);

  b.setVelocity({-3, 4});
  bds::velocityLimit(b, Vmax);
  CHECK(b.velocity().vx == doctest::Approx(-1.8));
  CHECK(b.velocity().vy == doctest::Approx(2.4));
  CHECK(b.absoluteVelocity() == 3);
}

TEST_CASE("Testing neighbours function")
{
  bds::Boid b1{1, 2, 0, 0};
  bds::Boid b2{3, 3, 0, 0};
  bds::Boid b3{3, 2, 0, 0};
  bds::Boid b4{5, 5, 0, 0};
  bds::Boid b5{100, 60, 0, 0};
  bds::Boid b6{40, 40, 0, 0};
  bds::Boid b7{200, 50, 0, 0};

  std::vector<bds::Boid> flock{b2, b3, b4, b5, b6, b7};
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
  bds::Boid b1{0, 70, 2.4, 2};
  bds::Boid b2{64, 21, -4, -2.1};
  bds::Boid b3{0, 0, 0, 0};
  CHECK(b1.velocity().vx == 2.4);
  CHECK(b1.velocity().vy == 2);
  CHECK(b2.velocity().vx == -4);
  CHECK(b2.velocity().vy == -2.1);
  CHECK(b3.velocity().vx == 0);
  CHECK(b3.velocity().vy == 0);
}

TEST_CASE("Testing position method")
{
  bds::Boid b1{0, 70, 2.4, 2};
  bds::Boid b2{64, 21, -4, -2.1};
  bds::Boid b3{0, 0, 0, 0};
  CHECK(b1.position().x == 0);
  CHECK(b1.position().y == 70);
  CHECK(b2.position().x == 64);
  CHECK(b2.position().y == 21);
  CHECK(b3.position().x == 0);
  CHECK(b3.position().y == 0);
  // ricordati di verificare che la posizione sia positiva(assert o exception?)
}

TEST_CASE("Testing separation function")
{
  bds::Boid b1{13., 11., 3., 4.};
  bds::Boid b2{12., 7., 0., 0.};
  bds::Boid b3{20., 10., 2., 0.};
  bds::Boid b4{15.5, 12., 1., 0.};
  bds::Boid b5{14., 12.2, 0., 5.};
  bds::Boid b6{33., 11.4, 4., 0.};

  std::vector<bds::Boid> flock{b2, b3, b4, b5, b6};

  SUBCASE("Testing separation function with neighbours")

  {
    double ds{4};
    double s{0.2};

    auto sep_vel = bds::separation(b1, flock, ds, s);
    CHECK(sep_vel.vx == doctest::Approx(-0.7));
    CHECK(sep_vel.vy == doctest::Approx(-0.44));
  }

  SUBCASE("Testing separation function without neighbours")
  {
    double ds{1};
    double s{0.4};

    auto sep_vel = bds::separation(b1, flock, ds, s);
    CHECK(sep_vel.vx == doctest::Approx(0.));
    CHECK(sep_vel.vy == doctest::Approx(0.));
  }
}

// testing alignment function
TEST_CASE("Testing alignment function")
{
  bds::Boid b1{4., 8., 0.5, 0.5};
  bds::Boid b2{5., 10., -1.0, 1.0};
  bds::Boid b3{2., 7., 1.5, 1.5};
  bds::Boid b4{11., 5., 2.0, 2.0};
  bds::Boid b5{9., 8., -2.5, 2.5};
  bds::Boid b6{7., 10., 3.0, 3.0};
  bds::Boid b7{5., 5., 3.5, 3.5};
  std::vector<bds::Boid> flock{b2, b3, b4, b5, b6, b7};

  SUBCASE("Testing alignment function with neighbours")
  {
    double d{10.};
    double a{0.5};
    auto alignment_vel = bds::alignment(b1, flock, d, a);
    CHECK(alignment_vel.vx == doctest::Approx(0.291).epsilon(0.01));
    CHECK(alignment_vel.vy == doctest::Approx(0.88).epsilon(0.01));
  }

  SUBCASE("Testing alignment function with no neighbours")
  {
    double d{2.};
    double a{0.5};
    auto alignment_vel = bds::alignment(b1, flock, d, a);
    CHECK(alignment_vel.vx == 0);
    CHECK(alignment_vel.vy == 0);
  }
}

TEST_CASE("Testing cohesion function")
{
  bds::Boid b1{13., 7., 3., 4.};
  bds::Boid b2{10., 5., -2., 5.};
  bds::Boid b3{30., 5., 3., 2.};
  bds::Boid b4{11., 45., 0, 1.};
  bds::Boid b5{8., 6., 3., 4.};
  bds::Boid b6{7., 6.5, 0., 0.};
  bds::Boid b7{10.5, 7., 0., 0.};
  bds::Boid b8{7.5, 15., 0., 0.};
  std::vector<bds::Boid> flock{b2, b3, b4, b5, b6, b7, b8};

  SUBCASE("Testing cohesion function with neighbours")
  {
    double d{7};
    double c{0.5};
    auto coh_vel = bds::cohesion(b1, flock, d, c);
    CHECK(coh_vel.vx == doctest::Approx(-2.06).epsilon(0.01));
    CHECK(coh_vel.vy == doctest::Approx(-0.44).epsilon(0.01));
  }

  SUBCASE("Testing cohesion function without neighbours")
  {
    double d{1};
    double c{0.5};
    auto coh_vel = bds::cohesion(b1, flock, d, c);
    CHECK(coh_vel.vx == doctest::Approx(0.).epsilon(0.01));
    CHECK(coh_vel.vy == doctest::Approx(0.).epsilon(0.01));
  }
}

TEST_CASE("Testing edgeForce function")
{
  SUBCASE("Testing edgeForce function with Boid inside the window")
  {
    bds::Boid b1{90, 80, -1.3, -2.};
    bds::Boid b2{500 - 90, 330 - 80, 0, 0};
    unsigned int w{500};
    unsigned int h{330};
    auto edge_force_b1 = bds::edgeForce(b1, w, h);
    auto edge_force_b2 = bds::edgeForce(b2, w, h);
    CHECK(edge_force_b1.vx == doctest::Approx(0.385).epsilon(0.001));
    CHECK(edge_force_b1.vy == doctest::Approx(1).epsilon(0.001));
    CHECK(edge_force_b1.vx == -edge_force_b2.vx);
    CHECK(edge_force_b1.vy == -edge_force_b2.vy);
  }

  SUBCASE("Testing edgeForce function with Boid outside the window")
  {
    bds::Boid b1{465, 290, 6.5, 2.};
    unsigned int w{500};
    unsigned int h{330};
    auto edge_force = bds::edgeForce(b1, w, h);
    CHECK(edge_force.vx == doctest::Approx(-72.89).epsilon(0.01));
    CHECK(edge_force.vy == doctest::Approx(-45.2).epsilon(0.01));
  }
}

TEST_CASE("Testing stats")
{
  bds::Boid b1{0, 70, 2.4, 2};
  bds::Boid b2{64, 21, -4, -2.1};
  bds::Boid b3{0, 0, 0, 0};
  bds::Boid b4{1., 2., 1.5, 1.5};
  bds::Boid b5{3., 0., 1., 0.};

  SUBCASE("Testing stats")
  {
    std::vector<bds::Boid> flock1{b1, b2, b3};
    const auto stats = bds::stats(flock1);

    CHECK(stats.dis_mean == doctest::Approx(1.9223));
    CHECK(stats.dis_err == doctest::Approx(0.0874421));
    CHECK(stats.speed_mean == doctest::Approx(0.0673968));
    CHECK(stats.speed_err == doctest::Approx(0.0288547));
  }

  SUBCASE("Testing stats method with 0 Boids")
  {
    std::vector<bds::Boid> flock2;
    const auto stats = bds::stats(flock2);
    CHECK(stats.dis_mean == 0);
    CHECK(stats.dis_err == 0);
    CHECK(stats.speed_mean == 0);
    CHECK(stats.speed_err == 0);
  }

  SUBCASE("Testing stats with 2 Boids")
  {
    std::vector<bds::Boid> flock3{b4, b5};
    const auto stats = bds::stats(flock3);
    CHECK(stats.dis_mean == doctest::Approx(0.07483547));
    CHECK(stats.dis_err == doctest::Approx(0.));
    CHECK(stats.speed_mean == doctest::Approx(0.041292));
    CHECK(stats.speed_err == doctest::Approx(0.0104893));
  }

  SUBCASE("Testing stats with 1 Boid")
  {
    std::vector<bds::Boid> flock4{b2};
    const auto stats = bds::stats(flock4);
    CHECK(stats.dis_mean == doctest::Approx(0.));
    CHECK(stats.dis_err == doctest::Approx(0.));
    CHECK(stats.speed_mean == doctest::Approx(0.119532));
    CHECK(stats.speed_err == doctest::Approx(0.));
  }
}

// test escape function
TEST_CASE("Testing escape function")
{
  bds::Boid p1{36., 40., 5., 3.};
  bds::Boid b1{39., 41., 2., 3.};

  SUBCASE("Testing escape function with a boid near a predator")
  {
    double d{4.};
    double e{1.5};
    auto escape_vel = bds::escape(p1, b1, d, e);
    CHECK(escape_vel.vx == doctest::Approx(4.5));
    CHECK(escape_vel.vy == doctest::Approx(1.5));

    escape_vel = bds::escape(p1, b1, d, 0);
    CHECK(escape_vel.vx == 0);
    CHECK(escape_vel.vy == 0);
  }

  SUBCASE("Testing escape function with a boid far from a predator")
  {
    double d{1.};
    double e{1.5};
    auto escape_vel = bds::escape(p1, b1, d, e);
    CHECK(escape_vel.vx == 0);
    CHECK(escape_vel.vy == 0);
  }
}

// test follow function
TEST_CASE("Testing follow function")
{
  bds::Boid p1{32., 23., 3., 2.};
  bds::Boid b1{30., 27., -5., 2.3};
  bds::Boid b2{22., 20., 3.2, 2.1};
  bds::Boid b3{29., 23., -1., 1.2};
  bds::Boid b4{31., 26., 0., 0.};

  SUBCASE("Testing follow function with a flock of boids")
  {
    std::vector<bds::Boid> flock{b1, b2, b3, b4};
    double f{2.};
    auto follow_vel = bds::follow(p1, flock, f);
    CHECK(follow_vel.vx == doctest::Approx(-6.));
    CHECK(follow_vel.vy == doctest::Approx(0.));
  }

  SUBCASE("Testing follow function with an empty flock")
  {
    std::vector<bds::Boid> flock{};
    double f{2.5};
    bds::Velocity follow_vel = bds::follow(p1, flock, f);
    follow_vel               = bds::follow(p1, flock, f);
    follow_vel               = bds::follow(p1, flock, f);
    CHECK(follow_vel.vx == 0);
    CHECK(follow_vel.vy == 0);
  }
}


TEST_CASE("Testing setVelocity method")
{
  bds::Boid b1{0, 0, 0, 0};
  bds::Velocity newVel{1, 1.5};
  b1.setVelocity(newVel);
  CHECK(b1.velocity().vx == 1);
  CHECK(b1.velocity().vy == 1.5);
  newVel = {0, 0};
  b1.setVelocity(newVel);
  CHECK(b1.velocity().vx == 0);
  CHECK(b1.velocity().vy == 0);
  newVel = {-1.5, -1};
  b1.setVelocity(newVel);
  CHECK(b1.velocity().vx == -1.5);
  CHECK(b1.velocity().vy == -1);
}

TEST_CASE("Testing eat")
{
  bds::Boid b1{1, 3.9, 0, 0};
  bds::Boid b2{1, 4, 0, 0};
  bds::Boid b3{2, 2, 0, 0};
  bds::Boid b4{0, 0, 0, 0};
  bds::Boid b5{5, 2, 0, 0};
  bds::Boid p1{1, 2, 0, 0};
  std::vector<bds::Boid> flock{b1, b2, b3, b4, b5};
  bds::eat(p1, flock, 2);
  CHECK(flock.size() == 3);
  CHECK(flock[0].position().x == b2.position().x);
  CHECK(flock[1].position().x == b4.position().x);
  CHECK(flock[2].position().x == b5.position().x);
}

TEST_CASE("Testing updatePosition")
{
  bds::Boid b1{50, 45, 1000.63, 1070.92};
  bds::Boid b2{1, 2, 0, 0};
  bds::Boid b3{0, 0, 1.5, 3.32414};
  b1.updatePosition();
  b2.updatePosition();
  b3.updatePosition();

  CHECK(b1.position().x == doctest::Approx(83.35).epsilon(0.01));
  CHECK(b1.position().y == doctest::Approx(80.70).epsilon(0.01));

  CHECK(b2.position().x == 1);
  CHECK(b2.position().y == 2);

  CHECK(b3.position().x == doctest::Approx(0.05));
  CHECK(b3.position().y == doctest::Approx(0.111).epsilon(0.001));
}

TEST_CASE("Testing operator+ on Position")
{
  bds::Position p1{1, 1};
  bds::Position p2{2, 2};
  auto c = p1 + p2;
  CHECK(c.x == 3);
  CHECK(c.y == 3);

  p1 = {0, 0};
  p2 = {0, 0};
  c  = p1 + p2;
  CHECK(c.x == 0);
  CHECK(c.y == 0);

  p1 = {2.5, 1};
  p2 = {-1, -1};
  c  = p1 + p2;
  CHECK(c.x == 1.5);
  CHECK(c.y == 0);
}

TEST_CASE("Testing operator- on Position")
{
  bds::Position p1{1, 1};
  bds::Position p2{2, 2};
  auto c = p1 - p2;
  CHECK(c.x == -1);
  CHECK(c.y == -1);

  p1 = {0, 0};
  p2 = {0, 0};
  c  = p1 - p2;
  CHECK(c.x == 0);
  CHECK(c.y == 0);

  p1 = {2.5, 1};
  p2 = {-1, -1};
  c  = p1 - p2;
  CHECK(c.x == 3.5);
  CHECK(c.y == 2);
}

TEST_CASE("Testing operator/ on Position")
{
  bds::Position p1{1, 1};
  double b{2};
  auto c = p1 / b;
  CHECK(c.x == 0.5);
  CHECK(c.y == 0.5);

  p1 = {3, -2};
  b  = 1;
  c  = p1 / b;
  CHECK(c.x == 3);
  CHECK(c.y == -2);

  p1 = {2.5, 1};
  b  = -1;
  c  = p1 / b;
  CHECK(c.x == -2.5);
  CHECK(c.y == -1);
}

TEST_CASE("Testing operator* on Position")
{
  bds::Position p1{1, 1};
  double b{2};
  auto c = p1 * b;
  CHECK(c.x == 2);
  CHECK(c.y == 2);

  p1 = {3, -2};
  b  = 0;
  c  = p1 * b;
  CHECK(c.x == 0);
  CHECK(c.y == 0);

  p1 = {2.5, 1};
  b  = -1;
  c  = p1 * b;
  CHECK(c.x == -2.5);
  CHECK(c.y == -1);
}

TEST_CASE("Testing operator+ on Velocity")
{
  bds::Velocity v1{1, 1};
  bds::Velocity v2{2, 2};
  auto c = v1 + v2;
  CHECK(c.vx == 3);
  CHECK(c.vy == 3);

  v1 = {0, 0};
  v2 = {0, 0};
  c  = v1 + v2;
  CHECK(c.vx == 0);
  CHECK(c.vy == 0);

  v1 = {2.5, 1};
  v2 = {-1, -1};
  c  = v1 + v2;
  CHECK(c.vx == 1.5);
  CHECK(c.vy == 0);
}

TEST_CASE("Testing operator- on Velocity")
{
  bds::Velocity v1{1, 1};
  bds::Velocity v2{2, 2};
  auto c = v1 - v2;
  CHECK(c.vx == -1);
  CHECK(c.vy == -1);

  v1 = {0, 0};
  v2 = {0, 0};
  c  = v1 - v2;
  CHECK(c.vx == 0);
  CHECK(c.vy == 0);

  v1 = {2.5, 1};
  v2 = {-1, -1};
  c  = v1 - v2;
  CHECK(c.vx == 3.5);
  CHECK(c.vy == 2);
}

TEST_CASE("Testing operator* on Velocity")
{
  bds::Velocity v1{1, 1};
  double b{2};
  auto c = v1 * b;
  CHECK(c.vx == 2);
  CHECK(c.vy == 2);

  v1 = {3, -2};
  b  = 0;
  c  = v1 * b;
  CHECK(c.vx == 0);
  CHECK(c.vy == 0);

  v1 = {2.5, 1};
  b  = -1;
  c  = v1 * b;
  CHECK(c.vx == -2.5);
  CHECK(c.vy == -1);
}

TEST_CASE("Testing operator/ on Velocity")
{
  bds::Velocity v1{1, 1};
  double b{2};
  auto c = v1 / b;
  CHECK(c.vx == 0.5);
  CHECK(c.vy == 0.5);

  v1 = {3, -2};
  b  = 1;
  c  = v1 / b;
  CHECK(c.vx == 3);
  CHECK(c.vy == -2);

  v1 = {2.5, 1};
  b  = -1;
  c  = v1 / b;
  CHECK(c.vx == -2.5);
  CHECK(c.vy == -1);
}
