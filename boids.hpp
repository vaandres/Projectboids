#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cassert>
#include <vector>
#include <array>
#include <array>
namespace bds {
// Classe principale dei boids
class boid
{
 private:
  std::array<double,2> position_{0, 0};
  std::array<double,2> velocity_{0, 0};
  // Costruttore della classe boids
 public:
  boid(double x, double y, double vx, double vy)
      : position_{x, y}
      , velocity_{vx, vy}
  {}

  std::array<double,2> position() const;

  std::array<double,2> velocity() const;

  void setPosition(const std::array<double, 2>& newPos);

  void setVelocity(const std::array<double,2>& newVel);

  double absoluteVelocity();

};//fine classe boid

std::array<double,2> operator+(std::array<double,2>, std::array<double,2>);

std::array<double,2> operator*(std::array<double,2> v1, double k);

double dist(boid const&, boid const&);

std::vector<boid> neighbours(boid const&, std::vector<boid> const&, double);

std::array<double,2> separation(boid const&, std::vector<boid> const&, double,
                               double);

std::array<double,2> alignment(boid const&, std::vector<boid> const&, double,
                              double);

std::array<double,2> cohesion(boid const&, std::vector<boid> const&, double,
                             double);

std::array<double,2> edgeforce(boid const& b, int width, int height);

void velocitylimit(boid& b, double Vmax);

std::array<double,2> edgeforce(boid const& b, unsigned int width, unsigned int height);
} // namespace bds

#endif
