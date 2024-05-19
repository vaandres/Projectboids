#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cassert>
#include <vector>
namespace bds {
// Classe principale dei boids
class boid
{
 private:
  std::vector<double> position_{0, 0};
  std::vector<double> velocity_{0, 0};
  // Costruttore della classe boids
 public:
  boid(double x, double y, double vx, double vy)
      : position_{x, y}
      , velocity_{vx, vy}
  {}

  std::vector<double> position() const;

  std::vector<double> velocity() const;

  void update_position(double x, double y);
};

double dist(boid const&, boid const&);

std::vector<boid> neighbours(boid const&, std::vector<boid> const&, double);

std::vector<double> alignment(boid const&, std::vector<boid> const&, double,
                              double);

} // namespace bds

#endif