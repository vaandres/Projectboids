#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cassert>
#include <vector>
namespace bds {
// Classe principale dei boids
class boids
{
 private:
  std::vector<double> position_{0, 0};
  std::vector<double> velocity_{0, 0};
// Costruttore della classe boids
public:
  boids(double x, double y, double vx, double vy)
      : position_{x, y}, velocity_{vx, vy} {}

  std::vector<double> position();

  std::vector<double> velocity();

  void update_position(double x, double y);
};

double abs(boids const& b1, boids const& b2);

} // namespace bds

#endif