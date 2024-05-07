#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cassert>
#include <vector>
namespace bds {
// Classe principale dei boids
class boids {
private:
  std::vector<double> position_{0, 0};
  std::vector<double> velocity_{0, 0};

public:
  boids(double x, double y, double vx,
        double vy) { // Guarda se va bene costruttore così e se c'è algoritmo
    position_[0] = x;
    position_[1] = y;
    velocity_[0] = vx;
    velocity_[1] = vy;
  }

  std::vector<double> position();

  std::vector<double> velocity();

  void update_position(double x, double y);
};

double abs(boids const &b1, boids const &b2);

} // namespace bds

#endif