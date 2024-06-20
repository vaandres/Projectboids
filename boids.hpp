#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <array>
#include <cassert>
#include <vector>
namespace bds {
// Classe principale dei boids
class boid
{
 private:
  std::array<double, 2> position_{0, 0};
  std::array<double, 2> velocity_{0, 0};
  // Costruttore della classe boids
 public:
  boid(double x, double y, double vx,
       double vy) // aggiungere un assert per posizioni negative?
      : position_{x, y}
      , velocity_{vx, vy}
  {}

  std::array<double, 2> position() const;

  std::array<double, 2> velocity() const;

  void setPosition(const std::array<double, 2>& newPos);

  void setVelocity(const std::array<double, 2>& newVel);

  double absoluteVelocity()const;

}; // fine classe boid

struct statistics
{
   double dis_mean;
  double dis_err;
  double speed_mean;
  double speed_err;
};
class flock
{
 private:
  std::vector<boid> flock_;

 public:
  flock(std::vector<boid> const& boids)
      : flock_{boids}
  {}
  statistics stats() const;
  int size() const;

}; // fine classe flock

double dist(boid const&, boid const&);

std::vector<boid> neighbours(boid const&, std::vector<boid> const&, double);

std::array<double, 2> separation(boid const&, std::vector<boid> const&, double,
                                 double);

std::array<double, 2> alignment(boid const&, std::vector<boid> const&, double,
                                double);

std::array<double, 2> cohesion(boid const&, std::vector<boid> const&, double,
                               double);

std::array<double, 2> edgeforce(boid const& b, unsigned int width,
                                unsigned int height);

void velocitylimit(boid& b, double Vmax);
} // namespace bds

#endif
