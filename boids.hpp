#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <array>
#include <cassert>
#include <vector>
namespace bds {
// Classe principale dei Boids
class Boid
{
 private:
  std::array<double, 2> position_{0, 0};
  std::array<double, 2> velocity_{0, 0};
  // Costruttore della classe Boids
 public:
  Boid(double x, double y, double vx,
       double vy) // aggiungere un assert per posizioni negative?
      : position_{x, y}
      , velocity_{vx, vy}
  {}

  std::array<double, 2> position() const;

  std::array<double, 2> velocity() const;

  void updatePosition();

  void setVelocity(const std::array<double, 2>& newVel);

  double absoluteVelocity() const;

}; // fine classe Boid

struct Statistics
{
  double dis_mean;
  double dis_err;
  double speed_mean;
  double speed_err;
};
// fine classe flock

double dist(Boid const&, Boid const&);

std::vector<Boid> neighbours(Boid const&, std::vector<Boid> const&, double);

std::array<double, 2> separation(Boid const&, std::vector<Boid> const&, double,
                                 double);

std::array<double, 2> alignment(Boid const&, std::vector<Boid> const&, double,
                                double);

std::array<double, 2> cohesion(Boid const&, std::vector<Boid> const&, double,
                               double);
std::array<double, 2> escape(Boid const&, Boid const&, double, double);

std::array<double, 2> follow(Boid const&, std::vector<Boid> const&, double);

void eat(Boid const&, std::vector<Boid>&, double);

std::array<double, 2> edgeforce(Boid const&, unsigned int, unsigned int);

void applyRules(Boid&, double, double, double, double, double, double,
                unsigned int, unsigned int, std::vector<Boid> const&, Boid&);

void velocitylimit(Boid&, double);

Statistics stats(std::vector<Boid> const&);

void RulesPred(Boid&, std::vector<Boid> const&, double, unsigned int,
               unsigned int);

} // namespace bds

#endif
