#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <array>
#include <cassert>
#include <vector>
namespace bds {

// Struct for position
struct Position
{
  double x;
  double y;
};

// Struct for velocity
struct Velocity
{
  double vx;
  double vy;
};

// Classe principale dei Boids
class Boid
{
 private:
  Position position_{0, 0};
  Velocity velocity_{0, 0};

 public:
  Boid(double x, double y, double vx, double vy)
      : position_{x, y}
      , velocity_{vx, vy}
  {
    assert(x >= 0 && y >= 0);
  }

  Position position() const;
  Velocity velocity() const;
  void updatePosition();
  void setVelocity(const Velocity&);
  double absoluteVelocity() const;
};

struct Statistics
{
  double dis_mean;
  double dis_err;
  double speed_mean;
  double speed_err;
};

double dist(const Boid&, const Boid&);
std::vector<Boid> neighbours(const Boid&, const std::vector<Boid>&, double);
Velocity separation(const Boid&, const std::vector<Boid>&, double, double);
Velocity alignment(const Boid&, const std::vector<Boid>&, double, double);
Velocity cohesion(const Boid&, const std::vector<Boid>&, double, double);
Velocity escape(const Boid&, const Boid&, double, double);
Velocity follow(const Boid&, const std::vector<Boid>&, double);
void eat(const Boid&, std::vector<Boid>&, double);
Velocity edgeForce(const Boid&, unsigned int, unsigned int);
void applyRules(Boid&, double, double, double, double, double, double,
                unsigned int, unsigned int, const std::vector<Boid>&, Boid&);
void velocityLimit(Boid&, double);
Statistics stats(const std::vector<Boid>&);
void rulesPred(Boid&, const std::vector<Boid>&, double, unsigned int,
               unsigned int);

} // namespace bds

#endif
