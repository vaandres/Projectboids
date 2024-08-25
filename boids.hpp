#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <array>
#include <cassert>
#include <vector>
namespace bds {
// alcune costanti globali
inline constexpr unsigned int framerate{30};

inline constexpr double conv_fac{
    0.0264583333}; // fattore di conversione da pixel a cm

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
  {}
  Boid() = default;

  Position get_position() const;
  Velocity get_velocity() const;
  void update_position();
  void set_velocity(const Velocity&);
  double absolute_velocity() const;
};

// Classe per le statistiche
struct Statistics
{
  double dist_mean;
  double dist_err;
  double speed_mean;
  double speed_err;
};

// Dichiarazione delle funzioni del namespace

double distance(const Boid&, const Boid&);
std::vector<Boid> neighbours(const Boid&, const std::vector<Boid>&, double);
std::array<Velocity, 3> accumulator(const Boid&, const std::vector<Boid>&,
                                    double);
Velocity separation(std::array<Velocity, 3> const&, double);
Velocity alignment(Boid const&, std::vector<Boid> const&,
                   std::array<Velocity, 3> const&, double);
Velocity cohesion(Boid const&, std::vector<Boid> const&,
                  std::array<Velocity, 3> const&, double);
Velocity escape(const Boid&, const Boid&, double, double);
Velocity follow(const Boid&, const std::vector<Boid>&, double);
void eat(const Boid&, std::vector<Boid>&, double);
Velocity edge_force(const Boid&, unsigned int, unsigned int);
void velocity_limit(Boid&, double);
Statistics stats(const std::vector<Boid>&);
void apply_rules_predator(Boid&, const std::vector<Boid>&, double, unsigned int,
                          unsigned int);
Velocity vel_increment(Boid&, double, double, double, double, double, double,
                 unsigned int, unsigned int, const std::vector<Boid>&, Boid&, bool);

} // namespace bds

#endif
