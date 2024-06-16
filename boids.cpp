#include "boids.hpp"
#include "operator.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

// Metodo che rende la posizione di un boid
std::array<double, 2> bds::boid::position() const
{
  return position_;
}

// Metodo che rende le componenti della velocità di un boid
std::array<double, 2> bds::boid::velocity() const
{
  return velocity_;
}

// Metode che rende il modulo della velocità di un boid
double bds::boid::absoluteVelocity()
{
  return std::sqrt(std::pow(velocity_[0], 2) + std::pow(velocity_[1], 2));
}

// Metodo di cambio di posizione del boid
void bds::boid::setPosition(const std::array<double, 2>& newPos)
{
  position_ = newPos;
}

// Funzione distanza tra due boids
double bds::dist(boid const& b1, boid const& b2)
{
  auto pos1 = b1.position();
  auto pos2 = b2.position();
  return std::sqrt(std::pow(pos1[0] - pos2[0], 2)
                   + std::pow(pos1[1] - pos2[1], 2));
}

// Funzione cambio velocità del boid
void bds::boid::setVelocity(const std::array<double, 2>& newVel)
{
  velocity_ = newVel;
}

// Funzione per trovare i vicini di un boid
std::vector<bds::boid> bds::neighbours(boid const& b1,
                                       std::vector<boid> const& flock, double d)
{
  std::vector<bds::boid> neighbours;
  std::copy_if(flock.begin(), flock.end(), std::back_inserter(neighbours),
               [&b1, d](bds::boid const& b2) {
                 return dist(b1, b2) < d && dist(b1, b2) != 0;
               });
  return neighbours;
}

// Regola di separazione
std::array<double, 2> bds::separation(bds::boid const& b1,
                                      std::vector<bds::boid> const& flock,
                                      double ds, double s)
{
  auto neighbours = bds::neighbours(b1, flock, ds);

  std::array<double, 2> sep_vel{0, 0};

  sep_vel = std::accumulate(
      neighbours.begin(), neighbours.end(), std::array<double, 2>{0, 0},
      [&b1, ds, s](std::array<double, 2> acc, bds::boid const& b) {
        if (dist(b1, b) < ds) {
          auto pos = b.position();
          acc[0] -= s
                  * std::pow((pos[0] - b1.position()[0]),
                             2); // usare operator+ e operator* per array
          acc[1] -= s * std::pow((pos[1] - b1.position()[1]), 2);
        }
        return acc;
      });

  return sep_vel * -s;
}

// Funzione per allineare i boids
std::array<double, 2> bds::alignment(boid const& b1,
                                     std::vector<boid> const& flock, double d,
                                     double a)
{
  auto neighbours = bds::neighbours(b1, flock, d);
  if (neighbours.empty())
    return {0, 0};
  std::array<double, 2> v = std::accumulate(
      neighbours.begin(), neighbours.end(), std::array<double, 2>{0, 0},
      [](std::array<double, 2> v, boid const& b) { return v + b.velocity(); });
  return (v / neighbours.size() - b1.velocity())
       * a; 
}

// Regola di coesione
std::array<double, 2> bds::cohesion(bds::boid const& b1,
                                    std::vector<bds::boid> const& flock,
                                    double d, double c)
{
  auto neighbours = bds::neighbours(b1, flock, d);
  if (neighbours.empty())
    return {0, 0};
  std::array<double, 2> mass_c =
      std::accumulate(neighbours.begin(), neighbours.end(), mass_c,
                      [&b1, c](std::array<double, 2> acc, bds::boid const& b) {
                        return acc + b.position();
                      })
      / neighbours.size();

    return (mass_c-b1.position())*c;
}

// Funzione che limita la velocità dei boids
void bds::velocitylimit(boid& b, double Vmax)
{
  double V{b.absoluteVelocity()};
  if (V > Vmax) {
    b.setVelocity(b.velocity() * (Vmax / V));
  }
}

// Funzione della forza di repulsione dei boids
std::array<double, 2> bds::edgeforce(boid const& b, unsigned int width,
                                     unsigned int height)
{
  double x{b.position()[0]};
  double y{b.position()[1]};

  double vx{0};
  double vy{0};

  vx = (std::pow(1 / x, 3) - std::pow(1 / (x - width), 3)) * 100;

  vy = (std::pow(1 / y, 3) - std::pow(1 / (y - width), 3)) * 100;

  std::array<double, 2> a{vx, vy};
  return a;
}