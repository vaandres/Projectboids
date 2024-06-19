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
double bds::boid::absoluteVelocity() const
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
        acc = acc + (b.position() - b1.position());

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
      [](std::array<double, 2> s, boid const& b) { return s + b.velocity(); });
  return (v / static_cast<double>(neighbours.size()) - b1.velocity()) * a;
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
      / static_cast<double>(neighbours.size());

  return (mass_c - b1.position()) * c;
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

  vx = (std::pow(10 / x, 2) - std::pow(10 / (x - width), 2)) * 100;

  vy = (std::pow(10 / y, 2) - std::pow(10 / (y - height), 2)) * 100;

  std::array<double, 2> a{vx, vy};
  return a;
}
int bds::flock::size() const
{
  return static_cast<int>(flock_.size());
}

bds::statistics bds::flock::stats() const
{
  double dis_mean{};
  double dis_sigma{};
  double dis_err{};
  double speed_mean{};
  double speed_sigma{};
  double speed_err{};

  int n = static_cast<int>(flock_.size());
  double sum_dist{};
  double sum_dist2{};
  double sum_speed{};
  double sum_speed2{};
  double conv_fac{0.0264583333}; // fattore di conversione da pixel a cm

  sum_dist = std::accumulate(
      flock_.begin(), flock_.end(), 0.0, [this](double s, boid const& b1) {
        return s
             + std::accumulate(flock_.begin(), flock_.end(), 0.0,
                               [&b1](double t, boid const& b2) {
                                 return t + dist(b1, b2);
                               });
      });

  sum_dist2 = std::accumulate(
      flock_.begin(), flock_.end(), 0.0, [this](double s, boid const& b1) {
        return s
             + std::accumulate(flock_.begin(), flock_.end(), 0.0,
                               [&b1](double t, boid const& b2) {
                                 return t + dist(b1, b2) * dist(b1, b2);
                               });
      });

  sum_speed = std::accumulate(
      flock_.begin(), flock_.end(), 0.0,
      [](double s, boid const& b) { return s + b.absoluteVelocity(); });

  sum_speed2 = std::accumulate(
      flock_.begin(), flock_.end(), 0.0, [](double s, boid const& b) {
        return s + b.absoluteVelocity() * b.absoluteVelocity();
      });

  dis_mean    = sum_dist / (n * n) * conv_fac;
  dis_sigma   = std::sqrt(sum_dist2 / (n * n) - dis_mean * dis_mean) * conv_fac;
  dis_err     = dis_sigma / std::sqrt(n);
  speed_mean  = sum_speed / n * conv_fac;
  speed_sigma = std::sqrt(sum_speed2 / n - speed_mean * speed_mean) * conv_fac;
  speed_err   = speed_sigma / std::sqrt(n);

  return {dis_mean, dis_err, speed_mean, speed_err};
}
