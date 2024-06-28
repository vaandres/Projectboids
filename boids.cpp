#include "boids.hpp"
#include "operator.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

// Metodo che rende la posizione di un Boid
std::array<double, 2> bds::Boid::position() const
{
  return position_;
}

// Metodo che rende le componenti della velocità di un Boid
std::array<double, 2> bds::Boid::velocity() const
{
  return velocity_;
}

// Metode che rende il modulo della velocità di un Boid
double bds::Boid::absoluteVelocity() const
{
  return std::sqrt(std::pow(velocity_[0], 2) + std::pow(velocity_[1], 2));
}

// Metodo di aggiornamento di posizione del Boid
void bds::Boid::updatePosition()
{
  position_ = position_ + velocity_/30;
}

// Funzione distanza tra due Boids
double bds::dist(Boid const& boid_1, Boid const& boid_2)
{
  auto pos_1 = boid_1.position();
  auto pos_2 = boid_2.position();
  return std::sqrt(std::pow(pos_1[0] - pos_2[0], 2)
                   + std::pow(pos_1[1] - pos_2[1], 2));
}

// Funzione cambio velocità del Boid
void bds::Boid::setVelocity(const std::array<double, 2>& newVel)
{
  velocity_ = newVel;
}

// Funzione per trovare i vicini di un Boid
std::vector<bds::Boid> bds::neighbours(Boid const& boid_1,
                                       std::vector<Boid> const& flock, double d)
{
  std::vector<bds::Boid> neighbours;
  std::copy_if(flock.begin(), flock.end(), std::back_inserter(neighbours),
               [&boid_1, d](bds::Boid const& boid_2) {
                 return dist(boid_1, boid_2) < d && dist(boid_1, boid_2) != 0;
               });
  return neighbours;
}

// Regola di separazione
std::array<double, 2> bds::separation(bds::Boid const& boid_1,
                                      std::vector<bds::Boid> const& flock,
                                      double ds, double s)
{
  auto neighbours = bds::neighbours(boid_1, flock, ds);

  std::array<double, 2> sep_vel{0, 0};

  sep_vel = std::accumulate(
      neighbours.begin(), neighbours.end(), std::array<double, 2>{0, 0},
      [&boid_1, ds, s](std::array<double, 2> acc, bds::Boid const& boid_2) {
        acc = acc + (boid_2.position() - boid_1.position());

        return acc;
      });

  return sep_vel * -s;
}
std::array<double, 2> bds::escape(bds::Boid const& predator,
                                  bds::Boid const& boid, double d, double e)
{
  std::vector<bds::Boid> pred = {predator}; // un po' dispendioso
  return separation(boid, pred, d, e);      // magic number
}

// Funzione per allineare i Boids

std::array<double, 2> bds::alignment(Boid const& boid_1,
                                     std::vector<Boid> const& flock, double d,
                                     double a)
{
  auto neighbours = bds::neighbours(boid_1, flock, d);
  if (neighbours.empty())
    return {0, 0};
  std::array<double, 2> alig_vel = std::accumulate(
      neighbours.begin(), neighbours.end(), std::array<double, 2>{0, 0},
      [](std::array<double, 2> acc, Boid const& boid_2) {
        return acc + boid_2.velocity();
      });
  return (alig_vel / static_cast<double>(neighbours.size()) - boid_1.velocity())
       * a;
}

// Regola di coesione
std::array<double, 2> bds::cohesion(bds::Boid const& boid_1,
                                    std::vector<bds::Boid> const& flock,
                                    double d, double c)
{
  auto neighbours = bds::neighbours(boid_1, flock, d);
  if (neighbours.empty()) {
    return {0, 0};
  }
  std::array<double, 2> mass_c =
      std::accumulate(
          neighbours.begin(), neighbours.end(), std::array<double, 2>{0, 0},
          [&boid_1, c](std::array<double, 2> acc, bds::Boid const& boid_2) {
            return acc + boid_2.position();
          })
      / static_cast<double>(neighbours.size());

  return (mass_c - boid_1.position()) * c;
}
std::array<double, 2> bds::follow(bds::Boid const& predator,
                                  std::vector<bds::Boid> const& flock, double f)
{
  auto closest_it = std::min_element(
      flock.begin(), flock.end(),
      [&predator](const bds::Boid& boid_1, const bds::Boid& boid_2) {
        return dist(predator, boid_1) < dist(predator, boid_2);
      });

  if (closest_it == flock.end()) {
    return {0.0, 0.0};
  }

  auto closest = *closest_it;
  return (closest.position() - predator.position()) * f;
}

// Funzione che limita la velocità dei Boids
void bds::velocitylimit(Boid& boid, double Vmax)
{
  double V{boid.absoluteVelocity()};
  if (V > Vmax) {
    boid.setVelocity(boid.velocity() * (Vmax / V));
  }
}

// Funzione della forza di repulsione dei Boids
std::array<double, 2> bds::edgeforce(Boid const& boid, unsigned int width,
                                     unsigned int height)
{
  double x{boid.position()[0]};
  double y{boid.position()[1]};

  double vx{0};
  double vy{0};

  vx = (std::pow(1.1, -x+80) - std::pow(1.1, (x - width+80)));

  vy = (std::pow(1.1, -y+80) - std::pow(1.1, (y - height+80)));

  std::array<double, 2> edge_vel{vx, vy};
  return edge_vel;
}

// Funzione che applica le regole che determinano il movimento del boid
void bds::applyRules(Boid& boid, double a, double c, double s, double d,
                     double ds, double e, unsigned int windowWidth,
                     unsigned int windowHeight, std::vector<Boid> const& flock,
                     Boid& predator)
{
  boid.setVelocity(
      boid.velocity() + edgeforce(boid, windowWidth, windowHeight)
      + alignment(boid, flock, d, a) + separation(boid, flock, ds, s)
      + cohesion(boid, flock, d, c) + escape(predator, boid, d, e));
}

void bds::RulesPred(Boid& predator, std::vector<Boid> const& flock, double f,
                    unsigned int windowWidth, unsigned int windowHeight)
{
  predator.setVelocity(predator.velocity() + bds::follow(predator, flock, f)
                       + bds::edgeforce(predator, windowWidth, windowHeight));
}

void bds::eat(Boid const& predator, std::vector<Boid>& flock, double range)
{
  flock.erase(std::remove_if(flock.begin(), flock.end(),
                             [&predator, range](Boid const& boid) {
                               return (dist(predator, boid) < range);
                             }),
              flock.end());
}

bds::Statistics bds::stats(std::vector<Boid> const& flock)
{
  double dis_mean{};
  double dis_sigma{};
  double dis_err{};
  double speed_mean{};
  double speed_sigma{};
  double speed_err{};

  int n_boids = static_cast<int>(flock.size());
  double sum_dist{};
  double sum_dist2{};
  double sum_speed{};
  double sum_speed2{};
  double conv_fac{0.0264583333}; // fattore di conversione da pixel a cm

  if (n_boids == 0)
    return {0., 0., 0., 0.};

  if (n_boids == 1) {
    return {0., 0., flock[0].absoluteVelocity() * conv_fac, 0.};
  }
  if (n_boids > 1) {
    for (auto b1_iter = flock.begin(); b1_iter != flock.end(); ++b1_iter) {
      for (auto b2_iter = b1_iter + 1; b2_iter != flock.end(); ++b2_iter) {
        sum_dist += dist(*b1_iter, *b2_iter) * conv_fac;
        sum_dist2 += dist(*b1_iter, *b2_iter) * dist(*b1_iter, *b2_iter)
                   * conv_fac * conv_fac;
      }
    }

    sum_speed = std::accumulate(flock.begin(), flock.end(), 0.0,
                                [](double s, Boid const& b) {
                                  return s + b.absoluteVelocity();
                                })
              * conv_fac;

    sum_speed2 = std::accumulate(
                     flock.begin(), flock.end(), 0.0,
                     [](double s, Boid const& b) {
                       return s + b.absoluteVelocity() * b.absoluteVelocity();
                     })
               * conv_fac * conv_fac;
  }

  dis_mean   = sum_dist / (n_boids * (n_boids - 1) / 2);
  dis_sigma  = std::sqrt(std::abs(sum_dist2 / (n_boids * (n_boids - 1) / 2)
                                  - dis_mean * dis_mean));
  dis_err    = dis_sigma / std::sqrt(n_boids * (n_boids - 1) / 2);
  speed_mean = sum_speed / n_boids;
  speed_sigma =
      std::sqrt(std::abs(sum_speed2 / n_boids - speed_mean * speed_mean));
  speed_err = speed_sigma / std::sqrt(n_boids);

  return {dis_mean, dis_err, speed_mean, speed_err};
}
