#include "operator.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

// Metodo che rende la posizione di un Boid
bds::Position bds::Boid::position() const
{
  return position_;
}

// Metodo che rende le componenti della velocità di un Boid
bds::Velocity bds::Boid::velocity() const
{
  return velocity_;
}

// Metode che rende il modulo della velocità di un Boid
double bds::Boid::absoluteVelocity() const
{
  return std::sqrt(std::pow(velocity_.vx, 2) + std::pow(velocity_.vy, 2));
}

// Metodo di aggiornamento di posizione del Boid
void bds::Boid::updatePosition()
{
  position_.x += velocity_.vx / 30;
  position_.y += velocity_.vy / 30; // magic number
}

// Funzione cambio velocità del Boid
void bds::Boid::setVelocity(const Velocity& newVel)
{
  velocity_ = newVel;
}

// Funzione distanza tra due Boids
double bds::dist(Boid const& boid_1, Boid const& boid_2)
{
  return std::sqrt(std::pow(boid_1.position().x - boid_2.position().x, 2)
                   + std::pow(boid_1.position().y - boid_2.position().y, 2));
}

// Funzione per trovare i vicini di un Boid
std::vector<bds::Boid> bds::neighbours(Boid const& boid_1,
                                       std::vector<Boid> const& flock, double d)
{
  std::vector<Boid> neighbours;
  std::copy_if(flock.begin(), flock.end(), std::back_inserter(neighbours),
               [&boid_1, d](Boid const& boid_2) {
                 return dist(boid_1, boid_2) < d && dist(boid_1, boid_2) != 0;
               });
  return neighbours;
}

// Regola di separazione
bds::Velocity bds::separation(Boid const& boid_1,
                              std::vector<Boid> const& flock, double ds,
                              double s)
{
  auto neighbours = bds::neighbours(boid_1, flock, ds);

  Velocity sep_vel =
      std::accumulate(neighbours.begin(), neighbours.end(), Velocity{0, 0},
                      [&boid_1, ds, s](Velocity acc, Boid const& boid_2) {
                        acc.vx += (boid_2.position().x - boid_1.position().x);
                        acc.vy += (boid_2.position().y - boid_1.position().y);

                        return acc;
                      });

  return sep_vel * -s;
}

// Regola di allineamento
bds::Velocity bds::alignment(Boid const& boid_1, std::vector<Boid> const& flock,
                             double d, double a)
{
  auto neighbours = bds::neighbours(boid_1, flock, d);
  if (neighbours.empty())
    return {0, 0};
  Velocity alig_vel =
      std::accumulate(neighbours.begin(), neighbours.end(), Velocity{0, 0},
                      [](Velocity acc, Boid const& boid_2) {
                        return acc + boid_2.velocity();
                      })
          / static_cast<double>(neighbours.size())
      - boid_1.velocity();
  return alig_vel * a;
}

// Regola di coesione
bds::Velocity bds::cohesion(Boid const& boid_1, std::vector<Boid> const& flock,
                            double d, double c)
{
  auto neighbours = bds::neighbours(boid_1, flock, d);
  if (neighbours.empty()) {
    return {0, 0};
  }
  Position mass_c =
      std::accumulate(neighbours.begin(), neighbours.end(), Position{0, 0},
                      [&boid_1, c](Position acc, Boid const& boid_2) {
                        acc.x += boid_2.position().x;
                        acc.y += boid_2.position().y;
                        return acc;
                      })
      / static_cast<double>(neighbours.size());
  Velocity coh_vel{0, 0};
  coh_vel.vx = (mass_c.x - boid_1.position().x);
  coh_vel.vy = (mass_c.y - boid_1.position().y);
  return coh_vel * c;
}

// Regola di fuga
bds::Velocity bds::escape(Boid const& predator, Boid const& boid, double d,
                          double e)
{
  std::vector<Boid> pred = {predator};
  return separation(boid, pred, d, e);
}

// Regola di inseguimento
bds::Velocity bds::follow(Boid const& predator, std::vector<Boid> const& flock,
                          double f)
{
  auto closest_it =
      std::min_element(flock.begin(), flock.end(),
                       [&predator](const Boid& boid_1, const Boid& boid_2) {
                         return dist(predator, boid_1) < dist(predator, boid_2);
                       });

  if (closest_it == flock.end()) {
    return {0.0, 0.0};
  }

  auto closest = *closest_it;
  Velocity follow_vel{(closest.position().x - predator.position().x),
                      closest.position().y - predator.position().y};
  return follow_vel * f;
}

// Funzione che limita la velocità dei Boids
void bds::velocitylimit(Boid& boid, double Vmax)
{
  double V{boid.absoluteVelocity()};

  if (boid.absoluteVelocity() > Vmax) {
    boid.setVelocity(boid.velocity() * (Vmax / V));
  }
}

// Funzione della forza di repulsione dei Boids
bds::Velocity bds::edgeforce(Boid const& boid, unsigned int width,
                             unsigned int height)
{
  double x{boid.position().x};
  double y{boid.position().y};

  double vx{0};
  double vy{0};

  vx = (std::pow(1.1, -x + 80) - std::pow(1.1, (x - width + 80))); // magic number
  vy = (std::pow(1.1, -y + 80) - std::pow(1.1, (y - height + 80)));

  return Velocity{vx, vy};
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

// Funzione che applica le regole che determinano il movimento del predatore
void bds::RulesPred(Boid& predator, std::vector<Boid> const& flock, double f,
                    unsigned int windowWidth, unsigned int windowHeight)
{
  predator.setVelocity(predator.velocity() + follow(predator, flock, f)
                       + edgeforce(predator, windowWidth, windowHeight));
}

// Funzione che elimina i Boids che sono stati mangiati dal predatore
void bds::eat(Boid const& predator, std::vector<Boid>& flock, double range)
{
  flock.erase(std::remove_if(flock.begin(), flock.end(),
                             [&predator, range](Boid const& boid) {
                               return (dist(predator, boid) < range);
                             }),
              flock.end());
}

// Funzione che calcola le statistiche dello stormo
bds::Statistics bds::stats(std::vector<bds::Boid> const& flock)
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
