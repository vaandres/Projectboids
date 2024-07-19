#include "operator.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

// Metodo che rende la posizione di un Boid
bds::Position bds::Boid::get_position() const
{
  return position_;
}

// Metodo che rende le componenti della velocità di un Boid
bds::Velocity bds::Boid::get_velocity() const
{
  return velocity_;
}

// Metode che rende il modulo della velocità di un Boid
double bds::Boid::absolute_velocity() const
{
  return std::sqrt(std::pow(velocity_.vx, 2) + std::pow(velocity_.vy, 2));
}

// Metodo di aggiornamento di posizione del Boid
void bds::Boid::update_position()
{
  position_.x += velocity_.vx / framerate;
  position_.y += velocity_.vy / framerate;
}

// Funzione cambio velocità del Boid
void bds::Boid::set_velocity(const Velocity& newVel)
{
  velocity_ = newVel;
}

// Funzione distanza tra due Boids
double bds::distance(Boid const& boid_1, Boid const& boid_2)
{
  return std::sqrt(
      std::pow(boid_1.get_position().x - boid_2.get_position().x, 2)
      + std::pow(boid_1.get_position().y - boid_2.get_position().y, 2));
}

// Funzione per trovare i vicini di un Boid
std::vector<bds::Boid> bds::neighbours(Boid const& boid_1,
                                       std::vector<Boid> const& flock, double d)
{
  std::vector<Boid> neighbours;
  std::copy_if(flock.begin(), flock.end(), std::back_inserter(neighbours),
               [&boid_1, d](Boid const& boid_2) {
                 return distance(boid_1, boid_2) < d
                     && distance(boid_1, boid_2) != 0;
               });
  return neighbours;
}

// Funzione che verifica se due Boids sono vicini
bool bds::are_neighbors(bds::Boid const& boid_1, bds::Boid const& boid_2,
                        double d)
{
  return distance(boid_1, boid_2) < d;
}

// Regola di separazione
bds::Velocity bds::separation(Boid const& boid_1,
                              std::vector<Boid> const& neighbours, double ds,
                              double s)
{
  Velocity sep_vel = std::accumulate(
      neighbours.begin(), neighbours.end(), Velocity{0, 0},
      [&boid_1, ds](Velocity acc, Boid const& boid_2) {
        if (distance(boid_1, boid_2) < ds) {
          acc.vx += (boid_2.get_position().x - boid_1.get_position().x);
          acc.vy += (boid_2.get_position().y - boid_1.get_position().y);
        }
        return acc;
      });

  return sep_vel * -s;
}

// Regola di allineamento
bds::Velocity bds::alignment(Boid const& boid_1,
                             std::vector<Boid> const& neighbours, double a)
{
  if (neighbours.empty())
    return {0, 0};
  Velocity alig_vel =
      std::accumulate(neighbours.begin(), neighbours.end(), Velocity{0, 0},
                      [](Velocity acc, Boid const& boid_2) {
                        return acc + boid_2.get_velocity();
                      })
          / static_cast<double>(neighbours.size())
      - boid_1.get_velocity();
  return alig_vel * a;
}

// Regola di coesione
bds::Velocity bds::cohesion(Boid const& boid_1,
                            std::vector<Boid> const& neighbours, double c)
{
  if (neighbours.empty()) {
    return {0, 0};
  }
  Position mass_c =
      std::accumulate(neighbours.begin(), neighbours.end(), Position{0, 0},
                      [&boid_1](Position acc, Boid const& boid_2) {
                        acc.x += boid_2.get_position().x;
                        acc.y += boid_2.get_position().y;
                        return acc;
                      })
      / static_cast<double>(neighbours.size());
  Velocity coh_vel{0, 0};
  coh_vel.vx = (mass_c.x - boid_1.get_position().x);
  coh_vel.vy = (mass_c.y - boid_1.get_position().y);
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
  auto closest_it = std::min_element(
      flock.begin(), flock.end(),
      [&predator](const Boid& boid_1, const Boid& boid_2) {
        return distance(predator, boid_1) < distance(predator, boid_2);
      });

  if (closest_it == flock.end()) {
    return {0.0, 0.0};
  }

  auto closest = *closest_it;
  Velocity follow_vel{(closest.get_position().x - predator.get_position().x),
                      closest.get_position().y - predator.get_position().y};
  return follow_vel * f;
}

// Funzione che limita la velocità dei Boids
void bds::velocity_limit(Boid& boid, double Vmax)
{
  if (boid.absolute_velocity() > Vmax) {
    boid.set_velocity(boid.get_velocity() * (Vmax / boid.absolute_velocity()));
  }
}

// Funzione della forza di repulsione dei Boids
bds::Velocity bds::edge_force(Boid const& boid, unsigned int width,
                              unsigned int height)
{
  double x{boid.get_position().x};
  double y{boid.get_position().y};

  double vx{0};
  double vy{0};

  vx = (std::pow(1.1, -x + 80) - std::pow(1.1, (x - width + 80)));
  vy = (std::pow(1.1, -y + 80) - std::pow(1.1, (y - height + 80)));

  return Velocity{vx, vy};
}

// Funzione che applica le regole che determinano il movimento del boid
void bds::apply_rules_boids(Boid& boid, double a, double c, double s, double d,
                            double ds, double e, unsigned int windowWidth,
                            unsigned int windowHeight,
                            std::vector<Boid> const& flock, Boid& predator)
{
  auto neighbours = bds::neighbours(boid, flock, d);
  boid.set_velocity(boid.get_velocity()
                    + edge_force(boid, windowWidth, windowHeight)
                    + alignment(boid, neighbours, a) + separation(boid, neighbours, ds, s)
                    + cohesion(boid, neighbours, c) + escape(predator, boid, d, e));
}

// Funzione che applica le regole che determinano il movimento del predatore
void bds::apply_rules_predator(Boid& predator, std::vector<Boid> const& flock,
                               double f, unsigned int windowWidth,
                               unsigned int windowHeight)
{
  predator.set_velocity(predator.get_velocity() + follow(predator, flock, f)
                        + edge_force(predator, windowWidth, windowHeight));
}

// Funzione che elimina i Boids che sono stati mangiati dal predatore
void bds::eat(Boid const& predator, std::vector<Boid>& flock, double range)
{
  flock.erase(std::remove_if(flock.begin(), flock.end(),
                             [&predator, range](Boid const& boid) {
                               return (distance(predator, boid) < range);
                             }),
              flock.end());
}

// Funzione che calcola le statistiche dello stormo
bds::Statistics bds::stats(std::vector<Boid> const& flock, double d)
{
  int n_boids = static_cast<int>(flock.size());
  double sum_dist{};
  double sum_dist2{};
  double sum_speed{};
  double sum_speed2{};
  int neighbours_count{};

  if (n_boids == 0)
    return {0., 0., 0., 0.};

  if (n_boids == 1) {
    return {0., 0., flock[0].absolute_velocity() * conv_fac, 0.};
  }
  if (n_boids > 1) {
    /*for (auto b1_iter = flock.begin(); b1_iter != flock.end(); ++b1_iter) {
      for (auto b2_iter = b1_iter + 1; b2_iter != flock.end(); ++b2_iter) {
        sum_dist += distance(*b1_iter, *b2_iter) * conv_fac;
        sum_dist2 += distance(*b1_iter, *b2_iter) * distance(*b1_iter, *b2_iter)
                   * conv_fac * conv_fac;
      }
    }*/
    // sum_dist of the distance between the  neighbours boids

    for (auto b1_iter = flock.begin(); b1_iter != flock.end(); ++b1_iter) {
      for (auto b2_iter = b1_iter + 1; b2_iter != flock.end(); ++b2_iter) {
        if (are_neighbors(*b1_iter, *b2_iter, d)) {
          double dist = distance(*b1_iter, *b2_iter);
          sum_dist += dist * conv_fac;
          sum_dist2 += dist * dist * conv_fac * conv_fac;
          neighbours_count++;
        }
      }
    }

    sum_speed = std::accumulate(flock.begin(), flock.end(), 0.0,
                                [](double acc, Boid const& boid) {
                                  return acc + boid.absolute_velocity();
                                })
              * conv_fac;

    sum_speed2 = std::accumulate(flock.begin(), flock.end(), 0.0,
                                 [](double acc, Boid const& boid) {
                                   return acc
                                        + boid.absolute_velocity()
                                              * boid.absolute_velocity();
                                 })
               * conv_fac * conv_fac;
  }

  const double dist_mean = sum_dist / neighbours_count;
  const double dist_err =
      std::sqrt(std::abs(sum_dist2 / neighbours_count - dist_mean * dist_mean));
  const double speed_mean = sum_speed / n_boids;
  const double speed_err =
      std::sqrt(std::abs(sum_speed2 / n_boids - speed_mean * speed_mean));

  return {dist_mean, dist_err, speed_mean, speed_err};
}
