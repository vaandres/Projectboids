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
  position_ = position_ + velocity_;
}

// Funzione distanza tra due Boids
double bds::dist(Boid const& b1, Boid const& b2)
{
  auto pos1 = b1.position();
  auto pos2 = b2.position();
  return std::sqrt(std::pow(pos1[0] - pos2[0], 2)
                   + std::pow(pos1[1] - pos2[1], 2));
}

// Funzione cambio velocità del Boid
void bds::Boid::setVelocity(const std::array<double, 2>& newVel)
{
  velocity_ = newVel;
}

// Funzione per trovare i vicini di un Boid
std::vector<bds::Boid> bds::neighbours(Boid const& b1,
                                       std::vector<Boid> const& flock, double d)
{
  std::vector<bds::Boid> neighbours;
  std::copy_if(flock.begin(), flock.end(), std::back_inserter(neighbours),
               [&b1, d](bds::Boid const& b2) {
                 return dist(b1, b2) < d && dist(b1, b2) != 0;
               });
  return neighbours;
}

// Regola di separazione
std::array<double, 2> bds::separation(bds::Boid const& b1,
                                      std::vector<bds::Boid> const& flock,
                                      double ds, double s)
{
  auto neighbours = bds::neighbours(b1, flock, ds);

  std::array<double, 2> sep_vel{0, 0};

  sep_vel = std::accumulate(
      neighbours.begin(), neighbours.end(), std::array<double, 2>{0, 0},
      [&b1, ds, s](std::array<double, 2> acc, bds::Boid const& b) {
        acc = acc + (b.position() - b1.position());

        return acc;
      });

  return sep_vel * -s;
}
std::array<double, 2> bds::escape(bds::Boid const& p1, bds::Boid const& b1,
                                  double d, double c)
{
  std::vector<bds::Boid> pred = {p1};    // un po' dispendioso
  return separation(b1, pred, d, 3 * c); // magic number
}

// Funzione per allineare i Boids
std::array<double, 2> bds::alignment(Boid const& b1,
                                     std::vector<Boid> const& flock, double d,
                                     double a)
{
  auto neighbours = bds::neighbours(b1, flock, d);
  if (neighbours.empty())
    return {0, 0};
  std::array<double, 2> v = std::accumulate(
      neighbours.begin(), neighbours.end(), std::array<double, 2>{0, 0},
      [](std::array<double, 2> s, Boid const& b) { return s + b.velocity(); });
  return (v / static_cast<double>(neighbours.size()) - b1.velocity()) * a;
}

std::array<double, 2> bds::follow(bds::Boid const& p1, std::vector<bds::Boid> const& flock, double d) {
        auto closest = std::min_element(flock.begin(), flock.end(), [&p1](const bds::Boid& a, const bds::Boid& b) {
            return dist(p1, a) < dist(p1, b);
        });

        if (closest == flock.end()) {
            return {0.0, 0.0}; 
        }

        return cohesion(p1, neighbours(*closest, flock, d), d, 1) * 2.;
    }
/*std::array<double, 2> bds::follow(Boid const& p1,
                                  std::vector<Boid> const& flock, double d/* ,
                                  double Vmax )
{
  bds::Boid b0{0, 0, 0, 0};
  double dmin = 100.0;  //magic number

  for (const bds::Boid& b : flock) {
    double h = dist(p1, b);
    if (h <= dmin) {
      dmin = h;
      b0   = b;
    }
  }
  return cohesion(p1, neighbours(b0, flock, d), d, 1) * 2;
}*/

// Regola di coesione
std::array<double, 2> bds::cohesion(bds::Boid const& b1,
                                    std::vector<bds::Boid> const& flock,
                                    double d, double c)
{
  auto neighbours = bds::neighbours(b1, flock, d);
  if (neighbours.empty()) {
    return {0, 0};
  }
  std::array<double, 2> i{0, 0};
  std::array<double, 2> mass_c =
      std::accumulate(neighbours.begin(), neighbours.end(), i,
                      [&b1, c](std::array<double, 2> acc, bds::Boid const& b) {
                        return acc + b.position();
                      })
      / static_cast<double>(neighbours.size());

  return (mass_c - b1.position()) * c;
}

// Funzione che limita la velocità dei Boids
void bds::velocitylimit(Boid& b, double Vmax)
{
  double V{b.absoluteVelocity()};
  if (V > Vmax) {
    b.setVelocity(b.velocity() * (Vmax / V));
  }
}

// Funzione della forza di repulsione dei Boids
std::array<double, 2> bds::edgeforce(Boid const& b, unsigned int width,
                                     unsigned int height)
{
  double x{b.position()[0]};
  double y{b.position()[1]};

  double vx{0};
  double vy{0};

  vx = (std::pow(2, -x) - std::pow(2, (x - width)));

  vy = (std::pow(2, -y) - std::pow(2, (y - height)));

  std::array<double, 2> a{vx, vy};
  return a;
}

// Funzione che applica le regole che determinano il movimento del boid
void bds::applyRules(Boid& b1, double a, double c, double s, double d,
                     double ds, unsigned int windowWidth,
                     unsigned int windowHeight, std::vector<Boid> const& flock,
                     Boid& p1)
{
  b1.setVelocity(b1.velocity() + edgeforce(b1, windowWidth, windowHeight)
                 + alignment(b1, flock, d, a) + separation(b1, flock, ds, s)
                 + cohesion(b1, flock, d, c) + escape(p1, b1, d, c));
}

void bds::RulesPred(Boid& p1, std::vector<Boid> const& flock, double d,
                    unsigned int windowWidth, unsigned int windowHeight)
{
  p1.setVelocity(p1.velocity() + bds::follow(p1, flock, d /* ,Vmax */)
                 + bds::edgeforce(p1, windowWidth, windowHeight));
}

bds::Statistics bds::stats(std::vector<Boid> const& flock)
{
  double dis_mean{};
  double dis_sigma{};
  double dis_err{};
  double speed_mean{};
  double speed_sigma{};
  double speed_err{};

  int n = static_cast<int>(flock.size());
  double sum_dist{};
  double sum_dist2{};
  double sum_speed{};
  double sum_speed2{};
  double conv_fac{0.0264583333}; // fattore di conversione da pixel a cm

  if (n == 0)
    return {0., 0., 0., 0.};

  if (n == 1) {
    return {0., 0., flock[0].absoluteVelocity() * conv_fac, 0.};
  }
  if (n > 1) { 
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

  dis_mean = sum_dist / (n * (n - 1) / 2);
  dis_sigma =
      std::sqrt(std::abs(sum_dist2 / (n * (n - 1) / 2) - dis_mean * dis_mean));
  dis_err     = dis_sigma / std::sqrt(n * (n - 1) / 2);
  speed_mean  = sum_speed / n;
  speed_sigma = std::sqrt(std::abs(sum_speed2 / n - speed_mean * speed_mean));
  speed_err   = speed_sigma / std::sqrt(n);

  return {dis_mean, dis_err, speed_mean, speed_err};
}
