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
std::array<double, 2> bds::follow(boid const& p1,
                                     std::vector<boid> const& flock, double d,
                                     double a){

                                      return alignment(p1,flock,d,a)*3;
                                     }
// Regola di coesione
std::array<double, 2> bds::cohesion(bds::boid const& b1,
                                    std::vector<bds::boid> const& flock,
                                    double d, double c)
{
  auto neighbours = bds::neighbours(b1, flock, d);
  if (neighbours.empty()) {
    return {0, 0};
  }
  std::array<double, 2> i{0, 0};
  std::array<double, 2> mass_c =
      std::accumulate(neighbours.begin(), neighbours.end(), i,
                      [&b1, c](std::array<double, 2> acc, bds::boid const& b) {
                        return acc + b.position();
                      })
      / static_cast<double>(neighbours.size());

  return (mass_c - b1.position()) * c;
}
std::array<double,2> bds::escape(bds::boid const& p1, std::vector<bds::boid> const& flock,
                                    double d, double c){
                                      return cohesion(p1, flock, d, -5*c);
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

  vx = (std::pow(2, -x) - std::pow(2, (x - width)));

  vy = (std::pow(2, -y) - std::pow(2, (y - height)));

  std::array<double, 2> a{vx, vy};
  return a;
}

// bds::statistics bds::stats(std::vector<boid> const& flock)
// {
//   double dis_mean{0.0};
//   double dis_sigma{0.0};
//   std::array<double, 2> vel_mean;
//   std::array<double, 2> vel_sigma;
//   int n = flock.size();
//   double conv_fac{0.0264583333};
//   std::array<double, 2> vel_mean;
//   std::array<double, 2> vel_sigma;
//   double sum_dist{};
//   double sum_dist2{};
//   std::array<double, 2> sum_vel{};
//   std::array<double, 2> sum_vel2{};
//   double conv_fac{0.0264583333}; // fattore di conversione da pixel a cm

//   sum_dist = std::accumulate(
//       flock.begin(), flock.end(), 0.0, [&flock](double s, boid const& b1) {
//         return s
//              + std::accumulate(flock.begin(), flock.end(), 0.0,
//                                [&b1](double s, boid const& b2) {
//                                  return s + dist(b1, b2);
//                                });
//       });
//   sum_dist2 = std::accumulate(
//       flock.begin(), flock.end(), 0.0, [&flock](double s, boid const& b1) {
//         return s
//              + std::accumulate(flock.begin(), flock.end(), 0.0,
//                                [&b1](double s, boid const& b2) {
//                                  return s + dist(b1, b2) * dist(b1, b2);
//                                });
//       });
//   sum_vel = std::accumulate(
//       flock.begin(), flock.end(), std::array<double, 2>{0, 0},
//       [](std::array<double, 2> s, boid const& b) { return s + b.velocity();
//       });

//   sum_vel2 =
//       std::accumulate(flock.begin(), flock.end(), std::array<double, 2>{0,
//       0},
//                       [](std::array<double, 2> s, boid const& b) {
//                         return s + b.velocity() * b.velocity();
//                       });

//   dis_mean     = sum_dist / (n * n);
//   dis_sigma    = std::sqrt(fabs(sum_dist2 / (n * n) - dis_mean * dis_mean));
//   vel_mean     = sum_vel / n;
//   vel_sigma[0] = std::sqrt(fabs(sum_vel2[0] / n - vel_mean[0] *
//   vel_mean[0])); vel_sigma[1] = std::sqrt(fabs(sum_vel2[1] / n - vel_mean[1]
//   * vel_mean[1])); return {dis_mean, dis_sigma, vel_mean, vel_sigma};

/*for (boid const& b1 : flock) {
  // pos_mean = ;
  vel_mean = vel_mean + b1.velocity();
  for (boid const& b2 : flock) {
    dis_mean = dis_mean + dist(b1, b2);
  }
}
vel_mean =
    std::accumulate(flock.begin(), flock.end(), std::array<double, 2>{0, 0},
                    [](std::array<double, 2> s, boid const& b) {
                      return s + b.velocity();
                    })
    / n;

double dis_mean =
    std::accumulate(flock.begin(), flock.end(), 0.0,
                    [&flock](double s, boid const& b1) {
                      return s
                           + std::accumulate(flock.begin(), flock.end(), 0.0,
                                             [&b1](double s, boid const& b2) {
                                               return s + dist(b1, b2);
                                             });
                    })
    / (n * n);


for (boid const& b1 : flock) {
  for (boid const& b2 : flock) {
    dis_sigma = dis_sigma
              + (dis_mean - conv_fac * dist(b1, b2))
                    * (dis_mean - conv_fac * dist(b1, b2));
  }
}
for (boid const& b : flock) {
  vel_sigma =
      vel_sigma + (vel_mean - b.velocity()) * (vel_mean - b.velocity());
}
dis_sigma    = std::sqrt(fabs(dis_sigma) / (n - 1));
vel_sigma[0] = std::sqrt(fabs(vel_sigma[0]) / (n - 1));
vel_sigma[1] = std::sqrt(fabs(vel_sigma[1]) / (n - 1));
return {dis_mean, dis_sigma, vel_mean, vel_sigma};*/
//}
