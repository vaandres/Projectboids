#include "boids.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

std::vector<double> operator+(std::vector<double> v1, std::vector<double> v2)
{
  auto vxf               = v1[0] + v2[0];
  auto vyf               = v1[1] + v2[1];
  std::vector<double> vf = {vxf, vyf};
  return vf;
}
std::vector<double> operator*(std::vector<double> v1, double k)
{
  auto vxf               = k * v1[0];
  auto vyf               = k * v1[1];
  std::vector<double> vf = {vxf, vyf};
  return vf;
}

std::vector<double> bds::boid::position() const
{
  return position_;
}

std::vector<double> bds::boid::velocity() const
{
  return velocity_;
}

// Funzione distanza tra due boids
double bds::dist(boid const& b1, boid const& b2)
{
  auto pos1 = b1.position();
  auto pos2 = b2.position();
  return std::sqrt(std::pow(pos1[0] - pos2[0], 2)
                   + std::pow(pos1[1] - pos2[1], 2));
}

// Funzione per trovare i vicini di un boid
std::vector<bds::boid> bds::neighbours(boid const& b1,
                                       std::vector<boid> const& flock, double d)
{
  std::vector<bds::boid> neighbours;
  std::copy_if(flock.begin(), flock.end(), std::back_inserter(neighbours),
               [&b1, d](bds::boid const& b2) { return dist(b1, b2) < d; });
  return neighbours;
}

std::vector<double> bds::alignment(boid const& b1,
                                   std::vector<boid> const& flock, double d,
                                   double a)
{
  auto neighbours = bds::neighbours(b1, flock, d);
  if (neighbours.empty())
    return {0, 0};
  std::vector<double> v = std::accumulate(
      neighbours.begin(), neighbours.end(), std::vector<double>{0, 0},
      [](std::vector<double> v, boid const& b) { return v + b.velocity(); });
  return (v * (1.0 / neighbours.size()) + b1.velocity() * -1) * a;   //vedi se implementare anche operatore - per vettori velocità
};