#include "boids.hpp"
#include <algorithm>
#include <cmath>
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