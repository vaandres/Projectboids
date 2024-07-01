#ifndef OPERATOR_HPP
#define OPERATOR_HPP

#include <array>
#include <cassert>
#include <vector>
#include "boids.hpp"

namespace bds {
Velocity operator+(const Velocity&, const Velocity&);

Velocity operator-(const Velocity&, const Velocity&);

Velocity operator/(const Velocity&, double);

Velocity operator*(const Velocity&, double);

Position operator+(const Position&, const Position&);

Position operator-(const Position&, const Position&);

Position operator/(const Position&, double);

Position operator*(const Position&, double);
} // namespace bds

#endif