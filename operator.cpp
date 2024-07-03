#include "operator.hpp"

bds::Velocity bds::operator+(const Velocity& v1, const Velocity& v2)
{
  return {v1.vx + v2.vx, v1.vy + v2.vy};
}

bds::Velocity bds::operator-(const Velocity& v1, const Velocity& v2)
{
  return {v1.vx - v2.vx, v1.vy - v2.vy};
}

bds::Velocity bds::operator/(const Velocity& v, double scalar)
{
  return {v.vx / scalar, v.vy / scalar};
}

bds::Velocity bds::operator*(const Velocity& v, double scalar)
{
  return {v.vx * scalar, v.vy * scalar};
}

bds::Position bds::operator+(const Position& p1, const Position& p2)
{
  return {p1.x + p2.x, p1.y + p2.y};
}

bds::Position bds::operator-(const Position& p1, const Position& p2)
{
  return {p1.x - p2.x, p1.y - p2.y};
}

bds::Position bds::operator/(const Position& p, double scalar)
{
  return {p.x / scalar, p.y / scalar};
}

bds::Position bds::operator*(const Position& p, double scalar)
{
  return {p.x * scalar, p.y * scalar};
}
