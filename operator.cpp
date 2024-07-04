#include "operator.hpp"

//Operatore di somma per la struct velocity
bds::Velocity bds::operator+(const Velocity& v1, const Velocity& v2)
{
  return {v1.vx + v2.vx, v1.vy + v2.vy};
}

//Operatore di sottrazione per la struct velocity
bds::Velocity bds::operator-(const Velocity& v1, const Velocity& v2)
{
  return {v1.vx - v2.vx, v1.vy - v2.vy};
}

//Operatore di divisione per la struct velocity
bds::Velocity bds::operator/(const Velocity& v, double scalar)
{
  return {v.vx / scalar, v.vy / scalar};
}

//Operatore di moltiplicazione per la struct velocity
bds::Velocity bds::operator*(const Velocity& v, double scalar)
{
  return {v.vx * scalar, v.vy * scalar};
}

//Operatore di somma per la struct di position
bds::Position bds::operator+(const Position& p1, const Position& p2)
{
  return {p1.x + p2.x, p1.y + p2.y};
}

//Operatore di sottrazione per la struct di position
bds::Position bds::operator-(const Position& p1, const Position& p2)
{
  return {p1.x - p2.x, p1.y - p2.y};
}

//Operatore di divisione per la struct di position
bds::Position bds::operator/(const Position& p, double scalar)
{
  return {p.x / scalar, p.y / scalar};
}

//Operatore di moltiplicazione per la struct di position
bds::Position bds::operator*(const Position& p, double scalar)
{
  return {p.x * scalar, p.y * scalar};
}
