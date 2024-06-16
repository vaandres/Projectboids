#include "operator.hpp"

std::array<double, 2> bds::operator+(std::array<double, 2> v1,
                                     std::array<double, 2> v2)
{
  auto vxf                 = v1[0] + v2[0];
  auto vyf                 = v1[1] + v2[1];
  std::array<double, 2> vf = {vxf, vyf};
  return vf;
}
std::array<double, 2> bds::operator*(std::array<double, 2> v1, double k)
{
  auto vxf                 = k * v1[0];
  auto vyf                 = k * v1[1];
  std::array<double, 2> vf = {vxf, vyf};
  return vf;
}
std::array<double, 2> bds::operator*(std::array<double, 2> v1,std::array<double, 2> v2 )
{
  auto vxf                 = v2[0] * v1[0];
  auto vyf                 = v2[1] * v1[1];
  std::array<double, 2> vf = {vxf, vyf};
  return vf;
}
std::array<double, 2> bds::operator-(std::array<double, 2> v1,
                                     std::array<double, 2> v2)
{
  auto vxf                 = v1[0] - v2[0];
  auto vyf                 = v1[1] - v2[1];
  std::array<double, 2> vf = {vxf, vyf};
  return vf;
}
std::array<double, 2> bds::operator/(std::array<double, 2> v1, double k)
{ assert(k != 0); //meglio un throw?
  auto vxf                 = v1[0] / k;
  auto vyf                 = v1[1] / k;
  std::array<double, 2> vf = {vxf, vyf};
  return vf;
}