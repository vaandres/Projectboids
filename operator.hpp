#ifndef OPERATOR_HPP
#define OPERATOR_HPP

#include <array>
#include <cassert>
#include <vector>
namespace bds {
std::array<double, 2> operator+(std::array<double, 2>, std::array<double, 2>);
std::array<double, 2> operator*(std::array<double, 2>, double);
std::array<double, 2> operator-(std::array<double, 2>, std::array<double, 2>);
std::array<double, 2> operator/(std::array<double, 2>, double);
} // namespace bds

#endif