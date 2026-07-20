#include "dm_openarm/gravity_model.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

int main()
{
  using dm_openarm::GravityModel;
  using dm_openarm::JointGravityParam;

  GravityModel model({
    JointGravityParam{2.0, 0.0, 0.1},
    JointGravityParam{1.0, 0.5, 0.0},
  });

  const auto tau = model.compute({0.0, 0.0});
  assert(tau.size() == 2);
  assert(std::abs(tau[0] - (2.0 * std::sin(0.0) + 0.1)) < 1e-9);
  assert(std::abs(tau[1] - (1.0 * std::sin(0.5))) < 1e-9);

  const auto scaled = model.compute({1.0, -0.2}, 0.5);
  assert(std::abs(scaled[0] - 0.5 * (2.0 * std::sin(1.0) + 0.1)) < 1e-9);

  try
  {
    model.compute({0.0});
    std::cerr << "expected size mismatch throw\n";
    return 1;
  }
  catch(const std::invalid_argument&)
  {
  }

  std::cout << "test_gravity_model OK\n";
  return 0;
}
