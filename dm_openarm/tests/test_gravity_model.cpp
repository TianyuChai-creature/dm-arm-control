#include "dm_openarm/gravity_model.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

int main()
{
  using dm_openarm::CoupledGravityParam;
  using dm_openarm::GravityMode;
  using dm_openarm::GravityModel;
  using dm_openarm::JointGravityParam;

  // Decoupled
  {
    GravityModel model({
      JointGravityParam{2.0, 0.0, 0.1},
      JointGravityParam{1.0, 0.5, 0.0},
    });
    const auto tau = model.compute({0.0, 0.0});
    assert(std::abs(tau[0] - 0.1) < 1e-9);
    assert(std::abs(tau[1] - std::sin(0.5)) < 1e-9);
  }

  // Coupled: tau0 = 1.5*sin(q0) + 0.2; tau4 = 2.0*sin(q3+q4)
  {
    CoupledGravityParam cp;
    cp.basis = {
      "one",     "sin_q0",  "sin_q1",  "sin_q2",  "sin_q3",  "sin_q4",
      "cos_q0",  "cos_q1",  "cos_q2",  "cos_q3",  "cos_q4",
      "sin_q3_q4", "cos_q3_q4",
    };
    const std::size_t K = cp.basis.size();
    cp.weights.assign(5, std::vector<double>(K, 0.0));
    cp.weights[0][0] = 0.2;   // one
    cp.weights[0][1] = 1.5;   // sin_q0
    cp.weights[4][11] = 2.0;  // sin_q3_q4

    GravityModel model(GravityMode::Coupled, {}, cp);
    assert(model.mode() == GravityMode::Coupled);
    assert(model.size() == 5);

    const std::vector<double> q = {0.3, 0.0, 0.0, 0.4, 0.5};
    const auto tau = model.compute(q);
    assert(std::abs(tau[0] - (0.2 + 1.5 * std::sin(0.3))) < 1e-9);
    assert(std::abs(tau[1]) < 1e-12);
    assert(std::abs(tau[4] - 2.0 * std::sin(0.4 + 0.5)) < 1e-9);

    const auto scaled = model.compute(q, 0.5);
    assert(std::abs(scaled[0] - 0.5 * tau[0]) < 1e-9);
  }

  std::cout << "test_gravity_model OK\n";
  return 0;
}
