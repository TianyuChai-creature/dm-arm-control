#include "dm_openarm/gravity_model.hpp"

#include <utility>

namespace dm_openarm {

GravityModel::GravityModel(CoupledGravityParam coupled)
  : coupled_(std::move(coupled))
{
  if(coupled_.basis.empty())
  {
    throw std::invalid_argument("coupled gravity: basis must not be empty");
  }
  if(coupled_.weights.empty())
  {
    throw std::invalid_argument("coupled gravity: weights must not be empty");
  }
  for(const auto& row : coupled_.weights)
  {
    if(row.size() != coupled_.basis.size())
    {
      throw std::invalid_argument("coupled gravity: weight row size must match basis");
    }
  }
}

double GravityModel::eval_basis(const std::string& name, const std::vector<double>& q_in)
{
  std::vector<double> q = q_in;
  while(q.size() < 5)
  {
    q.push_back(0.0);
  }
  const double q0 = q[0];
  const double q1 = q[1];
  const double q2 = q[2];
  const double q3 = q[3];
  const double q4 = q[4];

  if(name == "one")
  {
    return 1.0;
  }
  if(name == "sin_q0")
  {
    return std::sin(q0);
  }
  if(name == "sin_q1")
  {
    return std::sin(q1);
  }
  if(name == "sin_q2")
  {
    return std::sin(q2);
  }
  if(name == "sin_q3")
  {
    return std::sin(q3);
  }
  if(name == "sin_q4")
  {
    return std::sin(q4);
  }
  if(name == "cos_q0")
  {
    return std::cos(q0);
  }
  if(name == "cos_q1")
  {
    return std::cos(q1);
  }
  if(name == "cos_q2")
  {
    return std::cos(q2);
  }
  if(name == "cos_q3")
  {
    return std::cos(q3);
  }
  if(name == "cos_q4")
  {
    return std::cos(q4);
  }
  if(name == "sin_q3_q4")
  {
    return std::sin(q3 + q4);
  }
  if(name == "cos_q3_q4")
  {
    return std::cos(q3 + q4);
  }
  if(name == "sin_q2_q3_q4")
  {
    return std::sin(q2 + q3 + q4);
  }
  if(name == "cos_q2_q3_q4")
  {
    return std::cos(q2 + q3 + q4);
  }
  throw std::invalid_argument("unknown gravity basis name: " + name);
}

std::vector<double> GravityModel::compute(const std::vector<double>& q) const
{
  if(coupled_.weights.empty())
  {
    throw std::runtime_error("gravity model has no coupled weights configured");
  }
  if(q.size() != coupled_.weights.size())
  {
    throw std::invalid_argument("coupled gravity: q size must match weight rows");
  }

  std::vector<double> phi(coupled_.basis.size(), 0.0);
  for(std::size_t k = 0; k < coupled_.basis.size(); ++k)
  {
    phi[k] = eval_basis(coupled_.basis[k], q);
  }

  std::vector<double> tau(coupled_.weights.size(), 0.0);
  for(std::size_t j = 0; j < coupled_.weights.size(); ++j)
  {
    double sum = 0.0;
    for(std::size_t k = 0; k < phi.size(); ++k)
    {
      sum += coupled_.weights[j][k] * phi[k];
    }
    tau[j] = sum;
  }
  return tau;
}

std::vector<double> GravityModel::compute(const std::vector<double>& q, double scale) const
{
  auto tau = compute(q);
  for(double& t : tau)
  {
    t *= scale;
  }
  return tau;
}

}  // namespace dm_openarm
