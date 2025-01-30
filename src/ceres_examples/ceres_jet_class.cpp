#include <ceres/autodiff_cost_function.h>
#include <ceres/jet.h>
#include <cmath>
#include <iostream>

template <typename T> T f(const T &a, const T &b) {
  return b * ceres::sin(a) + b * b; // f(a, b) = b * sin(a) + b^2
}

struct CostFunctor {
  template <typename T>
  bool operator()(const T *const a, const T *const b, T *residual) const {
    residual[0] = (*b) * ceres::sin(*a) + (*b) * (*b);
    return true;
  }
};

int main() {
  using Jet = ceres::Jet<double, 2>; // Jet for 2 variables (a, b)

  // Input values
  double a_val = 1.0; // a = 1.0
  double b_val = 2.0; // b = 2.0

  // Initialize Jets
  Jet a(a_val, 0); // a = 1.0, derivative w.r.t. a is 1.0
  Jet b(b_val, 1); // b = 2.0, derivative w.r.t. b is 1.0

  // Evaluate the function
  Jet result = f(a, b);

  // Output results
  std::cout << "f(a, b)       = " << result.a << std::endl;
  std::cout << "df/da(a, b)   = " << result.v[0]
            << std::endl; // Derivative w.r.t. a
  std::cout << "df/db(a, b)   = " << result.v[1]
            << std::endl; // Derivative w.r.t. b

  // Create a Ceres CostFunction with AutoDiff
  ceres::CostFunction *cost_function =
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1>(new CostFunctor());

  // Prepare storage for the residual
  double residuals[1];

  // Create an array of pointers to parameters (Ceres expects this format)
  double *parameters[] = {&a_val, &b_val};
  cost_function->Evaluate(parameters, residuals, nullptr);

  return 0;
}
