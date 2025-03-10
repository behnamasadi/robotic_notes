#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <iostream>
#include <string>
#include <vector>
using namespace ceres;

struct CostFunctor {

  template <typename T> bool operator()(const T *const x, T *residual) const {

    residual[0] = x[0] * x[0] - T(4.0); // r(x) = x^2 - 4

    return true;
  }
};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);

  // The variable to solve for with its initial value.
  double initial_x = 5.0;
  double x = initial_x;

  // Build the problem.
  Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  CostFunction *cost_function =
      new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor());

  // Add residual block.
  problem.AddResidualBlock(cost_function, nullptr, &x);

  // Run the solver!
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x << " -> " << x << "\n";

  return 0;
}
