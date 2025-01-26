#include "ceres/ceres.h"
#include "glog/logging.h"
#include <cmath>
#include <iostream>
#include <vector>

struct ExponentialResidualPackedParams {
  ExponentialResidualPackedParams(double x, double y) : x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T *const params, T *residual) const {
    residual[0] = T(y_) - exp(params[0] * T(x_) +
                              params[1]); // params[0] = m, params[1] = c
    return true;
  }

private:
  const double x_;
  const double y_;
};

struct ExponentialResidualSeperatedParams {
  ExponentialResidualSeperatedParams(double x, double y) : x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T *const m, const T *const c, T *residual) const {
    residual[0] = T(y_) - exp(m[0] * T(x_) + c[0]);
    return true;
  }

private:
  const double x_;
  const double y_;
};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);

  std::vector<double> x_data{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  std::vector<double> y_data;
  const double true_m = 2.0;
  const double true_c = 1.0;

  for (double x : x_data) {
    y_data.push_back(exp(true_m * x + true_c) +
                     0.1 * ((rand() % 100) / 100.0 - 0.5)); // Add noise
  }

  std::cout << "-------------Evaluate "
               "ExponentialResidualPackedParams-------------\n";
  {
    // Sample data point (x, y)
    double x = 2.0;
    double y = exp(2.0 * x + 1.0); // True value of y for m=2.0, c=1.0

    // Define the residual functor
    auto *cost_function =
        new ceres::AutoDiffCostFunction<ExponentialResidualPackedParams, 1, 2>(
            new ExponentialResidualPackedParams(x, y));

    // Parameters to evaluate the cost function
    double params[2] = {1.5, 0.5}; // Initial guess: m=1.5, c=0.5
    double residuals[1];           // To store residuals
    double *jacobians[1];          // To store Jacobians

    double jacobian_values[2]; // Jacobian storage
    jacobians[0] = jacobian_values;

    // Create a pointer to the parameter block
    const double *params_ptr[] = {params};
    // Evaluate the cost function
    bool success = cost_function->Evaluate(params_ptr, residuals, jacobians);

    if (success) {
      std::cout << "Residual: " << residuals[0] << "\n";
      std::cout << "Jacobian wrt m: " << jacobians[0][0] << "\n";
      std::cout << "Jacobian wrt c: " << jacobians[0][1] << "\n";
    } else {
      std::cout << "Cost function evaluation failed.\n";
    }

    delete cost_function;
  }

  std::cout << "-------------Evaluate "
               "ExponentialResidualSeperatedParams-------------\n";

  {
    // Sample data point (x, y)
    double x = 2.0;
    double y = exp(2.0 * x + 1.0); // True value of y for m=2.0, c=1.0

    // Define the residual functor
    auto *cost_function =
        new ceres::AutoDiffCostFunction<ExponentialResidualSeperatedParams, 1,
                                        1, 1>(
            new ExponentialResidualSeperatedParams(x, y));

    // Parameters to evaluate the cost function
    double m = 1.5;      // Initial guess for m
    double c = 0.5;      // Initial guess for c
    double residuals[1]; // To store residuals

    double *jacobians[2];      // To store Jacobians for both parameter blocks
    double jacobian_m[1];      // Jacobian storage for m
    double jacobian_c[1];      // Jacobian storage for c
    jacobians[0] = jacobian_m; // Assign Jacobian storage for m
    jacobians[1] = jacobian_c; // Assign Jacobian storage for c

    // Evaluate the cost function

    // Create an array of pointers to the parameter blocks
    const double *params[] = {&m, &c};
    bool success = cost_function->Evaluate(params, residuals, jacobians);

    if (success) {
      std::cout << "Residual: " << residuals[0] << "\n";
      std::cout << "Jacobian wrt m: " << jacobian_m[0] << "\n";
      std::cout << "Jacobian wrt c: " << jacobian_c[0] << "\n";
    } else {
      std::cout << "Cost function evaluation failed.\n";
    }

    delete cost_function;
  }

  std::cout << "-------------ExponentialResidualPackedParams-------------\n";

  {
    double params[2] = {0.0, 0.0}; // Initial guess for [m, c]

    ceres::Problem problem;
    for (size_t i = 0; i < x_data.size(); ++i) {
      auto *cost_function =
          new ceres::AutoDiffCostFunction<ExponentialResidualPackedParams, 1,
                                          2>(
              new ExponentialResidualPackedParams(x_data[i], y_data[i]));
      problem.AddResidualBlock(cost_function, nullptr, params);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << "\n";
    std::cout << "Estimated m: " << params[0] << ", c: " << params[1] << "\n";
    std::cout << "True m: " << true_m << ", c: " << true_c << "\n";
  }

  std::cout << "-------------ExponentialResidualSeperatedParams-------------\n";
  {
    double m = 0.0; // Initial guess for m
    double c = 0.0; // Initial guess for c

    ceres::Problem problem;
    for (size_t i = 0; i < x_data.size(); ++i) {
      auto *cost_function =
          new ceres::AutoDiffCostFunction<ExponentialResidualSeperatedParams, 1,
                                          1, 1>(
              new ExponentialResidualSeperatedParams(x_data[i], y_data[i]));
      problem.AddResidualBlock(cost_function, nullptr, &m, &c);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << "\n";
    std::cout << "Estimated m: " << m << ", c: " << c << "\n";
    std::cout << "True m: " << true_m << ", c: " << true_c << "\n";
  }

  return 0;
}
