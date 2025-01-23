
#ifndef CERES_EXAMPLES_SNAVELY_REPROJECTION_ERROR_H_
#define CERES_EXAMPLES_SNAVELY_REPROJECTION_ERROR_H_

#include "ceres/autodiff_cost_function.h"
#include "ceres/rotation.h"

namespace ceres::examples {

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters:
// 3 for rotation (angle-axis),
// 3 for translation,
// 1 for focal length
// 2 for radial distortion.
// The principal point is not modeled

struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}
  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    AngleAxisRotatePoint(camera, point, p);
    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];
    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    const T xp = - p[0] / p[2];
    const T yp = - p[1] / p[2];
    // Apply second and fourth order radial distortion.
    const T& l1 = camera[7];
    const T& l2 = camera[8];
    const T r2 = xp*xp + yp*yp;
    const T distortion = T(1.0) + r2  * (l1 + l2  * r2);
    // Compute final projected point position.
    const T& focal = camera[6];
    const T predicted_x = focal * distortion * xp;
    const T predicted_y = focal * distortion * yp;
    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

/*
    std::cout<<"ceres predicted x,y:" <<predicted_x<<", "<<predicted_y<<std::endl;
    std::cout<<"opencv observed x,y:" <<observed_x<<", "<<observed_y<<std::endl;
*/

    // Residuals
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;



    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
                new SnavelyReprojectionError(observed_x, observed_y)));
  }
  double observed_x;
  double observed_y;
};




}  // namespace ceres::examples

#endif  // CERES_EXAMPLES_SNAVELY_REPROJECTION_ERROR_H_
