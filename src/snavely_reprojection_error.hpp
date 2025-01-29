
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
    ceres::AngleAxisRotatePoint(camera, point, p);
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


    /**/
    std::cout<<"observed_x, observed_y: "  <<  observed_x <<"," <<observed_y <<std::endl;
    std::cout<<"predicted_x, predicted_y: "  << predicted_x <<"," <<predicted_y <<std::endl;


    std::cin.get();
    return true;
  }

  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
                new SnavelyReprojectionError(observed_x, observed_y)));
  }
  double observed_x;
  double observed_y;
};



// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters:
// 3 for rotation (angle-axis),
// 3 for translation,


struct SnavelyReprojectionErrorFixedCamera {
  SnavelyReprojectionErrorFixedCamera(double observed_x, double observed_y,
                                      double fixed_focal, double fixed_l1, double fixed_l2, int block_id)
      : m_observed_x(observed_x), m_observed_y(observed_y),
        m_focal(fixed_focal), m_l1(fixed_l1), m_l2(fixed_l2),m_block_id(block_id) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {

      std::cout << "\nInside operator() for block " << m_block_id << ":\n";

//      std::cout << "camera: ";
//        for (int j = 0; j < 6; j++) {
//            std::cout << camera[j] << " ";
//        }
//        std::cout << std::endl;

        std::cout << "point: " << point[0] << " " << point[1] << " " << point[2] << std::endl;


    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion.
    const T xp = -p[0] / p[2];
    const T yp = -p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T r2 = xp * xp + yp * yp;
    const T distortion = T(1.0) + r2 * (T(m_l1) + T(m_l2) * r2);

    // Compute final projected point position.
    const T predicted_x = T(m_focal) * distortion * xp;
    const T predicted_y = T(m_focal) * distortion * yp;


    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(m_observed_x);
    residuals[1] = predicted_y - T(m_observed_y);
    /**/
    std::cout<<"predicted:"<< predicted_x<<","<<predicted_y <<std::endl;
    std::cout<<"observed:"<< m_observed_x<<","<<m_observed_y<<  std::endl;
    std::cin.get();

    return true;
  }

  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     const double fixed_focal,
                                     const double fixed_l1,
                                     const double fixed_l2,
                                     const int block_id)
  {


    //  2 dim of residual
    // 6 dim of camera parameters (3 rotation + 3 translation)
    // 3 dims of 3 point
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionErrorFixedCamera, 2, 6, 3>(
                new SnavelyReprojectionErrorFixedCamera(observed_x, observed_y,
                                                        fixed_focal, fixed_l1, fixed_l2,block_id)));
  }

  double m_observed_x;
  double m_observed_y;
  double m_focal;
  double m_l1;
  double m_l2;
  int    m_block_id;

};




}  // namespace ceres::examples

#endif  // CERES_EXAMPLES_SNAVELY_REPROJECTION_ERROR_H_
