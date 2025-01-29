#include "../snavely_reprojection_error.hpp"
#include "transformation.hpp"
#include <opencv2/opencv.hpp>

using namespace ceres::examples;

template <typename T> std::vector<T> createEllipsoidInWorldCoordinate() {
  std::vector<T> objectPointsInWorldCoordinate;

  float X, Y, Z;

  float phiStepSize, thetaStepSize;
  phiStepSize = 0.2;
  thetaStepSize = 0.1;
  float a, b, c;
  a = 2;
  b = 3;
  c = 1.6;
  for (float phi = -M_PI; phi < M_PI; phi = phi + phiStepSize) {
    for (float theta = -M_PI / 2; theta < M_PI / 2;
         theta = theta + thetaStepSize) {
      X = a * cos(theta) * cos(phi);
      Y = b * cos(theta) * sin(phi);
      Z = c * sin(theta);
      objectPointsInWorldCoordinate.push_back({X, Y, Z});
    }
  }

  return objectPointsInWorldCoordinate;
}

void projectPointcloudInStereoImagePlane() {

  ///////////////// cameras extrinsic /////////////////
  /*
                                          Z
                                          ▲
                                         /
                                        /
                                       /1 2 3 4     X
                          world      |------------ ⯈
                                    1|
                                    2|
                                    3|
                                   Y |
                                     ⯆


                    Z
                    ▲
                   /
                  /
                 /1 2 3 4     X
    Left Cam   |------------ ⯈
              1|
              2|
              3|
             Y |
               ⯆


  */
  cv::Mat leftCameraRotation_w_c;
  double rollLeft, pitchLeft, yawLeft, txLeft, tyLeft, tzLeft;

  cv::Vec3d thetaLeft;

  rollLeft = 0;
  pitchLeft = +M_PI / 3;
  yawLeft = 0;

  thetaLeft[0] = rollLeft;
  thetaLeft[1] = pitchLeft;
  thetaLeft[2] = yawLeft;

  txLeft = -1;
  tyLeft = 0.0;
  tzLeft = -4.0;

  leftCameraRotation_w_c = eulerAnglesToRotationMatrix(thetaLeft);

  cv::Mat leftCameraTranslation_w_c =
      (cv::Mat_<double>(3, 1) << txLeft, tyLeft, tzLeft);

  std::cout << "///////// ground truth /////////" << std::endl;

  std::cout << "left camera translation in world:\n"
            << leftCameraTranslation_w_c << std::endl;
  std::cout << "left camera roll, pitch , yaw in world:\n"
            << thetaLeft << std::endl;

  ///////// creating ellipsoid in the world coordinate /////////
  std::vector<cv::Vec3f> objectPointsInWorldCoordinate;

  objectPointsInWorldCoordinate = createEllipsoidInWorldCoordinate<cv::Vec3f>();

  ///////// camera intrinsic parameters/////////

  unsigned int numberOfPixelInHeight, numberOfPixelInWidth;
  double heightOfSensor, widthOfSensor;
  double focalLength = 4.0;
  double mx, my, cx, cy;
  numberOfPixelInHeight = 600;
  numberOfPixelInWidth = 600;

  heightOfSensor = 10;
  widthOfSensor = 10;

  my = (numberOfPixelInHeight) / heightOfSensor;
  cx = (numberOfPixelInHeight) / 2;

  mx = (numberOfPixelInWidth) / widthOfSensor;
  cy = (numberOfPixelInWidth) / 2;

  double k1, k2, p1, p2;

  k1 = 1e-3;
  k2 = 1e-5;

  p1 = 0;
  p2 = 0;

  double fx = focalLength * mx;
  double fy = focalLength * my;

  cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << k1, k2, p1, p2);

  cv::Mat K = (cv::Mat_<double>(3, 3) << focalLength * mx, 0, cx, 0,
               focalLength * my, cy, 0, 0, 1);
  std::vector<cv::Point2f> imagePointsLeftCamera, imagePointsRightCamera;

  cv::Mat leftCameraRotation_c_w =
      leftCameraRotation_w_c.t(); // Transpose (or inverse) of R_w_c
  cv::Mat leftCamera_T_c_w =
      -leftCameraRotation_c_w *
      leftCameraTranslation_w_c; // Correct transformation of the translation
                                 // vector

  cv::projectPoints(objectPointsInWorldCoordinate, leftCameraRotation_c_w,
                    leftCamera_T_c_w, K, distCoeffs, imagePointsLeftCamera);

  double totalError = 0.0;

  for (size_t i = 0; i < objectPointsInWorldCoordinate.size(); ++i) {
    const auto &point = objectPointsInWorldCoordinate[i];
    double residuals[2];
    double point3D[3] = {point[0], point[1], point[2]};

    // negative sign for Snavely
    double x_sn = -(imagePointsLeftCamera[i].x - cx) / fx;
    double y_sn = -(imagePointsLeftCamera[i].y - cy) / fy;
    SnavelyReprojectionError errorModel(x_sn, y_sn);

    double camera_params[9];
    cv::Mat rvec;
    cv::Mat R_left_camera_in_left_camera = cv::Mat::eye(3, 3, CV_64F);

    cv::Mat t_left_camera_in_left_camera = (cv::Mat_<double>(3, 1) << 0, 0, 0);

    cv::Rodrigues(leftCameraRotation_c_w, rvec);
    for (int k = 0; k < 3; ++k) {
      camera_params[k] = rvec.at<double>(k);
      camera_params[k + 3] = leftCamera_T_c_w.at<double>(k, 0);
    }

    //    camera_params[6] = focalLength * mx;
    camera_params[6] = 1.0;
    camera_params[7] = k1;
    camera_params[8] = k2;

    // Evaluate the residual using the error model
    errorModel(camera_params, point3D, residuals);

    // Compute the squared Euclidean distance (reprojection error for this
    // point)
    double error =
        std::sqrt(residuals[0] * residuals[0] + residuals[1] * residuals[1]);

    totalError += error;

    std::cout << "error: " << error << std::endl;
  }
  auto projectionError =
      totalError / static_cast<double>(objectPointsInWorldCoordinate.size());

  std::cout << "Mean projection error for left camera: " << projectionError
            << " pixels" << std::endl;

  return;
}

int main() { projectPointcloudInStereoImagePlane(); }
