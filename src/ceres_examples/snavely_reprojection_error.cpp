#include "../snavely_reprojection_error.h"
#include "../collection_adapters.hpp"

#include "transformation.hpp"
#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <rerun/archetypes/arrows3d.hpp>

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


                    Z                        Z
                    ▲                         ▲
                   /                           \
                  /                             \
                 /1 2 3 4     X                  \ 1 2 3 4
    Left Cam   |------------ ⯈                   |------------ ⯈Right cam
              1|                               1 |
              2|                               2 |
              3|                               3 |
             Y |                               Y |
               ⯆                                ⯆


  */
  cv::Mat leftCameraRotation_w_c, rightCameraRotation_w_c;
  double rollLeft, pitchLeft, yawLeft, rollRight, pitchRight, yawRight, txLeft,
      tyLeft, tzLeft, txRight, tyRight, tzRight;

  cv::Vec3d thetaLeft, thetaRight;

  rollLeft = 0;
  pitchLeft = +M_PI / 3;
  //  pitchLeft = 60;
  yawLeft = 0;

  thetaLeft[0] = rollLeft;
  thetaLeft[1] = pitchLeft;
  thetaLeft[2] = yawLeft;

  rollRight = 0;
  pitchRight = -M_PI / 6;
  //  pitchRight = 0;
  yawRight = 0;

  thetaRight[0] = rollRight;
  thetaRight[1] = pitchRight;
  thetaRight[2] = yawRight;

  txLeft = -0.5;
  tyLeft = 0.0;
  tzLeft = -4.0;

  txRight = 0.5;
  tyRight = 0.0;
  tzRight = -4.0;

  leftCameraRotation_w_c = eulerAnglesToRotationMatrix(thetaLeft);
  rightCameraRotation_w_c = eulerAnglesToRotationMatrix(thetaRight);

  cv::Mat leftCameraTranslation_w_c =
      (cv::Mat_<double>(3, 1) << txLeft, tyLeft, tzLeft);
  cv::Mat rightCameraTranslation_w_c =
      (cv::Mat_<double>(3, 1) << txRight, tyRight, tzRight);

  std::cout << "///////// ground truth /////////" << std::endl;

  std::cout << "left camera translation in world:\n"
            << leftCameraTranslation_w_c << std::endl;
  std::cout << "left camera roll, pitch , yaw in world:\n"
            << thetaLeft << std::endl;
  std::cout << "right camera translation in world:\n"
            << rightCameraTranslation_w_c << std::endl;

  std::cout << "right camera roll, pitch , yaw in world:\n"
            << thetaRight << std::endl;

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

  //  k1 = 1.6076213815298762e-01;
  //  k2 = -1.2591517698167454e+00;

  k1 = 1e-3;
  k2 = 1e-5;

  p1 = 0;
  p2 = 0;

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

  cv::Mat rightCameraRotation_c_w =
      rightCameraRotation_w_c.t(); // Transpose (or inverse) of R_w_c
  cv::Mat rightCamera_T_c_w =
      -rightCameraRotation_c_w *
      rightCameraTranslation_w_c; // Correct transformation of the translation
                                  // vector

  cv::projectPoints(objectPointsInWorldCoordinate, rightCameraRotation_c_w,
                    rightCamera_T_c_w, K, distCoeffs, imagePointsRightCamera);

  std::cout << "///////// recoverPose /////////" << std::endl;

  cv::Mat R_recoverPose, t_recoverPose;

  // Compute relative pose
  cv::Mat E_recoverPose = cv::findEssentialMat(
      imagePointsLeftCamera, imagePointsRightCamera, K, cv::RANSAC);

  cv::recoverPose(E_recoverPose, imagePointsLeftCamera, imagePointsRightCamera,
                  K, R_recoverPose, t_recoverPose);

  std::cout << "Rotation of left camera expressed in right camera R_r_l:\n"
            << R_recoverPose
            << "\nTranslation of left camera expressed in right camera t_r_l:\n"
            << t_recoverPose << "\nE: " << E_recoverPose << std::endl;

  ///////////////////////////////////////////////
  //  t_recoverPose.at<double>(0, 0) = -2;

  // Compute the projection matrices, the wolrd is the left camera
  // since now the left camera is the world, so the extrinsic are R=identity and
  // t=0

  cv::Mat projectionLeftCamera =
      cv::Mat::eye(3, 4, CV_64F); // [I|0] for the first camera

  //
  cv::Mat projectionRightCamera(3, 4, CV_64F); // [R|t] for the second camera

  // the Rotation of right camera expressed in left camera (world) R_r_l

  // since the world is the left camera and we want to transform point from left
  // camera, R,t from recover pose is all we need for ytriangulation but if we
  // want to calculate the pose of right camera in the left camera we need
  // cv::Mat rotation_of_right_camera_expressed_in_left  = R_recoverPose.t();
  // cv::Mat translation_of_rightcamera_expressed_in_left = -R_recoverPose.t() *
  // t_recoverPose;

  R_recoverPose.copyTo(projectionRightCamera(
      cv::Rect(0, 0, 3, 3))); // Copy R into the top-left 3x3 submatrix

  t_recoverPose.copyTo(projectionRightCamera(
      cv::Rect(3, 0, 1, 3))); // Copy t into the last column

  // Triangulate points
  // imagePointsLeftCamera and imagePointsRightCamera are in pixel, the world is
  // the left camera, triangulated points are in left camera
  cv::Mat points4D_in_left_camera;
  cv::triangulatePoints(K * projectionLeftCamera, K * projectionRightCamera,
                        imagePointsLeftCamera, imagePointsRightCamera,
                        points4D_in_left_camera);

  // Convert points from homogeneous to 3D (divide by w)
  std::vector<cv::Point3f> triangulatedPoints_in_left_camera;
  for (int i = 0; i < points4D_in_left_camera.cols; ++i) {
    cv::Point3f point;
    point.x = points4D_in_left_camera.at<float>(0, i) /
              points4D_in_left_camera.at<float>(3, i);
    point.y = points4D_in_left_camera.at<float>(1, i) /
              points4D_in_left_camera.at<float>(3, i);
    point.z = points4D_in_left_camera.at<float>(2, i) /
              points4D_in_left_camera.at<float>(3, i);
    triangulatedPoints_in_left_camera.push_back(point);
  }

  const auto rec = rerun::RecordingStream("stereo_vision");
  // Log the triangulated points to Rerun
  std::vector<rerun::components::Position3D>
      triangulated3d_positions_in_left_camera;
  for (const auto &point : triangulatedPoints_in_left_camera) {
    triangulated3d_positions_in_left_camera.push_back(
        {point.x, point.y, point.z});
  }
  rec.log("world/triangulatedPoints_in_left_camera",
          rerun::Points3D(triangulated3d_positions_in_left_camera)
              .with_radii(std::vector<float>(
                  triangulatedPoints_in_left_camera.size(), 0.05)));

  double totalError = 0.0;
  size_t numPoints = triangulatedPoints_in_left_camera.size();

  for (size_t i = 0; i < numPoints; ++i) {
    const auto &point = triangulatedPoints_in_left_camera[i];
    double residuals[2];
    double point3D[3] = {point.x, point.y, point.z};

    //    auto obs_x = -(imagePointsLeftCamera[i].x - cx);
    //    auto obs_y = -(imagePointsLeftCamera[i].y - cy);
    //    SnavelyReprojectionError errorModel(obs_x, obs_y);

    double fx = focalLength * mx;
    double fy = focalLength * my;

    // negative sign for Snavely
    double x_sn = -(imagePointsLeftCamera[i].x - cx) / fx;
    double y_sn = -(imagePointsLeftCamera[i].y - cy) / fy;
    SnavelyReprojectionError errorModel(x_sn, y_sn);

    double camera_params[9];
    cv::Mat rvec;
    cv::Mat R_left_camera_in_left_camera = cv::Mat::eye(3, 3, CV_64F);

    cv::Mat t_left_camera_in_left_camera = (cv::Mat_<double>(3, 1) << 0, 0, 0);

    cv::Rodrigues(R_left_camera_in_left_camera, rvec);
    for (int k = 0; k < 3; ++k) {
      camera_params[k] = rvec.at<double>(k);
      camera_params[k + 3] = t_left_camera_in_left_camera.at<double>(k, 0);
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

    cv::Mat objectPoints = cv::Mat(1, 3, CV_64F, point3D).clone();
    cv::Mat reprojectedPoint;
    cv::projectPoints(objectPoints, R_left_camera_in_left_camera,
                      t_left_camera_in_left_camera, K, cv::noArray(),
                      reprojectedPoint);

    totalError += error;

    std::cout << "error: " << error << std::endl;
  }
  auto projectionError = totalError / static_cast<double>(numPoints);

  std::cout << "Mean projection error for left camera: " << projectionError
            << " pixels" << std::endl;

  //  std::cout << "Rotation of left camera expressed in right camera:\n"
  //            << R << "\nTranslation of left camera expressed in right
  //            camera:\n"
  //            << T << "\nE: " << E << "\nF: " << F << "\n Per View Errors:\n"
  //            << perViewErrors << std::endl;

  return;
}

int main() { projectPointcloudInStereoImagePlane(); }
