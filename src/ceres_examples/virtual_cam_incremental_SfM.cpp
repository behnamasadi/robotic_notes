#include "../collection_adapters.hpp"
#include "../snavely_reprojection_error.h"
#include "../transformation.hpp"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <rerun/archetypes/arrows3d.hpp>
#include <rerun/datatypes/rgba32.hpp>

using namespace ceres::examples;
//------------------------------------------------------------------------------
// ) Convert from homogeneous (4 x N) to std::vector<cv::Point3f> (divide by w)
//------------------------------------------------------------------------------
std::vector<cv::Point3f> convertHomogeneous(const cv::Mat &points4D) {
  std::vector<cv::Point3f> points3D;

  points3D.reserve(points4D.cols);

  for (int i = 0; i < points4D.cols; ++i) {
    cv::Point3f point;
    point.x = points4D.at<float>(0, i) / points4D.at<float>(3, i);
    point.y = points4D.at<float>(1, i) / points4D.at<float>(3, i);
    point.z = points4D.at<float>(2, i) / points4D.at<float>(3, i);
    //    std::cout << "triangulatedPoints: " << point << std::endl;
    points3D.push_back(point);
  }
  return points3D;
}

cv::Mat createImage(double focalLength, int numberOfPixelInHeight,
                    int numberOfPixelInWidth,
                    std::vector<cv::Point2f> projectedPoints,
                    std::string fileName) {

  double row, col;

  cv::Mat cameraImage =
      cv::Mat::zeros(numberOfPixelInHeight, numberOfPixelInWidth, CV_8UC1);
  cv::line(cameraImage, cv::Point2d(numberOfPixelInWidth / 2, 0),
           cv::Point2d(numberOfPixelInWidth / 2, numberOfPixelInHeight),
           cv::Scalar(255, 255, 255));
  cv::line(cameraImage, cv::Point2d(0, numberOfPixelInHeight / 2),
           cv::Point2d(numberOfPixelInWidth, numberOfPixelInHeight / 2),
           cv::Scalar(255, 255, 255));
  for (std::size_t i = 0; i < projectedPoints.size(); i++) {
    col = int(projectedPoints.at(i).x);
    row = int(projectedPoints.at(i).y);
    // std::cout << row << "," << col << std::endl;
    if (int(row) < numberOfPixelInHeight && int(col) < numberOfPixelInWidth)
      cameraImage.at<char>(int(row), int(col)) = char(255);
    else {
      std::cout << row << "," << col << "is out of image" << std::endl;
    }
  }

  cv::imwrite(fileName, cameraImage);
  return cameraImage;
}

template <typename T> rerun::Vec3D getRerunTranslationFromCvMat(cv::Mat t) {

  // Extract translation vector
  rerun::Vec3D translation(t.at<T>(0, 0), t.at<T>(1, 0), t.at<T>(2, 0));

  return translation;
}

template <typename T> rerun::Mat3x3 getRerunRotationFromCvMat(cv::Mat R) {

  // Extract rotation matrix as std::array<float, 9> in
  // column-major order
  std::array<float, 9> rotation_data = {
      static_cast<float>(R.at<double>(0, 0)),
      static_cast<float>(R.at<double>(1, 0)),
      static_cast<float>(R.at<double>(2, 0)),

      static_cast<float>(R.at<double>(0, 1)),
      static_cast<float>(R.at<double>(1, 1)),
      static_cast<float>(R.at<double>(2, 1)),

      static_cast<float>(R.at<double>(0, 2)),
      static_cast<float>(R.at<double>(1, 2)),
      static_cast<float>(R.at<double>(2, 2)),
  };

  return rotation_data;
}

template <typename T> void printArray(std::vector<T> array) {
  for (auto element : array)
    std::cout << element << std::endl;
}

template <typename T>
std::vector<T> createEllipsoidInWorldCoordinate(float centerX = 0,
                                                float centerY = 0,
                                                float centerZ = 0) {
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
      X = a * cos(theta) * cos(phi) - centerX;
      Y = b * cos(theta) * sin(phi) - centerY;
      Z = c * sin(theta) - centerZ;
      objectPointsInWorldCoordinate.push_back({X, Y, Z});
    }
  }

  return objectPointsInWorldCoordinate;
}

template <typename T>
cv::Mat matrixFromVectorOfCVPoints(std::vector<T> vertices) {

  // reshape(1)  make Nx3 1-channel matrix out of Nx1 3-channel.
  // t() transpose the Nx3 matrix.

  cv::Mat pointInWorld = cv::Mat(vertices).reshape(1).t();
  std::cout << pointInWorld << std::endl;
  std::cout << "rows: " << pointInWorld.rows << std::endl;
  std::cout << "cols: " << pointInWorld.cols << std::endl;
  std::cout << "channels: " << pointInWorld.channels() << std::endl;
  return pointInWorld;
}

void drawMatchesBetweenTheTwoFrames(cv::Mat image1, cv::Mat image2,
                                    std::vector<cv::KeyPoint> keypoints1,
                                    std::vector<cv::KeyPoint> keypoints2,

                                    const std::vector<cv::DMatch> &matches) {
  cv::Mat matchImage;
  cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, matchImage);

  // Display the match image
  std::string windowTitle = "Matches between images";
  cv::imshow(windowTitle, matchImage);
  // Wait for ESC key to proceed to the next pair
  while (true) {
    int key = cv::waitKey(0);
    if (key == 27) { // ESC key
      cv::destroyWindow(windowTitle);
      break;
    }
  }
}

//------------------------------------------------------------------------------
// ) Utility to convert (R,t) from OpenCV to the 9-parameter format
//    for the Snavely model in Ceres:
//    - camera[0..2]: angle-axis
//    - camera[3..5]: translation
//    - camera[6]: focal
//    - camera[7..8]: k1, k2
//------------------------------------------------------------------------------
void RtToAngleAxisAndTranslate(const cv::Mat &R, const cv::Mat &t,
                               double *camera_params) {
  // 1) Convert rotation matrix R (3x3) to angle-axis (3)
  // We'll store it in camera_params[0..2].
  double rotMat[9];
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      rotMat[r * 3 + c] = R.at<double>(r, c);
    }
  }
  double angleAxis[3];
  ceres::RotationMatrixToAngleAxis<double>(rotMat, angleAxis);
  camera_params[0] = angleAxis[0];
  camera_params[1] = angleAxis[1];
  camera_params[2] = angleAxis[2];

  // 2) translation => camera_params[3..5]

  camera_params[3] = t.at<double>(0, 0);
  camera_params[4] = t.at<double>(1, 0);
  camera_params[5] = t.at<double>(2, 0);
  //  std::cout << "translation" << camera_params[3] << "," <<
  //  camera_params[4]<< "," << camera_params[5] << std::endl;
}

void setCameraIntrinsic(double focal, double k1, double k2,
                        double *camera_params) {

  camera_params[6] = focal; // focal
  camera_params[7] = k1;    // k1
  camera_params[8] = k1;    // k2
}

//------------------------------------------------------------------------------
// ) Simple struct to hold per-camera data in a more readable form (OpenCV
// style)
//    We'll transform them into the 9-parameter "bundle" format for Ceres.
//------------------------------------------------------------------------------
struct CameraExtrinsics {
  cv::Mat R; // 3x3
  cv::Mat t; // 3x1
};

//------------------------------------------------------------------------------
// We'll keep a global list of 3D points (in world coords)
// and for each 3D point, we'll store which camera+feature sees it
// (observations).
//------------------------------------------------------------------------------

struct Observation {
  int camera_idx; // which camera sees this point
  int point_idx;  // which 3D point is observed
  double x, y;    // 2D feature coords
};

void virtualCamIncrementalSfM() {

  double txCam0, txCam1, txCam2, tyCam0, tyCam1, tyCam2, tzCam0, tzCam1, tzCam2;
  cv::Vec3d thetaCam0, thetaCam1, thetaCam2;

  ///////////////// cameras extrinsic /////////////////
  // clang-format off
    /*


                  //-\\
                 // - \\
                /// - \\\
                \\\   ///
                 \\ - //
                  \\-//




                      Z                        Z                            Z
                      ▲                         ▲                           ▲
                     /                           \                           \
                    /                             \                           \
                   /1 2 3 4     X                  \ 1 2 3 4                   \ 1 2 3 4
      (world)Cam0  |------------ ⯈                 |------------ ⯈Cam1         |------------ ⯈Cam2
                  1|                             1 |                          1 |
                  2|                             2 |                          2 |
                  3|                             3 |                          3 |
                 Y |                             Y |                          Y |
                   ⯆                              ⯆                            ⯆


    */
  // clang-format on

  thetaCam0[0] = 0;
  thetaCam0[1] = +M_PI / 12;
  thetaCam0[2] = 0;

  thetaCam1[0] = 0;
  thetaCam1[1] = +M_PI / 18;
  thetaCam1[2] = 0;

  thetaCam2[0] = 0;
  thetaCam2[1] = -M_PI / 24;
  thetaCam2[2] = 0;

  txCam0 = 0;
  tyCam0 = 0.0;
  tzCam0 = 0;

  txCam1 = 0.75;
  tyCam1 = 0.0;
  tzCam1 = 0;

  txCam2 = +1.5;
  tyCam2 = 0.0;
  tzCam2 = 0;

  cv::Mat t_Cam0_in_world = (cv::Mat_<double>(3, 1) << txCam0, tyCam0, tzCam0);

  cv::Mat t_Cam1_in_world = (cv::Mat_<double>(3, 1) << txCam1, tyCam1, tzCam1);

  cv::Mat t_Cam2_in_world = (cv::Mat_<double>(3, 1) << txCam2, tyCam2, tzCam2);

  cv::Mat rotation_Cam0_in_world, rotation_Cam1_in_world,
      rotation_Cam2_in_world;

  rotation_Cam0_in_world = eulerAnglesToRotationMatrix(thetaCam0);
  rotation_Cam1_in_world = eulerAnglesToRotationMatrix(thetaCam1);
  rotation_Cam2_in_world = eulerAnglesToRotationMatrix(thetaCam2);

  ///////// creating ellipsoid in the world coordinate /////////
  std::vector<cv::Vec3f> objectPointsInWorldCoordinate;

  float ellipsoidCenterX = -1.5;
  float ellipsoidCenterY = 0;
  float ellipsoidCenterZ = -4;
  objectPointsInWorldCoordinate = createEllipsoidInWorldCoordinate<cv::Vec3f>(
      ellipsoidCenterX, ellipsoidCenterY, ellipsoidCenterZ);

  ///////// camera intrinsic parameters/////////

  //  4, 5, 8, 12 or 14 elements
  double k1, k2, p1, p2, k3;

  k1 = 0.;
  k2 = 0.;
  p1 = 0.;
  p2 = 0.;
  k3 = 0.;

  cv::Mat distortionCoefficient =
      (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

  unsigned int numberOfPixelInHeight, numberOfPixelInWidth;
  double heightOfSensor, widthOfSensor;
  double focalLength = 4.0;
  double mx, my, U0, V0;
  numberOfPixelInHeight = 600;
  numberOfPixelInWidth = 600;

  heightOfSensor = 10;
  widthOfSensor = 10;

  my = (numberOfPixelInHeight) / heightOfSensor;

  mx = (numberOfPixelInWidth) / widthOfSensor;

  double fx = focalLength * mx;
  double fy = focalLength * my;

  double cx, cy;

  cx = (numberOfPixelInWidth) / 2;
  cy = (numberOfPixelInHeight) / 2;

  cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

  std::vector<cv::Point2f> imagePointsCam0, imagePointsCam1, imagePointsCam2;

  cv::Mat rotation_world_in_Cam0 = rotation_Cam0_in_world.t();
  cv::Mat t_world_in_Cam0 = -rotation_Cam0_in_world.t() * t_Cam0_in_world;

  cv::projectPoints(objectPointsInWorldCoordinate, rotation_world_in_Cam0,
                    t_world_in_Cam0, K, cv::noArray(), imagePointsCam0);

  cv::Mat rotation_world_in_Cam1 = rotation_Cam1_in_world.t();
  cv::Mat t_world_in_Cam1 = -rotation_Cam1_in_world.t() * t_Cam1_in_world;

  cv::projectPoints(objectPointsInWorldCoordinate, rotation_world_in_Cam1,
                    t_world_in_Cam1, K, cv::noArray(), imagePointsCam1);

  cv::Mat rotation_world_in_Cam2 = rotation_Cam2_in_world.t();
  cv::Mat t_world_in_Cam2 = -rotation_Cam2_in_world.t() * t_Cam2_in_world;

  cv::projectPoints(objectPointsInWorldCoordinate, rotation_world_in_Cam2,
                    t_world_in_Cam2, K, cv::noArray(), imagePointsCam2);

  const auto rec = rerun::RecordingStream("virtual_cam_incremental_SfM");
  rec.spawn().exit_on_failure();
  // OpenCV X=Right, Y=Down, Z=Forward
  rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);

  std::vector<rerun::components::Position3D> point3d_positions;
  std::vector<float> point_sizes; // Define a vector for point sizes

  // Log the arrows to the Rerun Viewer
  rec.log("world/xyz",
          rerun::Arrows3D::from_vectors(
              {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}})
              .with_colors({{255, 0, 0}, {0, 255, 0}, {0, 0, 255}}));

  // objectPointsInCameraCoordinate;
  float x, y, z;

  for (std::size_t i = 0; i < objectPointsInWorldCoordinate.size(); i++) {
    x = objectPointsInWorldCoordinate[i][0];
    y = objectPointsInWorldCoordinate[i][1];
    z = objectPointsInWorldCoordinate[i][2];
    point3d_positions.push_back({x, y, z});
    point_sizes.push_back(0.05);
  }

  rec.log("world/points",
          rerun::Points3D(point3d_positions).with_radii(point_sizes));

  // Extract translation vector
  rerun::Vec3D rr_translation_cam0 =
      getRerunTranslationFromCvMat<double>(t_Cam0_in_world);
  rerun::Vec3D rr_translation_cam1 =
      getRerunTranslationFromCvMat<double>(t_Cam1_in_world);
  rerun::Vec3D rr_translation_cam2 =
      getRerunTranslationFromCvMat<double>(t_Cam2_in_world);

  rerun::Mat3x3 rr_rotation_matrix_cam0 =
      getRerunRotationFromCvMat<double>(rotation_Cam0_in_world);
  rerun::Mat3x3 rr_rotation_matrix_cam1 =
      getRerunRotationFromCvMat<double>(rotation_Cam1_in_world);
  rerun::Mat3x3 rr_rotation_matrix_cam2 =
      getRerunRotationFromCvMat<double>(rotation_Cam2_in_world);

  // Log the data
  std::string camera_name_cam0 = "world/cam0";
  std::string camera_name_cam1 = "world/cam1";
  std::string camera_name_cam2 = "world/cam2";

  rec.log(camera_name_cam0,
          rerun::Pinhole::from_focal_length_and_resolution(
              {float(focalLength * mx), float(focalLength * my)},
              {float(numberOfPixelInWidth), float(numberOfPixelInHeight)}));

  rec.log(camera_name_cam1,
          rerun::Pinhole::from_focal_length_and_resolution(
              {float(focalLength * mx), float(focalLength * my)},
              {float(numberOfPixelInWidth), float(numberOfPixelInHeight)}));

  rec.log(camera_name_cam2,
          rerun::Pinhole::from_focal_length_and_resolution(
              {float(focalLength * mx), float(focalLength * my)},
              {float(numberOfPixelInWidth), float(numberOfPixelInHeight)}));

  rec.log(camera_name_cam0,
          rerun::Transform3D(rr_translation_cam0, rr_rotation_matrix_cam0));

  rec.log(camera_name_cam1,
          rerun::Transform3D(rr_translation_cam1, rr_rotation_matrix_cam1));

  rec.log(camera_name_cam2,
          rerun::Transform3D(rr_translation_cam2, rr_rotation_matrix_cam2));

  ////////////////////////////////////////////////////////////////////////

  std::string fileName;
  fileName = std::string("image_cam0") + std::to_string(focalLength) +
             std::string("_.png");
  cv::Mat img_cam0 =
      createImage(focalLength, numberOfPixelInHeight, numberOfPixelInWidth,
                  imagePointsCam0, fileName);
  // Log the image to the camera entity in the hierarchy
  rec.log("world/cam0/image/rgb",
          rerun::Image::from_greyscale8(
              img_cam0, {numberOfPixelInWidth, numberOfPixelInHeight}));

  fileName = std::string("image_cam1") + std::to_string(focalLength) +
             std::string("_.png");
  cv::Mat img_cam1 =
      createImage(focalLength, numberOfPixelInHeight, numberOfPixelInWidth,
                  imagePointsCam1, fileName);
  // Log the image to the camera entity in the hierarchy
  rec.log("world/cam1/image/rgb",
          rerun::Image::from_greyscale8(
              img_cam1, {numberOfPixelInWidth, numberOfPixelInHeight}));

  fileName = std::string("image_cam2") + std::to_string(focalLength) +
             std::string("_.png");
  cv::Mat img_cam2 =
      createImage(focalLength, numberOfPixelInHeight, numberOfPixelInWidth,
                  imagePointsCam2, fileName);
  // Log the image to the camera entity in the hierarchy
  rec.log("world/cam2/image/rgb",
          rerun::Image::from_greyscale8(
              img_cam2, {numberOfPixelInWidth, numberOfPixelInHeight}));

  std::cout << "press any keys to draw correspondance " << std::endl;
  std::cin.get();

  int N_CAMERAS = 3;

  std::vector<std::vector<cv::KeyPoint>> keypoints(N_CAMERAS);
  std::vector<std::vector<cv::DMatch>> all_matches;

  // Size of the keypoint is set to 1.0f
  for (const auto &point : imagePointsCam0) {
    keypoints[0].emplace_back(point, 1.0f);
  }

  for (const auto &point : imagePointsCam1) {
    keypoints[1].emplace_back(point, 1.0f);
  }

  for (const auto &point : imagePointsCam2) {
    keypoints[2].emplace_back(point, 1.0f);
  }

  std::vector<cv::Mat> images;
  images.push_back(img_cam0);
  images.push_back(img_cam1);
  images.push_back(img_cam2);

  for (size_t i = 0; i < N_CAMERAS - 1; ++i) {

    std::vector<cv::DMatch> matches;
    // Generate matches based on point order, assumes all cameras have the same
    // number of points
    size_t num_points = keypoints[i].size();
    for (size_t j = 0; j < num_points; ++j) {
      // Match index j to j with a distance of 0.0
      matches.emplace_back(cv::DMatch(j, j, 0.0f));
      //      std::cout << "j:" << j << std::endl;
    }

    drawMatchesBetweenTheTwoFrames(images[i], images[i + 1], keypoints[i],
                                   keypoints[i + 1], matches);

    all_matches.push_back(matches);
  }

  // camera0  is the world,we set identity and zero
  std::vector<CameraExtrinsics> cameras(N_CAMERAS);
  cameras[0].R = cv::Mat::eye(3, 3, CV_64F);
  cameras[0].t = cv::Mat::zeros(3, 1, CV_64F);

  std::vector<double> cameraParams(9 * N_CAMERAS, 0.0);

  // Initialize camera0 in the bundle with R=I, t=0 => angle-axis=0, ...
  RtToAngleAxisAndTranslate(cameras[0].R, cameras[0].t, &cameraParams[0 * 9]);
  //  setCameraIntrinsic(fx, k1, k2, &cameraParams[0 * 9]);

  setCameraIntrinsic(1.0, k1, k2, &cameraParams[0 * 9]);

  // All 3D points in world coords
  std::vector<cv::Point3f> globalPoints3D;

  // All 2D observations of these 3D points, camera_idx, 3d point_idx, x, y
  std::vector<Observation> observations;

  for (size_t i = 1; i < N_CAMERAS; ++i) {

    // Log the triangulated points to Rerun
    std::vector<rerun::components::Position3D> rr_triangulated_pointsInCam0;

    std::cout << "i is : " << i << ", prcessing camera: " << i - 1
              << " and camera: " << i << std::endl;

    std::vector<cv::Point2f> pts_im1, pts_i;

    for (const auto &match : all_matches[i - 1]) {
      pts_im1.push_back(keypoints[i - 1][match.queryIdx].pt);
      pts_i.push_back(keypoints[i][match.trainIdx].pt);
    }

    cv::Mat mask;
    cv::Mat E =
        cv::findEssentialMat(pts_im1, pts_i, K, cv::RANSAC, 0.999, 1.0, mask);

    cv::Mat R_im1_to_i, t_im1_to_i;
    cv::recoverPose(E, pts_im1, pts_i, K, R_im1_to_i, t_im1_to_i, mask);

    //--------------------------------------------------------------------------
    // 9) Now we want camera i has the transformation to transform points in 3d
    // camera1 is the world, we know camera [i-1] has extrinsics (R_1_to_im1,
    // t_1_to_im1). So: R_{1->i} = R_{1->(i-1)} * R_{(i-1)->i} t_{1->i} =
    // R_{1->(i-1)} * t_{(i-1)->i} + t_{1->(i-1)}
    //--------------------------------------------------------------------------

    cv::Mat R_0_to_im1 = cameras[i - 1].R;
    cv::Mat t_0_to_im1 = cameras[i - 1].t;

    cv::Mat R_0_to_i = R_0_to_im1 * R_im1_to_i;
    cv::Mat t_0_to_i = R_0_to_im1 * t_im1_to_i + t_0_to_im1;

    cameras[i].R = R_0_to_i.clone();
    cameras[i].t = t_0_to_i.clone();

    RtToAngleAxisAndTranslate(cameras[i].R, cameras[i].t, &cameraParams[i * 9]);
    //    setCameraIntrinsic(fx, k1, k2, &cameraParams[i * 9]);
    setCameraIntrinsic(1.0, k1, k2, &cameraParams[i * 9]);

    cv::Mat Rt_im1(3, 4, CV_64F);
    cameras[i - 1].R.copyTo(Rt_im1(cv::Rect(0, 0, 3, 3)));
    cameras[i - 1].t.copyTo(Rt_im1(cv::Rect(3, 0, 1, 3)));
    cv::Mat P_im1 = K * Rt_im1;

    cv::Mat Rt_i(3, 4, CV_64F);
    cameras[i].R.copyTo(Rt_i(cv::Rect(0, 0, 3, 3)));
    cameras[i].t.copyTo(Rt_i(cv::Rect(3, 0, 1, 3)));
    cv::Mat P_i = K * Rt_i;

    cv::Mat points4D;

    cv::triangulatePoints(P_im1, P_i, pts_im1, pts_i, points4D);
    std::vector<cv::Point3f> newPoints_in_cam0 = convertHomogeneous(points4D);

    cv::Mat R_im1_to_0 = cameras[i - 1].R.t();
    cv::Mat t_im1_to_0 = -cameras[i - 1].R.t() * cameras[i - 1].t;

    std::cout << "Rotation from cam" << i - 1 << " to cam0 cam_0_" << i - 1
              << "\n"
              << R_im1_to_0 << std::endl;
    std::cout << "Translation from cam" << i - 1 << " to cam0\n"
              << t_im1_to_0 << std::endl;

    for (size_t k = 0; k < newPoints_in_cam0.size(); k++) {

      int newPointIdx = static_cast<int>(globalPoints3D.size());
      globalPoints3D.push_back(newPoints_in_cam0[k]);

      {
        Observation obs;
        obs.camera_idx = i - 1;
        obs.point_idx = newPointIdx;

        // negative sign for Snavely
        obs.x = -(pts_im1[k].x - cx) / fx;
        obs.y = -(pts_im1[k].y - cy) / fy;

        observations.push_back(obs);
      }
      // Observation from camera (i):
      {
        Observation obs;
        obs.camera_idx = i;
        obs.point_idx = newPointIdx;
        // negative sign for Snavely
        obs.x = -(pts_i[k].x - cx) / fx;
        obs.y = -(pts_i[k].y - cy) / fy;
        observations.push_back(obs);
      }

      rr_triangulated_pointsInCam0.push_back({newPoints_in_cam0[k].x,
                                              newPoints_in_cam0[k].y,
                                              newPoints_in_cam0[k].z});
    }

    std::vector<rerun::Color> colors(rr_triangulated_pointsInCam0.size());
    for (size_t j = 0; j < rr_triangulated_pointsInCam0.size(); ++j) {

      if ((i % 2) == 0) {
        colors[j] = rerun::Color(255, 0, 0); // Red
      } else {
        colors[j] = rerun::Color(0, 255, 0); // Green, for example
      }
    }

    rec.log("world/triangulated_pointsCam" + std::to_string(i) + "InCam0",
            rerun::Points3D(rr_triangulated_pointsInCam0)
                .with_radii(std::vector<float>(
                    rr_triangulated_pointsInCam0.size(), 0.05))
                .with_colors(colors));

    std::cout << "press any key to publish new 3d points " << std::endl;
    std::cin.get();
  }

  std::vector<double> pointParams;
  pointParams.resize(3 * globalPoints3D.size());
  // Copy from cv::Point3f to the double vector
  for (size_t i = 0; i < globalPoints3D.size(); i++) {
    pointParams[3 * i + 0] = globalPoints3D[i].x;
    pointParams[3 * i + 1] = globalPoints3D[i].y;
    pointParams[3 * i + 2] = globalPoints3D[i].z;
  }

  ceres::Problem problem;

  std::cout << "observations.size(): " << observations.size() << std::endl;

  for (const Observation &obs : observations) {

    ceres::CostFunction *costFunc =
        SnavelyReprojectionError::Create(obs.x, obs.y);

    double *cameraParamsPtr = &cameraParams[obs.camera_idx * 9];
    double *pointPtr = &pointParams[3 * obs.point_idx];
    // Create the reprojection error cost:

    // Add a residual block:
    problem.AddResidualBlock(costFunc,
                             nullptr, // squared loss
                             cameraParamsPtr, pointPtr);
  }

  std::cout << "===================Camera Param before "
               "optimization:===================\n";

  for (std::size_t i = 0; i < N_CAMERAS; i++) {

    std::cout << "Camera " << i << " initial parameters:\n";
    std::cout << "Angle-axis = (" << cameraParams[i * 9] << ", "
              << cameraParams[i * 9 + 1] << ", " << cameraParams[i * 9 + 2]
              << ")\n";
    std::cout << "Translation = (" << cameraParams[i * 9 + 3] << ", "
              << cameraParams[i * 9 + 4] << ", " << cameraParams[i * 9 + 5]
              << ")\n";
    std::cout << "Focal length = " << cameraParams[i * 9 + 6] << "\n";
    std::cout << "k1 = " << cameraParams[i * 9 + 7]
              << ", k2 = " << cameraParams[i * 9 + 8] << "\n";
  }

  // fix the first camera:
  problem.SetParameterBlockConstant(&cameraParams[0]);

  // Configure solver:
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;

  // Solve BA:
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);
  std::cout << "Bundle Adjustment Summary:\n"
            << summary.FullReport() << std::endl;

  std::cout << "===================Camera Param After "
               "optimization:===================\n";

  for (std::size_t i = 0; i < N_CAMERAS; i++) {

    std::cout << "Camera " << i << " initial parameters:\n";
    std::cout << "Angle-axis = (" << cameraParams[i * 9] << ", "
              << cameraParams[i * 9 + 1] << ", " << cameraParams[i * 9 + 2]
              << ")\n";
    std::cout << "Translation = (" << cameraParams[i * 9 + 3] << ", "
              << cameraParams[i * 9 + 4] << ", " << cameraParams[i * 9 + 5]
              << ")\n";
    std::cout << "Focal length = " << cameraParams[i * 9 + 6] << "\n";
    std::cout << "k1 = " << cameraParams[i * 9 + 7]
              << ", k2 = " << cameraParams[i * 9 + 8] << "\n";
  }
}

int main() { virtualCamIncrementalSfM(); }
