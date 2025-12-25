#include "../collection_adapters.hpp"
#include "../snavely_reprojection_error.hpp"
#include <algorithm>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <regex>
#include <rerun.hpp>
#include <rerun/archetypes/arrows3d.hpp>
#include <string>
#include <thread>
#include <vector>

using namespace ceres::examples;

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

//------------------------------------------------------------------------------
// 1) Function to extract numeric parts of file names for sorting:
//------------------------------------------------------------------------------

int extractNumber(const std::string &filename) {
  std::regex numberRegex("(\\d+)");
  std::smatch match;
  if (std::regex_search(filename, match, numberRegex)) {
    return std::stoi(match.str(0));
  }
  return 0; // Default if no number found
}

std::vector<std::string> loadImagePaths(const std::string &directoryPath) {
  std::vector<std::string> imagePaths;
  std::vector<std::string> validExtensions = {".jpg", ".jpeg", ".png", ".bmp",
                                              ".tiff"};

  try {
    for (const auto &entry :
         std::filesystem::directory_iterator(directoryPath)) {
      if (entry.is_regular_file()) {
        std::string extension = entry.path().extension().string();
        if (std::find(validExtensions.begin(), validExtensions.end(),
                      extension) != validExtensions.end()) {
          imagePaths.push_back(entry.path().string());
        }
      }
    }

    // Sort the image paths in numerical order
    std::sort(imagePaths.begin(), imagePaths.end(),
              [](const std::string &a, const std::string &b) {
                return extractNumber(std::filesystem::path(a).stem().string()) <
                       extractNumber(std::filesystem::path(b).stem().string());
              });
  } catch (const std::exception &e) {
    std::cerr << "Error reading directory: " << e.what() << std::endl;
  }

  return imagePaths;
}

//------------------------------------------------------------------------------
// ) Match descriptors between consecutive images
//------------------------------------------------------------------------------

void matchDescriptorsBetweenConsecutiveImages(
    const std::vector<cv::Mat> &descriptors,
    const std::vector<std::vector<cv::KeyPoint>> &keypoints,
    std::vector<std::vector<cv::DMatch>> &all_matches,
    std::vector<cv::DMatch> &matches, int i, int quantiles = 4) {
  cv::BFMatcher matcher(cv::NORM_L2);
  matcher.match(descriptors[i], descriptors[i + 1], matches);

  // Filter matches by median distance
  std::vector<float> distances;
  for (const auto &match : matches) {
    distances.push_back(match.distance);
  }
  std::nth_element(distances.begin(),
                   distances.begin() + distances.size() / quantiles,
                   distances.end());
  float median_distance = distances[distances.size() / quantiles];

  matches.erase(std::remove_if(matches.begin(), matches.end(),
                               [median_distance](const cv::DMatch &match) {
                                 return match.distance > median_distance;
                               }),
                matches.end());

  // Prepare points for RANSAC
  std::vector<cv::Point2f> points1, points2;
  for (const auto &match : matches) {

    points1.push_back(keypoints[i][match.queryIdx].pt);
    points2.push_back(keypoints[i + 1][match.trainIdx].pt);
  }

  // Apply RANSAC to find the fundamental matrix and filter inliers
  std::vector<uchar> inliersMask;
  if (points1.size() >= 8 &&
      points2.size() >= 8) { // Minimum points for fundamental matrix estimation
    cv::Mat fundamentalMatrix = cv::findFundamentalMat(
        points1, points2, cv::FM_RANSAC, 3.0, 0.99, inliersMask);

    // Filter matches based on RANSAC inliers
    std::vector<cv::DMatch> inlier_matches;
    for (size_t j = 0; j < matches.size(); ++j) {
      if (inliersMask[j]) {
        inlier_matches.push_back(matches[j]);
      }
    }
    matches = inlier_matches; // Update matches to include only inliers
  }

  all_matches.push_back(matches);
}

//------------------------------------------------------------------------------
// ) Draw matches between the two frames
//------------------------------------------------------------------------------

void drawMatchesBetweenTheTwoFrames(
    const std::vector<cv::Mat> &images,
    const std::vector<std::vector<cv::KeyPoint>> &keypoints,
    const std::vector<std::string> &image_paths,
    const std::vector<cv::DMatch> &matches, int i) {
  cv::Mat matchImage;
  cv::drawMatches(images[i], keypoints[i], images[i + 1], keypoints[i + 1],
                  matches, matchImage);

  // Display the match image
  std::string windowTitle =
      "Matches between " + image_paths[i] + " and " + image_paths[i + 1];
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
// ) Invert a rotation-translation transform. If X2 = R*X1 + t, then
//    X1 = R^T * (X2 - t). => R_inv = R^T, t_inv = -R^T*t
//------------------------------------------------------------------------------
void invertRt(const cv::Mat &R, const cv::Mat &t, cv::Mat &Rinv,
              cv::Mat &tinv) {
  Rinv = R.t();
  tinv = -Rinv * t;
}

//------------------------------------------------------------------------------
// ) Transform 3D points from camera2 coords => camera1 coords
//    using X1 = R_2to1 * X2 + t_2to1
//------------------------------------------------------------------------------
std::vector<cv::Point3f>
transformPoints(const std::vector<cv::Point3f> &pts_in_C2,
                const cv::Mat &R_2to1, const cv::Mat &t_2to1) {
  std::vector<cv::Point3f> pts_in_C1;
  pts_in_C1.reserve(pts_in_C2.size());

  for (auto &pt2 : pts_in_C2) {
    cv::Mat X2 = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, pt2.z);
    cv::Mat X1 = R_2to1 * X2 + t_2to1; // 3x1
    pts_in_C1.emplace_back(static_cast<float>(X1.at<double>(0, 0)),
                           static_cast<float>(X1.at<double>(1, 0)),
                           static_cast<float>(X1.at<double>(2, 0)));
  }
  return pts_in_C1;
}

//------------------------------------------------------------------------------
// ) Convert from homogeneous (4 x N) to std::vector<cv::Point3f> (divide by w)
//------------------------------------------------------------------------------
std::vector<cv::Point3f> convertHomogeneous(const cv::Mat &points4D) {
  std::vector<cv::Point3f> points3D;
  points3D.reserve(points4D.cols);
  for (int i = 0; i < points4D.cols; i++) {
    float w = static_cast<float>(points4D.at<double>(3, i));

    float x = static_cast<float>(points4D.at<double>(0, i) / w);
    float y = static_cast<float>(points4D.at<double>(1, i) / w);
    float z = static_cast<float>(points4D.at<double>(2, i) / w);

    //    std::cout << "w,x,y,z: " << points4D.at<double>(3, i) << ","
    //              << points4D.at<double>(0, i) << "," <<
    //              points4D.at<double>(1, i)
    //              << "," << points4D.at<double>(2, i) << std::endl;

    points3D.emplace_back(x, y, z);
  }
  return points3D;
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

  // 3) For this skeleton, we hardcode initial guess for focal, k1, k2:
  camera_params[6] = 1000.0; // e.g. some guess for focal
  camera_params[7] = 0.0;    // k1
  camera_params[8] = 0.0;    // k2
}

template <typename T> std::vector<T> createEllipsoidInWorldCoordinate() {
  std::vector<T> objectPointsInWorldCoordinate;

  float X, Y, Z;

  float phiStepSize, thetaStepSize;
  //  phiStepSize = 0.2;
  //  thetaStepSize = 0.1;

  phiStepSize = 1.2;
  thetaStepSize = 0.6;

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

// Rerun helper functions disabled
// template <typename T> rerun::Vec3D getRerunTranslationFromCvMat(cv::Mat t) {
//   rerun::Vec3D translation(t.at<T>(0, 0), t.at<T>(1, 0), t.at<T>(2, 0));
//   return translation;
// }
// template <typename T> rerun::Mat3x3 getRerunRotationFromCvMat(cv::Mat R) {
//   std::array<float, 9> rotation_data = {
//       static_cast<float>(R.at<double>(0, 0)),
//       static_cast<float>(R.at<double>(1, 0)),
//       static_cast<float>(R.at<double>(2, 0)),
//       static_cast<float>(R.at<double>(0, 1)),
//       static_cast<float>(R.at<double>(1, 1)),
//       static_cast<float>(R.at<double>(2, 1)),
//       static_cast<float>(R.at<double>(0, 2)),
//       static_cast<float>(R.at<double>(1, 2)),
//       static_cast<float>(R.at<double>(2, 2)),
//   };
//   return rotation_data;
// }

cv::Mat createImage(double focalLength, int numberOfPixelInHeight,
                    int numberOfPixelInWidth,
                    std::vector<cv::Point2f> projectedPoints,
                    std::string fileName) {

  double row, col;

  cv::Mat cameraImage =
      cv::Mat::zeros(numberOfPixelInHeight, numberOfPixelInWidth, CV_8UC1);

  //  cv::line(cameraImage, cv::Point2d(numberOfPixelInWidth / 2, 0),
  //           cv::Point2d(numberOfPixelInWidth / 2, numberOfPixelInHeight),
  //           cv::Scalar(255, 255, 255));

  //  cv::line(cameraImage, cv::Point2d(0, numberOfPixelInHeight / 2),
  //           cv::Point2d(numberOfPixelInWidth, numberOfPixelInHeight / 2),
  //           cv::Scalar(255, 255, 255));

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

cv::Mat eulerAnglesToRotationMatrix(cv::Vec3d &theta) {
  // Calculate rotation about x axis
  cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(theta[0]),
                 -sin(theta[0]), 0, sin(theta[0]), cos(theta[0]));

  // Calculate rotation about y axis
  cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(theta[1]), 0, sin(theta[1]), 0,
                 1, 0, -sin(theta[1]), 0, cos(theta[1]));

  // Calculate rotation about z axis
  cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(theta[2]), -sin(theta[2]), 0,
                 sin(theta[2]), cos(theta[2]), 0, 0, 0, 1);

  // Combined rotation matrix
  cv::Mat R = R_z * R_y * R_x;

  return R;
}

//------------------------------------------------------------------------------
// ) MAIN: Incremental approach with multiple cameras
//------------------------------------------------------------------------------
int main(int argc, char **argv) {

  const auto rec = rerun::RecordingStream("incremental_SfM");
  rec.spawn().exit_on_failure();
  // OpenCV X=Right, Y=Down, Z=Forward
  rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);

  //  double rollCam0, rollCam1, rollCam2, pitchCam0, pitchCam1, pitchCam2,
  //  yawCam0,
  //      yawCam1, yawCam2;

  double txCam0, txCam1, txCam2, tyCam0, tyCam1, tyCam2, tzCam0, tzCam1, tzCam2;
  cv::Vec3d thetaCam0, thetaCam1, thetaCam2;

  ///////////////// cameras extrinsic /////////////////
  // clang-format off
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


                    Z                        Z                            Z
                    ▲                         ▲                           ▲
                   /                           \                           \
                  /                             \                           \
                 /1 2 3 4     X                  \ 1 2 3 4                   \ 1 2 3 4
           Cam0  |------------ ⯈                 |------------ ⯈Cam1         |------------ ⯈Cam2
                1|                             1 |                          1 |
                2|                             2 |                          2 |
                3|                             3 |                          3 |
               Y |                             Y |                          Y |
                 ⯆                              ⯆                            ⯆


  */
  // clang-format on

  thetaCam0[0] = 0;
  //  thetaCam0[1] = +M_PI / 12;
  thetaCam0[1] = 0;
  thetaCam0[2] = 0;

  thetaCam1[0] = 0;
  //  thetaCam1[1] = -M_PI / 24;
  thetaCam1[1] = 0;
  thetaCam1[2] = 0;

  thetaCam2[0] = 0;
  //  thetaCam2[1] = -M_PI / 10;
  thetaCam2[1] = 0;
  thetaCam2[2] = 0;

  txCam0 = -1;
  tyCam0 = 0.0;
  tzCam0 = -4.0;

  txCam1 = +1;
  tyCam1 = 0.0;
  tzCam1 = -4.0;

  txCam2 = +1;
  tyCam2 = 0.0;
  tzCam2 = -4.0;

  cv::Mat t_Cam0_in_world = (cv::Mat_<double>(3, 1) << txCam0, tyCam0, tzCam0);

  cv::Mat t_Cam1_in_world = (cv::Mat_<double>(3, 1) << txCam1, tyCam1, tzCam1);

  cv::Mat t_Cam2_in_world = (cv::Mat_<double>(3, 1) << txCam2, tyCam2, tzCam2);

  cv::Mat rotation_Cam0_in_world, rotation_Cam1_in_world,
      rotation_Cam2_in_world;

  rotation_Cam0_in_world = eulerAnglesToRotationMatrix(thetaCam0);
  rotation_Cam1_in_world = eulerAnglesToRotationMatrix(thetaCam1);
  rotation_Cam2_in_world = eulerAnglesToRotationMatrix(thetaCam2);

  std::cout << "///////// ground truth /////////" << std::endl;

  std::cout << "Cam0 translation in world:\n" << t_Cam0_in_world << std::endl;

  std::cout << "Cam0 roll, pitch , yaw in world:\n" << thetaCam0 << std::endl;

  std::cout << "Cam1 translation in world:\n" << t_Cam1_in_world << std::endl;

  std::cout << "Cam1 roll, pitch , yaw in world:\n" << thetaCam1 << std::endl;

  std::cout << "Cam2 translation in world:\n" << t_Cam2_in_world << std::endl;

  std::cout << "Cam2 roll, pitch , yaw in world:\n" << thetaCam2 << std::endl;

  ///////// creating ellipsoid in the world coordinate /////////
  std::vector<cv::Vec3f> objectPointsInWorldCoordinate;

  objectPointsInWorldCoordinate = createEllipsoidInWorldCoordinate<cv::Vec3f>();

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
  U0 = (numberOfPixelInHeight) / 2;

  mx = (numberOfPixelInWidth) / widthOfSensor;
  V0 = (numberOfPixelInWidth) / 2;

  cv::Mat K = (cv::Mat_<double>(3, 3) << focalLength * mx, 0, V0, 0,
               focalLength * my, U0, 0, 0, 1);
  std::vector<cv::Point2f> imagePointsCam0, imagePointsCam1, imagePointsCam2;

  double fx = focalLength * mx;
  double cx, cy;
  cx = (numberOfPixelInWidth) / 2;
  cy = (numberOfPixelInHeight) / 2;

  ////////////////////////////////////////////////////////////////////////////////////////

  // Transpose (or inverse) of R_w_c
  cv::Mat rotation_world_in_Cam0 = rotation_Cam0_in_world.t();

  // Correct transformation of the translation vector
  cv::Mat t_world_in_Cam0 = -rotation_world_in_Cam0 * t_Cam0_in_world;

  cv::projectPoints(objectPointsInWorldCoordinate, rotation_world_in_Cam0,
                    t_world_in_Cam0, K, distortionCoefficient, imagePointsCam0);

  // Transpose (or inverse) of R_w_c
  cv::Mat rotation_world_in_Cam1 = rotation_Cam1_in_world.t();

  // Correct transformation of the translation vector
  cv::Mat t_world_in_Cam1 = -rotation_world_in_Cam1 * t_Cam1_in_world;

  cv::projectPoints(objectPointsInWorldCoordinate, rotation_world_in_Cam1,
                    t_world_in_Cam1, K, distortionCoefficient, imagePointsCam1);

  // Transpose (or inverse) of R_w_c
  cv::Mat rotation_world_in_Cam2 = rotation_Cam2_in_world.t();

  // Correct transformation of the translation vector
  cv::Mat t_world_in_Cam2 = -rotation_world_in_Cam2 * t_Cam2_in_world;

  cv::projectPoints(objectPointsInWorldCoordinate, rotation_world_in_Cam2,
                    t_world_in_Cam2, K, distortionCoefficient, imagePointsCam2);

  // Rerun visualization disabled
  // rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);
  // std::vector<rerun::components::Position3D> point3d_positions;
  // std::vector<float> point_sizes;
  // rec.log("world/xyz", rerun::Arrows3D::from_vectors(...));
  // rec.log("world/points", rerun::Points3D(point3d_positions).with_radii(point_sizes));
  // ... (all rerun logging code commented out)

  std::string fileName;

  fileName = std::string("image_cam0_") + std::to_string(focalLength) +
             std::string("_.png");

  cv::Mat img_cam0 =
      createImage(focalLength, numberOfPixelInHeight, numberOfPixelInWidth,
                  imagePointsCam0, fileName);
  // Rerun visualization disabled
  // rec.log("world/cam0/image/rgb", rerun::Image::from_greyscale8(...));

  fileName = std::string("image_cam1_") + std::to_string(focalLength) +
             std::string("_.png");

  cv::Mat img_cam1 =
      createImage(focalLength, numberOfPixelInHeight, numberOfPixelInWidth,
                  imagePointsCam1, fileName);
  // Rerun visualization disabled
  // rec.log("world/cam1/image/rgb", rerun::Image::from_greyscale8(...));

  fileName = std::string("image_cam2_") + std::to_string(focalLength) +
             std::string("_.png");

  cv::Mat img_cam2 =
      createImage(focalLength, numberOfPixelInHeight, numberOfPixelInWidth,
                  imagePointsCam2, fileName);
  // Rerun visualization disabled
  // rec.log("world/cam2/image/rgb", rerun::Image::from_greyscale8(...));

  int N_CAMERAS = 3;

  // std::vector<cv::Point2f> imagePointsCam0
  // imagePointsCam0.

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

  std::cout << "keypoints.size(): " << keypoints.size() << std::endl;
  std::cout << "keypoints[0].size(): " << keypoints[0].size() << std::endl;
  std::cout << "keypoints[1].size(): " << keypoints[1].size() << std::endl;
  std::cout << "keypoints[2].size(): " << keypoints[2].size() << std::endl;
  std::cin.get();

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

    // Draw matches between the two frames
    drawMatchesBetweenTheTwoFrames(images[i], images[i + 1], keypoints[i],
                                   keypoints[i + 1], matches);

    all_matches.push_back(matches);
  }

  std::cout << "all_matches:" << all_matches.size() << "x"
            << all_matches[0].size() << std::endl;

  std::cin.get(); // Waits for the user to press Enter
  std::cout << "Press any key to continue..." << std::endl;

  //  //------------------------------------------------------------------------------
  //  // 1) Reading calibration XML file
  //  //------------------------------------------------------------------------------

  //  // Path to the calibration XML file
  //  std::string calibrationFile = "../data/laptop_calib_result_1280x720.xml";

  //  // Open the file storage
  //  cv::FileStorage fs(calibrationFile, cv::FileStorage::READ);
  //  if (!fs.isOpened()) {
  //    std::cerr << "Failed to open the calibration file: " << calibrationFile
  //              << std::endl;
  //    return -1;
  //  }

  //  //------------------------------------------------------------------------------
  //  // 2) loading camera parameters
  //  //------------------------------------------------------------------------------

  //  // Variables to hold the camera parameters, Intrinsics same for all
  //  cameras cv::Mat distortionCoefficients, extrinsicParameters; int
  //  imageWidth, imageHeight, boardWidth, boardHeight, nrOfFrames; double
  //  squareSize, fixAspectRatio;

  //  // Read the camera parameters
  //  fs["image_width"] >> imageWidth;
  //  fs["image_height"] >> imageHeight;
  //  fs["camera_matrix"] >> K;
  //  fs["distortion_coefficients"] >> distortionCoefficients;
  //  fs["extrinsic_parameters"] >> extrinsicParameters;

  //  // Close the file storage
  //  fs.release();

  //  double fx = K.at<double>(0, 0);
  //  std::cout << "fx:" << fx << std::endl;

  //  double fy = K.at<double>(1, 1);
  //  std::cout << "fy:" << fy << std::endl;

  //  double cx = K.at<double>(0, 2);
  //  std::cout << "cx:" << cx << std::endl;

  //  double cy = K.at<double>(1, 2);
  //  std::cout << "cy:" << cy << std::endl;

  //  double k1 = distortionCoefficients.at<double>(0, 0);
  //  double k2 = distortionCoefficients.at<double>(1, 0);
  //  double p1 = distortionCoefficients.at<double>(2, 0);
  //  double p2 = distortionCoefficients.at<double>(3, 0);
  //  double k3 = distortionCoefficients.at<double>(4, 0);

  //    k1 = 0.0;
  //    k2 = 0.0;
  //    p1 = 0.0;
  //    p2 = 0.0;
  //    k3 = 0.0;

  //  //------------------------------------------------------------------------------
  //  // 3) loading images
  //  //------------------------------------------------------------------------------

  //  const std::string directoryPath = "../images/";
  //  std::vector<std::string> image_paths = loadImagePaths(directoryPath);

  //  for (const auto &img : image_paths)
  //    std::cout << img << std::endl;

  //  std::vector<cv::Mat> images;
  //  for (const auto &path : image_paths) {
  //    images.push_back(cv::imread(path, cv::IMREAD_GRAYSCALE));
  //    if (images.back().empty()) {
  //      std::cerr << "Failed to load image: " << path << std::endl;
  //      return -1;
  //    }
  //  }

  //  //------------------------------------------------------------------------------
  //  // 4) Detect and compute features for all images
  //  //------------------------------------------------------------------------------
  //  N_CAMERAS = images.size();

  //  cv::Ptr<cv::SIFT> detector = cv::SIFT::create();
  //  std::vector<cv::Mat> descriptors(N_CAMERAS);

  //  for (size_t i = 0; i < N_CAMERAS; ++i) {
  //    detector->detectAndCompute(images[i], cv::noArray(), keypoints[i],
  //                               descriptors[i]);

  //    // Display the number of keypoints detected for each image
  //    std::cout << "Image: " << image_paths[i]
  //              << " - Number of keypoints: " << keypoints[i].size() <<
  //              std::endl;
  //  }

  //------------------------------------------------------------------------------
  // 5) Filtering top quantiles matches and draw them, If you choose "2", you
  // are asking for all elements below the median (50th percentile). If you
  // choose "4", you are asking for elements below the 25th percentile (1/4 of
  // the sorted data).
  //------------------------------------------------------------------------------

  //  int quantiles = 4;
  //  for (size_t i = 0; i < N_CAMERAS - 1; ++i) {
  //    std::vector<cv::DMatch> matches;
  //    matchDescriptorsBetweenConsecutiveImages(descriptors, keypoints,
  //    all_matches, matches, i, quantiles);
  //    // Draw matches between the two frames

  //    //    drawMatchesBetweenTheTwoFrames(images, keypoints, image_paths,
  //    //    matches, i);
  //  }

  //------------------------------------------------------------------------------
  // 6) We'll store each camera's extrinsics (R, t) in a
  // vector<CameraExtrinsics>, this brings the 3d points in world (camera1) into
  // ith camera, so cameras[i]={R_1^i, t_1^i  }, camera1 is identity.
  //------------------------------------------------------------------------------

  // camera1  is the world,we set identity and zero
  std::vector<CameraExtrinsics> cameras(N_CAMERAS);
  cameras[0].R = cv::Mat::eye(3, 3, CV_64F);
  cameras[0].t = cv::Mat::zeros(3, 1, CV_64F);

  //------------------------------------------------------------------------------
  // 7) We will also store the 9 parameters for each camera for Ceres:
  // angle-axis (3 param), translation (3 param), focal, k1, k2)
  // cameraParams: is a vector of shape  [9*N_CAMERAS x 1]
  // rows: 9*N_CAMERAS
  // cols:  1
  //------------------------------------------------------------------------------

  std::vector<double> cameraParams(9 * N_CAMERAS, 0.0);

  // Initialize camera0 in the bundle with R=I, t=0 => angle-axis=0, ...
  RtToAngleAxisAndTranslate(cameras[0].R, cameras[0].t, &cameraParams[0 * 9]);
  cameraParams[0 * 9 + 6] = fx; // focal guess
  cameraParams[0 * 9 + 7] = k1; // k1
  cameraParams[0 * 9 + 8] = k2; // k2

  // All 2D observations of these 3D points, camera_idx, 3d point_idx, x, y
  std::vector<Observation> observations;

  // All 3D points in world coords
  std::vector<cv::Point3f> globalPoints3D;

  //--------------------------------------------------------------------------
  // 8) INCREMENTAL LOOP: from camera i=1..N-1
  //--------------------------------------------------------------------------

  for (size_t i = 1; i < N_CAMERAS - 1; ++i) {
    std::cout << "camera: " << i - 1 << " and camera: " << i << std::endl;
    std::cout << "all_matches[i].size(): " << all_matches[i].size()
              << std::endl;

    // 1) Extract matched points, We have matched points between camera (i-1)
    // and camera i:
    std::vector<cv::Point2f> pts_im1, pts_i;
    for (const auto &match : all_matches[i]) {

      std::cout << "keypoints[" << i - 1 << "][" << match.queryIdx
                << "].pt: " << keypoints[i - 1][match.queryIdx].pt << std::endl;

      std::cout << "keypoints[" << i << "][" << match.trainIdx
                << "].pt: " << keypoints[i][match.trainIdx].pt << std::endl;

      pts_im1.push_back(keypoints[i - 1][match.queryIdx].pt);
      pts_i.push_back(keypoints[i][match.trainIdx].pt);

      std::cout << "------------------------------------------" << std::endl;
    }

    // 2) Compute Essential matrix & recoverPose
    cv::Mat mask;
    cv::Mat E =
        cv::findEssentialMat(pts_im1, pts_i, K, cv::RANSAC, 0.999, 1.0, mask);

    std::cout << "E:\n" << E << std::endl;

    cv::Mat R_im1_to_i, t_im1_to_i;
    cv::recoverPose(E, pts_im1, pts_i, K, R_im1_to_i, t_im1_to_i, mask);

    std::cout << "R_im1_to_i:\n" << R_im1_to_i << std::endl;
    std::cout << "t_im1_to_i:\n" << t_im1_to_i << std::endl;

    //--------------------------------------------------------------------------
    // 9) Now we want camera i has the transformation to transform points in 3d
    // camera1 is the world, we know camera [i-1] has extrinsics (R_1_to_im1,
    // t_1_to_im1). So: R_{1->i} = R_{1->(i-1)} * R_{(i-1)->i} t_{1->i} =
    // R_{1->(i-1)} * t_{(i-1)->i} + t_{1->(i-1)}
    //--------------------------------------------------------------------------

    cv::Mat R_1_to_im1 = cameras[i - 1].R;
    cv::Mat t_1_to_im1 = cameras[i - 1].t;

    cv::Mat R_1_to_i = R_1_to_im1 * R_im1_to_i;
    cv::Mat t_1_to_i = R_1_to_im1 * t_im1_to_i + t_1_to_im1;

    cameras[i].R = R_1_to_i.clone();
    cameras[i].t = t_1_to_i.clone();

    //--------------------------------------------------------------------------
    // 10) Convert to 9-parameter form for Ceres and put in cameraParams,
    // cameraParams[i * 9] is the starting address and up to address
    // cameraParams[i * 9+6] will be populated by rotation and translation
    //--------------------------------------------------------------------------

    RtToAngleAxisAndTranslate(cameras[i].R, cameras[i].t, &cameraParams[i * 9]);

    //--------------------------------------------------------------------------
    // 11) Triangulate some new 3D points from (i-1, i):
    //    We'll do a triangulation using (i-1) as [R_im1, t_im1] and (i) as
    //    [R_i, t_i].
    //--------------------------------------------------------------------------

    // Build projection matrix for camera(i-1) in world => let's call it P_im1,
    // this bring points in 3d from camera 1 to camera[i-1]

    //   P_im1 = K * [R_im1 | t_im1]
    cv::Mat Rt_im1(3, 4, CV_64F);
    cameras[i - 1].R.copyTo(Rt_im1(cv::Rect(0, 0, 3, 3)));
    cameras[i - 1].t.copyTo(Rt_im1(cv::Rect(3, 0, 1, 3)));
    cv::Mat P_im1 = K * Rt_im1;

    // Build projection matrix for camera i => P_i
    cv::Mat Rt_i(3, 4, CV_64F);
    cameras[i].R.copyTo(Rt_i(cv::Rect(0, 0, 3, 3)));
    cameras[i].t.copyTo(Rt_i(cv::Rect(3, 0, 1, 3)));
    cv::Mat P_i = K * Rt_i;

    //--------------------------------------------------------------------------
    // 12) Because both P_im1 and Pi assume X is in that same world reference,
    // camera[1] frame, the output of cv::triangulatePoints is a set of 4D
    // homogeneous coordinates X world in that same world frame. After you
    // convert them via convertHomogeneous those 3D points (newPoints) are in
    // the coordinate system of camera 1, which you designated as the “world.”
    //--------------------------------------------------------------------------

    for (std::size_t i = 0; i < pts_im1.size(); i++) {
      std::cout << "pts_im1[i]" << pts_im1[i] << std::endl;
      std::cout << pts_i[i] << std::endl;
    }

    cv::Mat points4D;
    cv::triangulatePoints(P_im1, P_i, pts_im1, pts_i, points4D);
    std::vector<cv::Point3f> newPoints = convertHomogeneous(points4D);

    for (const auto &p : newPoints) {
      //      std::cout << p.x << "," << p.y << "," << p.z << std::endl;
    }

    std::cout << "Rt_im1:\n" << Rt_im1 << std::endl;

    std::cout << "P_im1:\n" << P_im1 << std::endl;

    std::cout << "Rt_i:\n" << Rt_i << std::endl;

    std::cout << "P_i:\n" << P_i << std::endl;

    std::cin.get(); // Waits for the user to press Enter
    std::cout << "Press any key to continue..." << std::endl;

    //--------------------------------------------------------------------------
    // 13) Add these new points to our globalPoints3D array,
    //    and record the 2D observations for them in camera(i-1) and camera(i).
    // All 2D observations of these 3D points, camera_idx, 3d point_idx, x, y
    //--------------------------------------------------------------------------

    for (size_t k = 0; k < newPoints.size(); k++) {
      int newPointIdx = static_cast<int>(globalPoints3D.size());
      globalPoints3D.push_back(newPoints[k]);

      std::cout << "newPoints[" << k << "]: " << newPoints[k] << std::endl;

      // Observation from camera (i-1):
      {
        Observation obs;
        obs.camera_idx = i - 1;
        obs.point_idx = newPointIdx;
        obs.x = pts_im1[k].x;
        obs.y = pts_im1[k].y;
        observations.push_back(obs);

        //        std::cout << "obs.point_idx: " << obs.point_idx << std::endl;
      }
      // Observation from camera (i):
      {
        Observation obs;
        obs.camera_idx = i;
        obs.point_idx = newPointIdx;
        obs.x = pts_i[k].x;
        obs.y = pts_i[k].y;
        observations.push_back(obs);
      }
    }
  }

  //  // pointParamsPtr => we store 3D points in world in globalPoints3D,
  //  // but for Snavely we want them as a double[3]. We'll create a dynamic
  //  // array: Alternatively, we can store them in a single global
  //  vector<double>
  //  // of size 3*#points. For brevity, let's do a quick hack: store them
  //  // temporarily in a static array, then feed them to the CostFunction. (But
  //  // typically you'd keep them in a single vector.)
  //  std::vector<double> pointParams;
  //  pointParams.resize(3 * globalPoints3D.size());
  //  // Copy from cv::Point3f to the double vector
  //  for (size_t i = 0; i < globalPoints3D.size(); i++) {
  //    pointParams[3 * i + 0] = globalPoints3D[i].x;
  //    pointParams[3 * i + 1] = globalPoints3D[i].y;
  //    pointParams[3 * i + 2] = globalPoints3D[i].z;
  //  }

  //  //--------------------------------------------------------------------------
  //  // At this point, we have:
  //  //  - N_CAMERAS cameras in cameras[], with extrinsics in world coords
  //  //  - globalPoints3D (3D points in world coords)
  //  //  - observations (2D) telling us which camera sees which point, with
  //  OpenCV
  //  //  reproject model and NOT with SnavelyReprojectionError
  //  //
  //  // Next: Setup a Ceres bundle adjustment problem using
  //  // SnavelyReprojectionError.
  //  //--------------------------------------------------------------------------
  //  ceres::Problem problem;

  //  std::cout << "observations.size(): " << observations.size() << std::endl;

  //  // For each observation (camera_idx, point_idx, x, y)
  //  for (const Observation &obs : observations) {

  //    //    std::cout << "obs.point_idx: " << obs.point_idx << std::endl;

  //    double imagePointCamera_x = obs.x;
  //    double imagePointCamera_y = obs.y;

  //    auto obs_x = -(imagePointCamera_x - cx);
  //    auto obs_y = -(imagePointCamera_y - cy);

  //    // cameraParamsPtr => address of the 9 parameters for that camera,
  //    // cameraParams: 9 x N_CAMERAS
  //    double *cameraParamsPtr = &cameraParams[obs.camera_idx * 9];

  //    double *pointPtr = &pointParams[3 * obs.point_idx];
  //    std::cout << pointPtr[0] << "," << pointPtr[1] << "," << pointPtr[2]
  //              << std::endl;

  //    // Create the reprojection error cost:
  //    ceres::CostFunction *costFunc =
  //        SnavelyReprojectionError::Create(obs_x, obs_y);

  //    // Add a residual block:
  //    problem.AddResidualBlock(costFunc,
  //                             nullptr, // squared loss
  //                             cameraParamsPtr, pointPtr);
  //  }

  //  // Optionally, set some parameters as constant (e.g. fix the first
  //  camera): problem.SetParameterBlockConstant(&cameraParams[0]); // fix
  //  camera 0?

  //  // Configure solver:
  //  ceres::Solver::Options options;
  //  options.linear_solver_type = ceres::DENSE_SCHUR;
  //  options.minimizer_progress_to_stdout = true;
  //  options.max_num_iterations = 100;

  //  // Solve BA:
  //  ceres::Solver::Summary summary;
  //  ceres::Solve(options, &problem, &summary);
  //  std::cout << "Bundle Adjustment Summary:\n"
  //            << summary.FullReport() << std::endl;

  //  //--------------------------------------------------------------------------
  //  //  FINAL: we have refined cameraParams & 3D points. Typically you'd
  //  //         copy them back to your pipeline or continue incremental steps.
  //  //--------------------------------------------------------------------------
  //  // For example, let's print out the final camera1's angle-axis,
  //  translation,
  //  // focal:
  //  std::cout << "Camera0 final parameters:\n";
  //  std::cout << "Angle-axis = (" << cameraParams[0] << ", " <<
  //  cameraParams[1]
  //            << ", " << cameraParams[2] << ")\n";
  //  std::cout << "Translation = (" << cameraParams[3] << ", " <<
  //  cameraParams[4]
  //            << ", " << cameraParams[5] << ")\n";
  //  std::cout << "Focal length = " << cameraParams[6] << "\n";
  //  std::cout << "k1 = " << cameraParams[7] << ", k2 = " << cameraParams[8]
  //            << "\n";

  return 0;
}
