#include "../collection_adapters.hpp"
#include "../snavely_reprojection_error.hpp"
#include <algorithm>
#include <ceres/ceres.h>
#include <filesystem>
#include <fstream>
#include <iostream>
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

namespace fs = std::filesystem;

// Function to extract numeric parts of file names for sorting
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
    for (const auto &entry : fs::directory_iterator(directoryPath)) {
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
                return extractNumber(fs::path(a).stem().string()) <
                       extractNumber(fs::path(b).stem().string());
              });
  } catch (const std::exception &e) {
    std::cerr << "Error reading directory: " << e.what() << std::endl;
  }

  return imagePaths;
}

// Match descriptors between consecutive images
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

// Draw matches between the two frames

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

struct CameraPose {
  cv::Mat R; // Rotation matrix
  cv::Mat t; // Translation vector
};

int SfM() {

  // Path to the calibration XML file
  std::string calibrationFile = "../data/laptop_calib_result_1280x720.xml";

  // Open the file storage
  cv::FileStorage fs(calibrationFile, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Failed to open the calibration file: " << calibrationFile
              << std::endl;
    return -1;
  }

  // Variables to hold the camera parameters
  cv::Mat K, distortionCoefficients, extrinsicParameters;
  int imageWidth, imageHeight, boardWidth, boardHeight, nrOfFrames;
  double squareSize, fixAspectRatio;
  bool fisheyeModel;

  // Read the camera parameters
  fs["image_width"] >> imageWidth;
  fs["image_height"] >> imageHeight;
  fs["camera_matrix"] >> K;
  fs["distortion_coefficients"] >> distortionCoefficients;
  fs["extrinsic_parameters"] >> extrinsicParameters;

  // Close the file storage
  fs.release();

  const std::string directoryPath = "../images/";

  double fx = K.at<double>(0, 0);
  std::cout << "fx:" << fx << std::endl;

  double fy = K.at<double>(1, 1);
  std::cout << "fy:" << fy << std::endl;

  double k1 = distortionCoefficients.at<double>(0, 0);
  double k2 = distortionCoefficients.at<double>(1, 0);
  double p1 = distortionCoefficients.at<double>(2, 0);
  double p2 = distortionCoefficients.at<double>(3, 0);
  double k3 = distortionCoefficients.at<double>(4, 0);

  // Step 1: Load images
  std::vector<std::string> image_paths = loadImagePaths(directoryPath);

  for (const auto &img : image_paths)
    std::cout << img << std::endl;

  std::vector<cv::Mat> images;
  for (const auto &path : image_paths) {
    images.push_back(cv::imread(path, cv::IMREAD_GRAYSCALE));
    if (images.back().empty()) {
      std::cerr << "Failed to load image: " << path << std::endl;
      return -1;
    }
  }

  cv::Ptr<cv::SIFT> detector = cv::SIFT::create();
  std::vector<std::vector<cv::KeyPoint>> keypoints(images.size());
  std::vector<cv::Mat> descriptors(images.size());

  // Detect and compute features for all images
  for (size_t i = 0; i < images.size(); ++i) {
    detector->detectAndCompute(images[i], cv::noArray(), keypoints[i],
                               descriptors[i]);

    // Display the number of keypoints detected for each image
    std::cout << "Image: " << image_paths[i]
              << " - Number of keypoints: " << keypoints[i].size() << std::endl;
  }

  std::vector<std::vector<cv::DMatch>> all_matches;
  /*
  If you choose "2", you are asking for all elements below the median (50th
  percentile). If you choose "4", you are asking for elements below the 25th
  percentile (1/4 of the sorted data).
  */

  int quantiles = 4;
  for (size_t i = 0; i < images.size() - 1; ++i) {

    std::vector<cv::DMatch> matches;

    matchDescriptorsBetweenConsecutiveImages(
        descriptors, keypoints, all_matches, matches, i, quantiles);

    // Draw matches between the two frames
    //    drawMatchesBetweenTheTwoFrames(images, keypoints, image_paths,
    //    matches, i);
  }

  // Step 3: Estimate poses and triangulate points
  std::vector<CameraPose> camera_poses(images.size());
  camera_poses[0].R = cv::Mat::eye(3, 3, CV_64F);
  camera_poses[0].t = cv::Mat::zeros(3, 1, CV_64F);

  std::vector<std::array<double, 3>> points_3d;
  std::vector<std::vector<std::pair<int, cv::Point2f>>> observations;

  const auto rec = rerun::RecordingStream("rerun_sfm_RIGHT_HAND_Y_DOWN");
  rec.spawn().exit_on_failure();

  for (size_t i = 0; i < images.size() - 1; ++i) {
    if (i >= all_matches.size()) {
      break;
    }

    // Extract matched points
    std::vector<cv::Point2f> points1, points2;
    for (const auto &match : all_matches[i]) {
      points1.push_back(keypoints[i][match.queryIdx].pt);
      points2.push_back(keypoints[i + 1][match.trainIdx].pt);
    }

    // Compute relative pose
    cv::Mat E = cv::findEssentialMat(points1, points2, K, cv::RANSAC);

    if (E.empty()) {
      std::cerr << "Failed to compute essential matrix between images " << i
                << " and " << i + 1 << std::endl;
      continue;
    }

    cv::Mat R, t;
    if (!cv::recoverPose(E, points1, points2, K, R, t)) {
      std::cerr << "Failed to recover pose between images " << i << " and "
                << i + 1 << std::endl;
      continue;
    }

    //    // Accumulate pose
    //    camera_poses[i + 1].R = R;
    //    camera_poses[i + 1].t = t;

    // Update pose for the next camera
    camera_poses[i + 1].R = camera_poses[i].R * R;
    camera_poses[i + 1].t = camera_poses[i].R * t + camera_poses[i].t;

    // Triangulate points
    cv::Mat R1 = camera_poses[i].R;
    cv::Mat t1 = camera_poses[i].t;

    cv::Mat Rt1;
    cv::hconcat(R1, t1, Rt1);

    cv::Mat proj1 = K * Rt1;

    cv::Mat R_next = camera_poses[i + 1].R;
    cv::Mat t_next = camera_poses[i + 1].t;

    cv::Mat Rt_next;
    cv::hconcat(R_next, t_next, Rt_next);

    cv::Mat proj2 = K * Rt_next;

    cv::Mat points_4d;
    cv::triangulatePoints(proj1, proj2, points1, points2, points_4d);

    std::cout << "proj1:\n" << proj1 << std::endl;
    std::cout << "proj2:\n" << proj2 << std::endl;

    // Step 3: Validate triangulated points
    for (int j = 0; j < points_4d.cols; ++j) {
      cv::Mat col = points_4d.col(j);

      //      std::cout << "w,x,y,z:" << col.at<float>(3) << "," <<
      //      col.at<float>(0)
      //                << "," << col.at<float>(1) << "," << col.at<float>(2)
      //                << std::endl;

      if (std::abs(col.at<float>(3)) >
          1e-6) {                // Validate homogeneous coordinate
        col /= col.at<float>(3); // Convert to non-homogeneous coordinates
        points_3d.push_back(
            {col.at<float>(0), col.at<float>(1), col.at<float>(2)});
        observations.push_back({{static_cast<int>(i), points1[j]},
                                {static_cast<int>(i + 1), points2[j]}});
      }
    }

    // Step 4: Optimize using Ceres Solver

    //////////////////////////////////////////////////////////////////////////////////////////

    // Calculate reprojection error for this pair
    double total_reprojection_error = 0.0;
    int num_points = 0;

    //    for (size_t point_idx = 0; point_idx < points_3d.size(); ++point_idx)
    //    {
    //      for (const auto &obs : observations[point_idx]) {
    //        if (obs.first == static_cast<int>(i) ||
    //            obs.first == static_cast<int>(i + 1)) {
    //          double camera_params[6];
    //          cv::Mat rvec;
    //          cv::Rodrigues(obs.first == static_cast<int>(i)
    //                            ? camera_poses[i].R
    //                            : camera_poses[i + 1].R,
    //                        rvec);
    //          for (int k = 0; k < 3; ++k) {
    //            camera_params[k] = rvec.at<double>(k);
    //            camera_params[k + 3] =
    //                (obs.first == static_cast<int>(i) ? camera_poses[i].t
    //                                                  : camera_poses[i + 1].t)
    //                    .at<double>(k);
    //          }

    //          double residuals[2];
    //          SnavelyReprojectionError error(obs.second.x, obs.second.y, fx,
    //          k1, k2,
    //                                         k3);
    //          error(camera_params, points_3d[point_idx].data(), residuals);

    //          total_reprojection_error += std::sqrt(residuals[0] *
    //          residuals[0] +
    //                                                residuals[1] *
    //                                                residuals[1]);
    //          num_points++;
    //        }
    //      }
    //    }

    //    double avg_reprojection_error =
    //        num_points > 0 ? total_reprojection_error / num_points : 0.0;
    //    std::cout << "Average Reprojection Error for pair " << i << ", " << i
    //    + 1
    //              << ": " << avg_reprojection_error << std::endl;

    //    /////////////////////////////////////////////////////////////////////////////////////////

    //    // points_3d points are in the ith camera frame, we need to transform
    //    them
    //    // into world coordinate
    //    std::vector<std::array<double, 3>> points_3d_world;

    //    for (const auto &p : points_3d) {
    //      cv::Mat point_camera = (cv::Mat_<double>(3, 1) << p[0], p[1], p[2]);
    //      cv::Mat point_world =
    //          camera_poses[i].R * point_camera + camera_poses[i].t;

    //      points_3d_world.push_back({point_world.at<double>(0, 0),
    //                                 point_world.at<double>(1, 0),
    //                                 point_world.at<double>(2, 0)});
    //    }

    // OpenCV X=Right, Y=Down, Z=Forward
    rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);

    // Log the arrows to the Rerun Viewer
    rec.log("world/xyz",
            rerun::Arrows3D::from_vectors(
                {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}})
                .with_colors({{255, 0, 0}, {0, 255, 0}, {0, 0, 255}}));

    std::string camera_name = "world/camera" + std::to_string(i);
    std::cout << "camera_name: " << camera_name << std::endl;
    rec.log(camera_name, rerun::Pinhole::from_focal_length_and_resolution(
                             {float(fx), float(fy)},
                             {float(imageWidth), float(imageHeight)}));

    // Extract translation vector
    rerun::Vec3D translation(camera_poses[i].t.at<double>(0, 0),
                             camera_poses[i].t.at<double>(1, 0),
                             camera_poses[i].t.at<double>(2, 0));

    std::cout << "t:" << camera_poses[i].t.at<double>(0, 0) << ","
              << camera_poses[i].t.at<double>(1, 0) << ","
              << camera_poses[i].t.at<double>(2, 0) << std::endl;

    std::cout << "R:" << camera_poses[i].R << std::endl;

    // Extract rotation matrix as std::array<float, 9> in
    // column-major order
    std::array<float, 9> rotation_data = {
        static_cast<float>(camera_poses[i].R.at<double>(0, 0)),
        static_cast<float>(camera_poses[i].R.at<double>(1, 0)),
        static_cast<float>(camera_poses[i].R.at<double>(2, 0)),

        static_cast<float>(camera_poses[i].R.at<double>(0, 1)),
        static_cast<float>(camera_poses[i].R.at<double>(1, 1)),
        static_cast<float>(camera_poses[i].R.at<double>(2, 1)),

        static_cast<float>(camera_poses[i].R.at<double>(0, 2)),
        static_cast<float>(camera_poses[i].R.at<double>(1, 2)),
        static_cast<float>(camera_poses[i].R.at<double>(2, 2)),
    };

    rerun::Mat3x3 rotation_matrix;
    rotation_matrix = rotation_data;

    // Log the data
    rec.log(camera_name, rerun::Transform3D(translation, rotation_matrix));

    // Read image
    cv::Mat img = cv::imread(image_paths[i], cv::IMREAD_COLOR);
    if (img.empty()) {
      std::cout << "Could not read the image: " << image_paths[i] << std::endl;
      return 1;
    } else {
      std::cout << "reading the image at: " << image_paths[i] << std::endl;
    }

    //    uint32_t width = img.cols;
    //    uint32_t height = img.rows;

    // Convert image to RGB as Rerun expects this format
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    // Log the image to the camera entity in the hierarchy
    rec.log(camera_name + "/image/rgb",
            rerun::Image::from_rgb24(
                img, {uint32_t(imageWidth), uint32_t(imageHeight)}));

    // Prepare keypoints for logging
    std::vector<rerun::components::Position2D> keypoint_positions;
    for (const auto &kp : keypoints[i]) {
      keypoint_positions.push_back({kp.pt.x, kp.pt.y});
    }

    // Log the keypoints to the Rerun Viewer
    rec.log(camera_name + "/image/keypoints",
            rerun::Points2D(keypoint_positions));

    // Prepare 3d points for logging
    std::vector<rerun::components::Position3D> point3d_positions;
    for (const auto &p : points_3d) {
      point3d_positions.push_back({float(p[0]), float(p[1]), float(p[2])});

      //      std::cout << "points_3d: " << float(p[0]) << "," << float(p[1]) <<
      //      ","
      //                << float(p[2]) << std::endl;
    }

    std::cout << "Press any key to continue..." << std::endl;
    std::cin.get(); // Waits for the user to press Enter

    rec.log("world/points", rerun::Points3D(point3d_positions));

    //    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}

int main() {

  SfM();
  return 0;
}

/*

DATASET_PATH=/home/behnam/workspace/slam-tutorials
mkdir $DATASET_PATH/sparse
mkdir $DATASET_PATH/dense
CAM=848.5311753987206,848.5311753987206,639.5,359.5,0.15971715887123378,-0.6104577942629438,0.0,0.0








./colmap feature_extractor  --database_path $DATASET_PATH/database.db
--image_path $DATASET_PATH/images  --ImageReader.single_camera=true
--ImageReader.camera_model=OPENCV --ImageReader.camera_params=$CAM
--SiftExtraction.use_gpu 0





./colmap sequential_matcher \
   --database_path $DATASET_PATH/database.db \
   --SequentialMatching.overlap=2 \
   --SiftMatching.use_gpu 0 --SiftMatching.gpu_index=-1
--SiftMatching.guided_matching=true



./colmap mapper \
    --database_path $DATASET_PATH/database.db \
    --image_path $DATASET_PATH/images \
    --output_path $DATASET_PATH/sparse
*/
