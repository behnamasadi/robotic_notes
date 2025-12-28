#include "../arguments_parser.hpp"
#include "../collection_adapters.hpp"
#include "../snavely_reprojection_error.hpp"
#include "../transformation.hpp"
#include <algorithm>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <filesystem>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <rerun/archetypes/arrows3d.hpp>
#include <rerun/datatypes/rgba32.hpp>

using namespace ceres::examples;
namespace fs = std::filesystem;

//------------------------------------------------------------------------------
// Convert from homogeneous (4 x N) to std::vector<cv::Point3f> (divide by w)
//------------------------------------------------------------------------------
std::vector<cv::Point3f> convertHomogeneous(const cv::Mat &points4D) {
  std::vector<cv::Point3f> points3D;
  points3D.reserve(points4D.cols);

  for (int i = 0; i < points4D.cols; ++i) {
    cv::Point3f point;
    point.x = points4D.at<float>(0, i) / points4D.at<float>(3, i);
    point.y = points4D.at<float>(1, i) / points4D.at<float>(3, i);
    point.z = points4D.at<float>(2, i) / points4D.at<float>(3, i);
    points3D.push_back(point);
  }
  return points3D;
}

//------------------------------------------------------------------------------
// Utility to convert (R,t) from OpenCV to the 6-parameter format for Ceres
// - camera[0..2]: angle-axis
// - camera[3..5]: translation
//------------------------------------------------------------------------------
void RtToAngleAxisAndTranslate(const cv::Mat &R, const cv::Mat &t,
                               double *camera_params) {
  cv::Mat rodrigues;
  cv::Rodrigues(R, rodrigues);

  if (rodrigues.rows == 3 && rodrigues.cols == 1) {
    camera_params[0] = rodrigues.at<double>(0, 0);
    camera_params[1] = rodrigues.at<double>(1, 0);
    camera_params[2] = rodrigues.at<double>(2, 0);
  } else if (rodrigues.rows == 1 && rodrigues.cols == 3) {
    camera_params[0] = rodrigues.at<double>(0, 0);
    camera_params[1] = rodrigues.at<double>(0, 1);
    camera_params[2] = rodrigues.at<double>(0, 2);
  } else {
    std::cerr << "ERROR: Rodrigues vector has unexpected dimensions: "
              << rodrigues.rows << "x" << rodrigues.cols << std::endl;
  }

  camera_params[3] = t.at<double>(0, 0);
  camera_params[4] = t.at<double>(1, 0);
  camera_params[5] = t.at<double>(2, 0);
}

//------------------------------------------------------------------------------
// Simple struct to hold per-camera data
//------------------------------------------------------------------------------
struct CameraExtrinsics {
  cv::Mat R; // 3x3
  cv::Mat t; // 3x1
};

//------------------------------------------------------------------------------
// Observation structure
//------------------------------------------------------------------------------
struct Observation {
  int camera_idx;
  int point_idx;
  double x, y; // Snavely normalized coordinates
};

//------------------------------------------------------------------------------
// Feature observation and Track structures (from virtual camera version)
//------------------------------------------------------------------------------
struct FeatureObservation {
  int camera_idx;
  int feature_idx;
  cv::Point2f pixel;

  FeatureObservation(int cam, int feat, const cv::Point2f &px)
      : camera_idx(cam), feature_idx(feat), pixel(px) {}
};

struct Track {
  int point3D_idx;
  std::vector<FeatureObservation> observations;

  Track() : point3D_idx(-1) {}

  void addObservation(int camera_idx, int feature_idx,
                      const cv::Point2f &pixel) {
    observations.emplace_back(camera_idx, feature_idx, pixel);
  }

  bool isObservedBy(int camera_idx) const {
    for (const auto &obs : observations) {
      if (obs.camera_idx == camera_idx)
        return true;
    }
    return false;
  }

  int getFeatureIdx(int camera_idx) const {
    for (const auto &obs : observations) {
      if (obs.camera_idx == camera_idx)
        return obs.feature_idx;
    }
    return -1;
  }

  int numObservations() const { return static_cast<int>(observations.size()); }
};

//------------------------------------------------------------------------------
// Load camera intrinsics from file (YAML format)
//------------------------------------------------------------------------------
bool loadCameraIntrinsics(const std::string &filepath, cv::Mat &K,
                          cv::Mat &distCoeffs, double &k1, double &k2) {
  cv::FileStorage fs(filepath, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "ERROR: Cannot open camera file: " << filepath << std::endl;
    return false;
  }

  fs["camera_matrix"] >> K;
  fs["distortion_coefficients"] >> distCoeffs;

  if (K.empty() || distCoeffs.empty()) {
    std::cerr << "ERROR: Camera matrix or distortion coefficients not found"
              << std::endl;
    return false;
  }

  // Extract k1 and k2 (radial distortion)
  k1 = distCoeffs.at<double>(0);
  k2 = distCoeffs.at<double>(1);

  std::cout << "Camera intrinsics loaded successfully:" << std::endl;
  std::cout << "K = \n" << K << std::endl;
  std::cout << "Distortion coefficients = " << distCoeffs.t() << std::endl;
  std::cout << "k1 = " << k1 << ", k2 = " << k2 << std::endl;

  fs.release();
  return true;
}

//------------------------------------------------------------------------------
// Load images from directory
//------------------------------------------------------------------------------
std::vector<cv::Mat>
loadImagesFromDirectory(const std::string &dirPath,
                        std::vector<std::string> &filenames) {
  std::vector<cv::Mat> images;
  std::vector<std::string> imageFiles;

  // Collect image files
  for (const auto &entry : fs::directory_iterator(dirPath)) {
    if (entry.is_regular_file()) {
      std::string ext = entry.path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

      if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp" ||
          ext == ".tiff") {
        imageFiles.push_back(entry.path().string());
      }
    }
  }

  // Sort by filename to ensure consistent ordering
  std::sort(imageFiles.begin(), imageFiles.end());

  std::cout << "Found " << imageFiles.size() << " images in directory"
            << std::endl;

  // Load images
  for (const auto &filepath : imageFiles) {
    cv::Mat img = cv::imread(filepath);
    if (img.empty()) {
      std::cerr << "Warning: Could not load image: " << filepath << std::endl;
      continue;
    }

    // Convert to grayscale for feature detection
    cv::Mat gray;
    if (img.channels() == 3) {
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = img;
    }

    images.push_back(gray);
    filenames.push_back(fs::path(filepath).filename().string());
    std::cout << "  Loaded: " << filenames.back() << " (" << gray.cols << "x"
              << gray.rows << ")" << std::endl;
  }

  return images;
}

//------------------------------------------------------------------------------
// Detect and match features between consecutive images
//------------------------------------------------------------------------------
void detectAndMatchFeatures(const cv::Mat &img1, const cv::Mat &img2,
                            std::vector<cv::KeyPoint> &keypoints1,
                            std::vector<cv::KeyPoint> &keypoints2,
                            std::vector<cv::DMatch> &good_matches,
                            bool use_sift = true, float ratio_thresh = 0.7f) {

  cv::Ptr<cv::Feature2D> detector;

  if (use_sift) {
    detector = cv::SIFT::create(0, 3, 0.04, 10, 1.6);
    std::cout << "Using SIFT feature detector" << std::endl;
  } else {
    detector = cv::ORB::create(2000);
    std::cout << "Using ORB feature detector" << std::endl;
  }

  // Detect keypoints and compute descriptors
  cv::Mat descriptors1, descriptors2;
  detector->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
  detector->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

  std::cout << "  Detected " << keypoints1.size() << " keypoints in image 1"
            << std::endl;
  std::cout << "  Detected " << keypoints2.size() << " keypoints in image 2"
            << std::endl;

  // Match descriptors
  cv::Ptr<cv::DescriptorMatcher> matcher;
  if (use_sift) {
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
  } else {
    matcher = cv::DescriptorMatcher::create(
        cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
  }

  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

  // Apply Lowe's ratio test - lower ratio = more strict filtering
  std::cout << "  Using Lowe's ratio threshold: " << ratio_thresh << std::endl;
  for (size_t i = 0; i < knn_matches.size(); i++) {
    if (knn_matches[i].size() >= 2 &&
        knn_matches[i][0].distance <
            ratio_thresh * knn_matches[i][1].distance) {
      good_matches.push_back(knn_matches[i][0]);
    }
  }

  std::cout << "  Found " << good_matches.size()
            << " good matches after ratio test" << std::endl;
}

//------------------------------------------------------------------------------
// Draw matches between two images (with auto-resize for large images)
//------------------------------------------------------------------------------
void drawMatchesBetweenFrames(const cv::Mat &image1, const cv::Mat &image2,
                              const std::vector<cv::KeyPoint> &keypoints1,
                              const std::vector<cv::KeyPoint> &keypoints2,
                              const std::vector<cv::DMatch> &matches,
                              const std::string &windowTitle) {
  cv::Mat matchImage;
  cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, matchImage);

  // Resize if image is too large to fit on screen
  const int MAX_WIDTH = 1920;  // Max screen width
  const int MAX_HEIGHT = 1080; // Max screen height

  cv::Mat displayImage = matchImage;
  if (matchImage.cols > MAX_WIDTH || matchImage.rows > MAX_HEIGHT) {
    double scale = std::min(static_cast<double>(MAX_WIDTH) / matchImage.cols,
                            static_cast<double>(MAX_HEIGHT) / matchImage.rows);

    cv::resize(matchImage, displayImage, cv::Size(), scale, scale,
               cv::INTER_AREA);

    std::cout << "  Image resized for display: " << matchImage.cols << "x"
              << matchImage.rows << " → " << displayImage.cols << "x"
              << displayImage.rows << " (scale: " << scale << ")" << std::endl;
  }

  cv::namedWindow(windowTitle, cv::WINDOW_NORMAL);
  cv::imshow(windowTitle, displayImage);

  std::cout << "\nShowing matches: " << windowTitle << std::endl;
  std::cout << "Number of matches: " << matches.size() << std::endl;
  std::cout << "Press ESC to continue..." << std::endl;

  while (true) {
    int key = cv::waitKey(100);
    if (key == 27) { // ESC key
      break;
    }
  }

  cv::destroyWindow(windowTitle);
  cv::waitKey(1);
}

//------------------------------------------------------------------------------
// Template helpers for Rerun visualization
//------------------------------------------------------------------------------
template <typename T> rerun::Vec3D getRerunTranslationFromCvMat(cv::Mat t) {
  return rerun::Vec3D(t.at<T>(0, 0), t.at<T>(1, 0), t.at<T>(2, 0));
}

template <typename T> rerun::Mat3x3 getRerunRotationFromCvMat(cv::Mat R) {
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

//------------------------------------------------------------------------------
// Visualize keypoints in Rerun
//------------------------------------------------------------------------------
void logKeypointsToRerun(const rerun::RecordingStream &rec,
                         const std::string &entity_path,
                         const std::vector<cv::KeyPoint> &keypoints,
                         const cv::Mat &image,
                         const std::vector<bool> &is_inlier = {}) {

  // Log the image first
  rec.log(entity_path + "/image",
          rerun::Image::from_grayscale8(image,
                                        {static_cast<uint32_t>(image.cols),
                                         static_cast<uint32_t>(image.rows)}));

  // Prepare keypoint data
  std::vector<rerun::Position2D> positions;
  std::vector<rerun::Color> colors;
  std::vector<float> radii;

  for (size_t i = 0; i < keypoints.size(); i++) {
    positions.push_back({keypoints[i].pt.x, keypoints[i].pt.y});

    // Color based on inlier status if provided
    if (!is_inlier.empty() && i < is_inlier.size()) {
      if (is_inlier[i]) {
        colors.push_back(rerun::Color(0, 255, 0)); // Green for inliers
      } else {
        colors.push_back(rerun::Color(255, 0, 0)); // Red for outliers
      }
    } else {
      colors.push_back(rerun::Color(255, 255, 0)); // Yellow for all keypoints
    }

    // Size based on keypoint size
    radii.push_back(keypoints[i].size * 0.5f);
  }

  // Log keypoints as 2D points
  rec.log(entity_path + "/keypoints",
          rerun::Points2D(positions).with_radii(radii).with_colors(colors));
}

//------------------------------------------------------------------------------
// Visualize matches between two images in Rerun
//------------------------------------------------------------------------------
void logMatchesToRerun(const rerun::RecordingStream &rec,
                       const std::string &entity_path,
                       const std::vector<cv::KeyPoint> &keypoints1,
                       const std::vector<cv::KeyPoint> &keypoints2,
                       const std::vector<cv::DMatch> &matches,
                       const cv::Mat &image1, const cv::Mat &image2,
                       int img_idx1, int img_idx2) {

  // Create a combined image showing both views side by side
  cv::Mat combined_image;
  cv::hconcat(image1, image2, combined_image);

  rec.log(entity_path + "/combined_image",
          rerun::Image::from_grayscale8(
              combined_image, {static_cast<uint32_t>(combined_image.cols),
                               static_cast<uint32_t>(combined_image.rows)}));

  // Prepare match visualization as line segments
  std::vector<std::vector<rerun::Position2D>> line_strips;
  std::vector<rerun::Color> colors;

  int offset = image1.cols; // Offset for second image in combined view

  for (const auto &match : matches) {
    const cv::Point2f &pt1 = keypoints1[match.queryIdx].pt;
    const cv::Point2f &pt2 = keypoints2[match.trainIdx].pt;

    // Create line from point in image1 to point in image2 (with offset)
    std::vector<rerun::Position2D> line = {{pt1.x, pt1.y},
                                           {pt2.x + offset, pt2.y}};
    line_strips.push_back(line);

    // Color based on match quality (distance)
    if (match.distance < 50.0f) {
      colors.push_back(rerun::Color(0, 255, 0)); // Green for good matches
    } else if (match.distance < 100.0f) {
      colors.push_back(rerun::Color(255, 255, 0)); // Yellow for medium
    } else {
      colors.push_back(rerun::Color(255, 128, 0)); // Orange for weaker
    }
  }

  // Log the match lines
  rec.log(entity_path + "/matches",
          rerun::LineStrips2D(line_strips).with_colors(colors));

  // Also log keypoints in both images
  std::vector<rerun::Position2D> kp1_positions, kp2_positions;
  std::vector<float> kp_radii;

  for (const auto &match : matches) {
    kp1_positions.push_back(
        {keypoints1[match.queryIdx].pt.x, keypoints1[match.queryIdx].pt.y});
    kp2_positions.push_back({keypoints2[match.trainIdx].pt.x + offset,
                             keypoints2[match.trainIdx].pt.y});
    kp_radii.push_back(3.0f);
  }

  std::vector<rerun::Color> kp_colors(kp1_positions.size(),
                                      rerun::Color(255, 0, 0));

  rec.log(entity_path + "/keypoints_img1", rerun::Points2D(kp1_positions)
                                               .with_radii(kp_radii)
                                               .with_colors(kp_colors));

  rec.log(entity_path + "/keypoints_img2", rerun::Points2D(kp2_positions)
                                               .with_radii(kp_radii)
                                               .with_colors(kp_colors));

  std::cout << "  Logged " << matches.size()
            << " matches to Rerun: " << entity_path << std::endl;
}

//------------------------------------------------------------------------------
// Main incremental SfM pipeline for real data
//------------------------------------------------------------------------------
void realDataIncrementalSfM(const std::string &imageDir,
                            const std::string &cameraFile,
                            bool visualize_matches = true, bool use_sift = true,
                            float ratio_threshold = 0.7f) {

  std::cout << "\n" << std::string(80, '=') << std::endl;
  std::cout << "REAL DATA INCREMENTAL STRUCTURE FROM MOTION" << std::endl;
  std::cout << std::string(80, '=') << std::endl;
  std::cout << "Image directory: " << imageDir << std::endl;
  std::cout << "Camera file: " << cameraFile << std::endl;
  std::cout << std::string(80, '=') << "\n" << std::endl;

  // Load camera intrinsics
  cv::Mat K, distCoeffs;
  double k1, k2;
  if (!loadCameraIntrinsics(cameraFile, K, distCoeffs, k1, k2)) {
    return;
  }

  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  // Load images
  std::vector<std::string> filenames;
  std::vector<cv::Mat> images = loadImagesFromDirectory(imageDir, filenames);

  if (images.size() < 2) {
    std::cerr << "ERROR: Need at least 2 images for SfM!" << std::endl;
    return;
  }

  int N_CAMERAS = static_cast<int>(images.size());
  std::cout << "\nProcessing " << N_CAMERAS << " images" << std::endl;

  // Initialize Rerun
  std::cout << "\n🔧 Initializing Rerun viewer..." << std::endl;
  const auto rec = rerun::RecordingStream("real_data_incremental_SfM");
  rec.spawn().exit_on_failure();
  std::cout << "✅ Rerun viewer spawned successfully!" << std::endl;

  rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);
  std::cout << "✅ World coordinate system set (RIGHT_HAND_Y_DOWN)"
            << std::endl;

  // Log coordinate axes
  rec.log("world/xyz",
          rerun::Arrows3D::from_vectors(
              {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}})
              .with_colors({{255, 0, 0}, {0, 255, 0}, {0, 0, 255}}));
  std::cout << "✅ Coordinate axes logged" << std::endl;

  // Feature detection and matching
  std::vector<std::vector<cv::KeyPoint>> keypoints(N_CAMERAS);
  std::vector<std::vector<cv::DMatch>> all_matches;

  std::cout << "\n" << std::string(80, '=') << std::endl;
  std::cout << "FEATURE DETECTION AND MATCHING" << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  // First, detect features in all images and visualize
  std::cout << "Detecting features in all images..." << std::endl;
  cv::Ptr<cv::Feature2D> detector;
  if (use_sift) {
    detector = cv::SIFT::create(0, 3, 0.04, 10, 1.6);
  } else {
    detector = cv::ORB::create(2000);
  }

  for (int i = 0; i < N_CAMERAS; ++i) {
    cv::Mat descriptors;
    detector->detectAndCompute(images[i], cv::noArray(), keypoints[i],
                               descriptors);
    std::cout << "  Image " << i << " (" << filenames[i]
              << "): " << keypoints[i].size() << " keypoints" << std::endl;

    // Log keypoints to Rerun
    std::string kp_entity = "world/detection/image_" + std::to_string(i);
    logKeypointsToRerun(rec, kp_entity, keypoints[i], images[i]);
  }

  std::cout << "\nMatching features between consecutive images..." << std::endl;

  for (int i = 0; i < N_CAMERAS - 1; ++i) {
    std::cout << "\nProcessing image pair " << i << " <-> " << (i + 1)
              << std::endl;

    // Match features between consecutive images
    std::vector<cv::DMatch> matches;

    // Compute descriptors for both images
    cv::Mat descriptors1, descriptors2;
    detector->compute(images[i], keypoints[i], descriptors1);
    detector->compute(images[i + 1], keypoints[i + 1], descriptors2);

    // Match descriptors
    cv::Ptr<cv::DescriptorMatcher> matcher;
    if (use_sift) {
      matcher =
          cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    } else {
      matcher = cv::DescriptorMatcher::create(
          cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    }

    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

    // Apply Lowe's ratio test
    for (size_t k = 0; k < knn_matches.size(); k++) {
      if (knn_matches[k].size() >= 2 &&
          knn_matches[k][0].distance <
              ratio_threshold * knn_matches[k][1].distance) {
        matches.push_back(knn_matches[k][0]);
      }
    }

    std::cout << "  Found " << matches.size()
              << " good matches after ratio test" << std::endl;

    if (matches.size() < 8) {
      std::cerr << "ERROR: Not enough matches between images " << i << " and "
                << (i + 1) << " (need at least 8)" << std::endl;
      return;
    }

    all_matches.push_back(matches);

    // Log matches to Rerun
    std::string match_entity =
        "world/matches/pair_" + std::to_string(i) + "_" + std::to_string(i + 1);
    logMatchesToRerun(rec, match_entity, keypoints[i], keypoints[i + 1],
                      matches, images[i], images[i + 1], i, i + 1);

    if (visualize_matches) {
      std::string windowTitle =
          "Matches: " + filenames[i] + " <-> " + filenames[i + 1];
      drawMatchesBetweenFrames(images[i], images[i + 1], keypoints[i],
                               keypoints[i + 1], matches, windowTitle);
    }
  }

  // Initialize camera poses
  std::vector<CameraExtrinsics> cameras(N_CAMERAS);
  cameras[0].R = cv::Mat::eye(3, 3, CV_64F);
  cameras[0].t = cv::Mat::zeros(3, 1, CV_64F);

  std::vector<double> cameraParams(6 * N_CAMERAS, 0.0);
  RtToAngleAxisAndTranslate(cameras[0].R, cameras[0].t, &cameraParams[0]);

  // Track structure
  std::vector<cv::Point3f> globalPoints3D;
  std::vector<Observation> observations;
  std::vector<Track> tracks;
  std::map<std::pair<int, int>, int> featureToTrack;

  std::cout << "\n" << std::string(80, '=') << std::endl;
  std::cout << "INCREMENTAL RECONSTRUCTION" << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  // Incremental reconstruction
  for (size_t i = 1; i < N_CAMERAS; ++i) {
    std::cout << "\n--- Adding camera " << i << " ---" << std::endl;

    std::vector<cv::Point2f> pts_im1, pts_i;
    for (const auto &match : all_matches[i - 1]) {
      pts_im1.push_back(keypoints[i - 1][match.queryIdx].pt);
      pts_i.push_back(keypoints[i][match.trainIdx].pt);
    }

    std::cout << "  Total matches before filtering: " << pts_im1.size()
              << std::endl;

    // Estimate essential matrix and recover pose
    cv::Mat mask;
    cv::Mat E =
        cv::findEssentialMat(pts_im1, pts_i, K, cv::RANSAC, 0.999, 1.0, mask);

    cv::Mat R_im1_to_i, t_im1_to_i;
    int inliers =
        cv::recoverPose(E, pts_im1, pts_i, K, R_im1_to_i, t_im1_to_i, mask);

    // Filter matches to keep only inliers (good correspondences)
    std::vector<cv::DMatch> filtered_matches;
    std::vector<cv::Point2f> filtered_pts_im1, filtered_pts_i;

    for (size_t k = 0; k < all_matches[i - 1].size(); k++) {
      if (mask.at<uchar>(k) != 0) { // Check if this match is an inlier
        filtered_matches.push_back(all_matches[i - 1][k]);
        filtered_pts_im1.push_back(pts_im1[k]);
        filtered_pts_i.push_back(pts_i[k]);
      }
    }

    // Log inlier matches to Rerun
    std::string inlier_entity = "world/inlier_matches/pair_" +
                                std::to_string(i - 1) + "_" + std::to_string(i);
    logMatchesToRerun(rec, inlier_entity, keypoints[i - 1], keypoints[i],
                      filtered_matches, images[i - 1], images[i], i - 1, i);

    // Replace with filtered data
    all_matches[i - 1] = filtered_matches;
    pts_im1 = filtered_pts_im1;
    pts_i = filtered_pts_i;

    std::cout << "  Inliers after RANSAC filtering: " << pts_im1.size() << " ("
              << (100.0 * pts_im1.size() / mask.rows) << "%)" << std::endl;

    if (pts_im1.size() < 8) {
      std::cerr << "ERROR: Not enough inlier matches after filtering (need at "
                   "least 8)"
                << std::endl;
      return;
    }

    // Ensure translation is column vector
    if (t_im1_to_i.rows == 1 && t_im1_to_i.cols == 3) {
      t_im1_to_i = t_im1_to_i.t();
    }

    // Chain transformations
    cv::Mat R_0_to_im1 = cameras[i - 1].R;
    cv::Mat t_0_to_im1 = cameras[i - 1].t;

    cv::Mat R_0_to_i = R_im1_to_i * R_0_to_im1;
    cv::Mat t_0_to_i = R_im1_to_i * t_0_to_im1 + t_im1_to_i;

    cameras[i].R = R_0_to_i.clone();
    cameras[i].t = t_0_to_i.clone();

    RtToAngleAxisAndTranslate(cameras[i].R, cameras[i].t, &cameraParams[i * 6]);

    std::cout << "Camera " << i << " pose estimated" << std::endl;
    std::cout << "  R:\n" << cameras[i].R << std::endl;
    std::cout << "  t: " << cameras[i].t.t() << std::endl;

    // Build projection matrices and triangulate
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

    // Build tracks
    int new_tracks = 0;
    int extended_tracks = 0;

    for (size_t k = 0; k < all_matches[i - 1].size(); k++) {
      const cv::DMatch &match = all_matches[i - 1][k];
      int feature_im1 = match.queryIdx;
      int feature_i = match.trainIdx;

      std::pair<int, int> key_im1(i - 1, feature_im1);
      int track_id = -1;
      bool is_new_track = false;

      if (featureToTrack.find(key_im1) != featureToTrack.end()) {
        // Extend existing track
        track_id = featureToTrack[key_im1];
        extended_tracks++;
      } else {
        // Create new track
        track_id = static_cast<int>(tracks.size());
        tracks.emplace_back();
        is_new_track = true;
        new_tracks++;

        tracks[track_id].addObservation(i - 1, feature_im1, pts_im1[k]);
        featureToTrack[key_im1] = track_id;
      }

      // Add observation for current camera
      std::pair<int, int> key_i(i, feature_i);
      tracks[track_id].addObservation(i, feature_i, pts_i[k]);
      featureToTrack[key_i] = track_id;

      // Triangulate if new track
      if (is_new_track) {
        int point3D_idx = static_cast<int>(globalPoints3D.size());
        globalPoints3D.push_back(newPoints_in_cam0[k]);
        tracks[track_id].point3D_idx = point3D_idx;
      }
    }

    std::cout << "  Created " << new_tracks << " new tracks" << std::endl;
    std::cout << "  Extended " << extended_tracks << " existing tracks"
              << std::endl;
    std::cout << "  Total tracks: " << tracks.size() << std::endl;
    std::cout << "  Total 3D points: " << globalPoints3D.size() << std::endl;
  }

  // Build observations for bundle adjustment
  std::cout << "\n" << std::string(80, '=') << std::endl;
  std::cout << "BUILDING OBSERVATIONS FOR BUNDLE ADJUSTMENT" << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  for (size_t track_id = 0; track_id < tracks.size(); track_id++) {
    const Track &track = tracks[track_id];

    if (track.point3D_idx < 0) {
      continue;
    }

    for (const auto &feat_obs : track.observations) {
      Observation obs;
      obs.camera_idx = feat_obs.camera_idx;
      obs.point_idx = track.point3D_idx;

      // Convert to Snavely normalized coordinates
      obs.x = -(feat_obs.pixel.x - cx) / fx;
      obs.y = -(feat_obs.pixel.y - cy) / fy;

      observations.push_back(obs);
    }
  }

  std::cout << "Total observations: " << observations.size() << std::endl;
  std::cout << "Total 3D points: " << globalPoints3D.size() << std::endl;

  // Prepare point parameters
  std::vector<double> pointParams(3 * globalPoints3D.size());
  for (size_t i = 0; i < globalPoints3D.size(); i++) {
    pointParams[3 * i + 0] = globalPoints3D[i].x;
    pointParams[3 * i + 1] = globalPoints3D[i].y;
    pointParams[3 * i + 2] = globalPoints3D[i].z;
  }

  // Bundle adjustment
  std::cout << "\n" << std::string(80, '=') << std::endl;
  std::cout << "BUNDLE ADJUSTMENT" << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  ceres::Problem problem;

  int id = 0;
  for (const Observation &obs : observations) {
    ceres::CostFunction *costFunc = SnavelyReprojectionErrorFixedCamera::Create(
        obs.x, obs.y, 1.0, k1, k2, id);
    id++;

    double *cameraParamsPtr = &cameraParams[obs.camera_idx * 6];
    double *pointPtr = &pointParams[3 * obs.point_idx];

    problem.AddResidualBlock(costFunc, nullptr, cameraParamsPtr, pointPtr);
  }

  // Fix camera 0
  problem.SetParameterBlockConstant(&cameraParams[0]);

  std::cout << "Camera 0 is FIXED (world reference frame)" << std::endl;
  std::cout << "Optimizing " << (N_CAMERAS - 1) << " camera poses and "
            << globalPoints3D.size() << " 3D points" << std::endl;

  // Solve
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << "\n" << summary.FullReport() << std::endl;

  // Visualize results in Rerun
  std::cout << "\n" << std::string(80, '=') << std::endl;
  std::cout << "VISUALIZATION" << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  // Log optimized 3D points
  std::vector<rerun::Position3D> point3d_positions;
  for (size_t i = 0; i < pointParams.size(); i += 3) {
    point3d_positions.push_back({static_cast<float>(pointParams[i + 0]),
                                 static_cast<float>(pointParams[i + 1]),
                                 static_cast<float>(pointParams[i + 2])});
  }

  std::vector<rerun::Color> colors(point3d_positions.size(),
                                   rerun::Color(255, 255, 0));

  std::cout << "\n📍 Logging " << point3d_positions.size()
            << " 3D points to Rerun..." << std::endl;

  rec.log("world/reconstructed_points",
          rerun::Points3D(point3d_positions)
              .with_radii(std::vector<float>(point3d_positions.size(), 0.02))
              .with_colors(colors));

  std::cout << "✅ 3D points logged to: world/reconstructed_points"
            << std::endl;

  // Log camera poses
  for (int cam_id = 0; cam_id < N_CAMERAS; cam_id++) {
    cv::Mat optimized_angleaxis =
        (cv::Mat_<double>(3, 1) << cameraParams[cam_id * 6 + 0],
         cameraParams[cam_id * 6 + 1], cameraParams[cam_id * 6 + 2]);

    cv::Mat optimized_t =
        (cv::Mat_<double>(3, 1) << cameraParams[cam_id * 6 + 3],
         cameraParams[cam_id * 6 + 4], cameraParams[cam_id * 6 + 5]);

    cv::Mat optimized_R;
    cv::Rodrigues(optimized_angleaxis, optimized_R);

    cv::Mat R_cam_in_world = optimized_R.t();
    cv::Mat t_cam_in_world = -optimized_R.t() * optimized_t;

    rerun::Vec3D rr_translation =
        getRerunTranslationFromCvMat<double>(t_cam_in_world);
    rerun::Mat3x3 rr_rotation =
        getRerunRotationFromCvMat<double>(R_cam_in_world);

    // ========================================================================
    // CAMERA RAY SETUP (based on verified working Python example)
    // ========================================================================
    // Build intrinsic matrix for camera rays (column-major format)
    rerun::datatypes::Mat3x3 image_from_camera{{
        static_cast<float>(fx), // [0,0]
        0.0f,                   // [1,0]
        0.0f,                   // [2,0]
        0.0f,                   // [0,1]
        static_cast<float>(fy), // [1,1]
        0.0f,                   // [2,1]
        static_cast<float>(cx), // [0,2]
        static_cast<float>(cy), // [1,2]
        1.0f                    // [2,2]
    }};

    std::string camera_name = "world/cameras/cam" + std::to_string(cam_id);

    // Transform to camera path
    rec.log(camera_name, rerun::Transform3D(rr_translation, rr_rotation));

    // Pinhole to IMAGE path (enables camera rays!)
    rec.log(camera_name + "/image",
            rerun::Pinhole(image_from_camera)
                .with_resolution({static_cast<float>(images[cam_id].cols),
                                  static_cast<float>(images[cam_id].rows)}));

    // Image to SAME path as Pinhole
    std::cout << "  Logging camera " << cam_id
              << " image: " << images[cam_id].cols << "x" << images[cam_id].rows
              << " to " << camera_name + "/image" << std::endl;

    rec.log(camera_name + "/image",
            rerun::Image::from_grayscale8(
                images[cam_id], {static_cast<uint32_t>(images[cam_id].cols),
                                 static_cast<uint32_t>(images[cam_id].rows)}));
  }

  std::cout << "\n✅ All " << N_CAMERAS << " cameras logged with images"
            << std::endl;

  std::cout << "Logged " << N_CAMERAS << " camera poses to Rerun" << std::endl;

  std::cout << "\n🎯 CAMERA RAYS ENABLED!" << std::endl;
  std::cout << "   Hover over any camera image to see 3D rays" << std::endl;
  std::cout << "   Images at: world/cameras/cam[0,1,2,...]/image" << std::endl;

  // List all entities logged
  std::cout << "\n📊 Rerun Entities Logged:" << std::endl;
  std::cout << "   • world/xyz (coordinate axes)" << std::endl;
  std::cout << "   • world/reconstructed_points (" << globalPoints3D.size()
            << " points)" << std::endl;
  for (int i = 0; i < N_CAMERAS; i++) {
    std::cout << "   • world/cameras/cam" << i << " (Transform3D)" << std::endl;
    std::cout << "   • world/cameras/cam" << i << "/image (Pinhole + Image)"
              << std::endl;
  }

  std::cout << "\n" << std::string(80, '=') << std::endl;
  std::cout << "RECONSTRUCTION COMPLETE" << std::endl;
  std::cout << std::string(80, '=') << std::endl;
  std::cout << "Initial cost: " << summary.initial_cost << std::endl;
  std::cout << "Final cost: " << summary.final_cost << std::endl;
  std::cout << "Cameras: " << N_CAMERAS << std::endl;
  std::cout << "3D points: " << globalPoints3D.size() << std::endl;
  std::cout << "Observations: " << observations.size() << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  // Keep Rerun viewer open
  std::cout << "\n✨ Rerun viewer is open!" << std::endl;
  std::cout << "   Explore the reconstruction in 3D" << std::endl;
  std::cout << "   Hover over camera images to see rays" << std::endl;
  std::cout << "\nPress Enter to exit and close Rerun..." << std::endl;
  std::cin.get();
}

//------------------------------------------------------------------------------
// Main function
//------------------------------------------------------------------------------
int main(int argc, char **argv) {
  ArgumentsParser parser(argc, argv);

  // Check for help flag
  if (parser.argExists("-h") || parser.argExists("--help")) {
    std::cout << "Usage: " << argv[0] << " [OPTIONS]" << std::endl;
    std::cout << "\nOptions:" << std::endl;
    std::cout << "  -i, --images DIR      Directory containing input images "
                 "(required)"
              << std::endl;
    std::cout << "  -c, --camera FILE     Camera intrinsics file (YAML format, "
                 "required)"
              << std::endl;
    std::cout
        << "  --ratio FLOAT         Lowe's ratio test threshold (default: 0.7)"
        << std::endl;
    std::cout << "                        Lower = stricter filtering (0.5-0.9)"
              << std::endl;
    std::cout << "  --no-viz              Disable match visualization"
              << std::endl;
    std::cout << "  --use-orb             Use ORB instead of SIFT for feature "
                 "detection"
              << std::endl;
    std::cout << "  -h, --help            Show this help message" << std::endl;
    std::cout << "\nExample:" << std::endl;
    std::cout << "  " << argv[0] << " -i /path/to/images -c camera.yaml"
              << std::endl;
    std::cout << "\nCamera file format (YAML):" << std::endl;
    std::cout << "  camera_matrix: !!opencv-matrix" << std::endl;
    std::cout << "    rows: 3" << std::endl;
    std::cout << "    cols: 3" << std::endl;
    std::cout << "    data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]" << std::endl;
    std::cout << "  distortion_coefficients: !!opencv-matrix" << std::endl;
    std::cout << "    rows: 5" << std::endl;
    std::cout << "    cols: 1" << std::endl;
    std::cout << "    data: [k1, k2, p1, p2, k3]" << std::endl;
    return 0;
  }

  // Get arguments
  std::string imageDir = parser.getArg("-i");
  if (imageDir.empty()) {
    imageDir = parser.getArg("--images");
  }

  std::string cameraFile = parser.getArg("-c");
  if (cameraFile.empty()) {
    cameraFile = parser.getArg("--camera");
  }

  if (imageDir.empty() || cameraFile.empty()) {
    std::cerr << "ERROR: Image directory and camera file are required!"
              << std::endl;
    std::cerr << "Use -h or --help for usage information." << std::endl;
    return 1;
  }

  bool visualize_matches = !parser.argExists("--no-viz");
  bool use_sift = !parser.argExists("--use-orb");

  // Get ratio threshold
  float ratio_threshold = 0.7f; // default
  std::string ratio_str = parser.getArg("--ratio");
  if (!ratio_str.empty()) {
    try {
      ratio_threshold = std::stof(ratio_str);
      if (ratio_threshold < 0.0f || ratio_threshold > 1.0f) {
        std::cerr << "ERROR: Ratio threshold must be between 0.0 and 1.0"
                  << std::endl;
        return 1;
      }
    } catch (const std::exception &e) {
      std::cerr << "ERROR: Invalid ratio threshold value: " << ratio_str
                << std::endl;
      return 1;
    }
  }

  std::cout << "\n=== Configuration ===" << std::endl;
  std::cout << "Feature detector: " << (use_sift ? "SIFT" : "ORB") << std::endl;
  std::cout << "Ratio threshold: " << ratio_threshold << std::endl;
  std::cout << "Visualize matches: " << (visualize_matches ? "Yes" : "No")
            << std::endl;
  std::cout << "=====================\n" << std::endl;

  // Run SfM
  realDataIncrementalSfM(imageDir, cameraFile, visualize_matches, use_sift,
                         ratio_threshold);

  return 0;
}
