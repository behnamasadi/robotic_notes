#include "../collection_adapters.hpp"
#include "../snavely_reprojection_error.hpp"
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
                                    const std::vector<cv::DMatch> &matches,
                                    int camera_pair_index = 0) {
  cv::Mat matchImage;
  cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, matchImage);

  // Use unique window title for each camera pair
  std::string windowTitle = "Matches: Cam" + std::to_string(camera_pair_index) +
                            " <-> Cam" + std::to_string(camera_pair_index + 1) +
                            " (Press ESC to close)";

  cv::namedWindow(windowTitle, cv::WINDOW_NORMAL);
  cv::imshow(windowTitle, matchImage);

  std::cout << "\n=== Feature Matches Visualization ===" << std::endl;
  std::cout << "Showing matches between Camera " << camera_pair_index
            << " and Camera " << (camera_pair_index + 1) << std::endl;
  std::cout << "Number of matches: " << matches.size() << std::endl;
  std::cout << "Press ESC key to close this window and continue..."
            << std::endl;

  // Wait for ESC key to proceed to the next pair
  while (true) {
    int key = cv::waitKey(100); // Check every 100ms
    if (key == 27) {            // ESC key
      break;
    }
  }

  // Properly destroy the window
  cv::destroyWindow(windowTitle);
  cv::waitKey(1); // Give time for window to actually close

  std::cout << "Window closed. Continuing..." << std::endl;
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

  cv::Mat rodrigues;
  cv::Rodrigues(R, rodrigues);
  //  std::cout << "Rodrigues:\n" << rodrigues << std::endl;

  // ============================================================================
  // BUG FIX: cv::Rodrigues returns a 3×1 column vector (not 1×3 row vector)
  // ============================================================================
  // cv::Rodrigues converts 3×3 rotation matrix → 3×1 angle-axis vector
  // Access as (row, col): (0,0), (1,0), (2,0) NOT (0,0), (0,1), (0,2)
  // ============================================================================

  // Verify dimensions
  if (rodrigues.rows == 3 && rodrigues.cols == 1) {
    // Column vector (expected case)
    camera_params[0] = rodrigues.at<double>(0, 0); // rodrigues[0]
    camera_params[1] = rodrigues.at<double>(1, 0); // rodrigues[1]
    camera_params[2] = rodrigues.at<double>(2, 0); // rodrigues[2]
  } else if (rodrigues.rows == 1 && rodrigues.cols == 3) {
    // Row vector (rare but handle it)
    camera_params[0] = rodrigues.at<double>(0, 0);
    camera_params[1] = rodrigues.at<double>(0, 1);
    camera_params[2] = rodrigues.at<double>(0, 2);
  } else {
    std::cerr << "ERROR: Rodrigues vector has unexpected dimensions: "
              << rodrigues.rows << "x" << rodrigues.cols << std::endl;
    std::cerr << "Expected 3x1 or 1x3" << std::endl;
  }

  // 2) translation => camera_params[3..5]
  camera_params[3] = t.at<double>(0, 0);
  camera_params[4] = t.at<double>(1, 0);
  camera_params[5] = t.at<double>(2, 0);
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
  double x, y;    // 2D feature coords (Snavely normalized)
};

// ============================================================================
// MULTI-VIEW TRACK STRUCTURE (Used in Production SfM Systems)
// ============================================================================
// A "Track" represents a single physical 3D point that has been observed
// in multiple camera views. This is the standard approach used in:
// - COLMAP: https://colmap.github.io/
// - OpenMVG: https://github.com/openMVG/openMVG
// - Bundler: http://www.cs.cornell.edu/~snavely/bundler/
//
// Key concepts:
// 1. Each track has a unique 3D point (stored in globalPoints3D)
// 2. Each track maintains observations from multiple cameras
// 3. Observations are identified by (camera_idx, feature_idx) pairs
// 4. Tracks grow as points are seen in additional views
// ============================================================================

struct FeatureObservation {
  int camera_idx;    // Which camera sees this feature
  int feature_idx;   // Index of the keypoint in that camera's keypoint list
  cv::Point2f pixel; // Original pixel coordinates (u, v)

  FeatureObservation(int cam, int feat, const cv::Point2f &px)
      : camera_idx(cam), feature_idx(feat), pixel(px) {}
};

struct Track {
  int point3D_idx; // Index in globalPoints3D (-1 if not triangulated yet)
  std::vector<FeatureObservation>
      observations; // All cameras that see this point

  Track() : point3D_idx(-1) {}

  // Add a new observation to this track
  void addObservation(int camera_idx, int feature_idx,
                      const cv::Point2f &pixel) {
    observations.emplace_back(camera_idx, feature_idx, pixel);
  }

  // Check if this track is observed by a specific camera
  bool isObservedBy(int camera_idx) const {
    for (const auto &obs : observations) {
      if (obs.camera_idx == camera_idx)
        return true;
    }
    return false;
  }

  // Get feature index in a specific camera (returns -1 if not observed)
  int getFeatureIdx(int camera_idx) const {
    for (const auto &obs : observations) {
      if (obs.camera_idx == camera_idx)
        return obs.feature_idx;
    }
    return -1;
  }

  // Get number of cameras observing this track
  int numObservations() const { return static_cast<int>(observations.size()); }
};

// ============================================================================
// PRINT RECONSTRUCTION QUALITY SUMMARY
// ============================================================================
void printReconstructionSummary(int num_cameras, size_t num_points,
                                size_t num_observations, double ba_initial_cost,
                                double ba_final_cost, int ba_iterations,
                                double avg_rotation_error_deg,
                                double avg_translation_error,
                                double points_rms_error,
                                double points_max_error) {

  std::cout << "\n" << std::string(80, '=') << std::endl;
  std::cout << "RECONSTRUCTION QUALITY SUMMARY" << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  std::cout
      << "\n┌─────────────────────────────────────┬──────────────────────────┐"
      << std::endl;
  std::cout
      << "│ Metric                              │ Value                    │"
      << std::endl;
  std::cout
      << "├─────────────────────────────────────┼──────────────────────────┤"
      << std::endl;

  char buffer[100];

  sprintf(buffer, "│ Number of cameras                   │ %-24d │",
          num_cameras);
  std::cout << buffer << std::endl;

  sprintf(buffer, "│ Number of 3D points                 │ %-24zu │",
          num_points);
  std::cout << buffer << std::endl;

  sprintf(buffer, "│ Total observations                  │ %-24zu │",
          num_observations);
  std::cout << buffer << std::endl;

  std::cout
      << "├─────────────────────────────────────┼──────────────────────────┤"
      << std::endl;

  sprintf(buffer, "│ BA initial cost                     │ %-24.6e │",
          ba_initial_cost);
  std::cout << buffer << std::endl;

  sprintf(buffer, "│ BA final cost                       │ %-24.6e │",
          ba_final_cost);
  std::cout << buffer << std::endl;

  sprintf(buffer, "│ BA iterations                       │ %-24d │",
          ba_iterations);
  std::cout << buffer << std::endl;

  std::cout
      << "├─────────────────────────────────────┼──────────────────────────┤"
      << std::endl;

  sprintf(buffer,
          "│ Avg camera rotation error           │ %.4f degrees         │",
          avg_rotation_error_deg);
  std::cout << buffer << std::endl;

  sprintf(buffer,
          "│ Avg camera translation error        │ %.6f units          │",
          avg_translation_error);
  std::cout << buffer << std::endl;

  std::cout
      << "├─────────────────────────────────────┼──────────────────────────┤"
      << std::endl;

  sprintf(buffer,
          "│ 3D points RMS error                 │ %.6f units          │",
          points_rms_error);
  std::cout << buffer << std::endl;

  sprintf(buffer,
          "│ 3D points max error                 │ %.6f units          │",
          points_max_error);
  std::cout << buffer << std::endl;

  std::cout
      << "└─────────────────────────────────────┴──────────────────────────┘"
      << std::endl;

  // Quality assessment
  std::cout << "\n✅ RECONSTRUCTION QUALITY: ";
  if (points_rms_error < 0.01 && avg_rotation_error_deg < 1.0) {
    std::cout << "EXCELLENT (Near-perfect reconstruction!)" << std::endl;
  } else if (points_rms_error < 0.1 && avg_rotation_error_deg < 5.0) {
    std::cout << "GOOD" << std::endl;
  } else if (points_rms_error < 1.0 && avg_rotation_error_deg < 10.0) {
    std::cout << "ACCEPTABLE" << std::endl;
  } else {
    std::cout << "POOR (Check your data and parameters)" << std::endl;
  }

  std::cout << "\n" << std::string(80, '=') << std::endl;

  // Visualization guide
  std::cout << "\n📊 Rerun Visualization Layers:" << std::endl;
  std::cout << "  CAMERAS (all in Camera 0 reference frame):" << std::endl;
  std::cout << "    • world/ground_truth/cam[0,1,2]          - Ground truth "
               "camera poses"
            << std::endl;
  std::cout << "    • world/ground_truth/cam[0,1,2]/image    - Rendered images "
               "with projected points"
            << std::endl;
  std::cout << "    • world/optimized/cam[0,1,2]             - Optimized "
               "camera poses (after BA)"
            << std::endl;
  std::cout << "  POINTS (all in Camera 0 reference frame):" << std::endl;
  std::cout
      << "    • world/points_ground_truth (gray)       - Ground truth ellipsoid"
      << std::endl;
  std::cout
      << "    • world/new_triangulated_points_*        - Initial triangulation"
      << std::endl;
  std::cout << "    • world/optimized_points_after_BA        - Final optimized "
               "structure (yellow)"
            << std::endl;
  std::cout << "\n  ⚠️  IMPORTANT: All visualizations are in Camera 0's "
               "reference frame"
            << std::endl;
  std::cout << "     - Ground truth cameras/points transformed from original "
               "world frame"
            << std::endl;
  std::cout << "     - Optimized cameras/points are naturally in Camera 0 frame"
            << std::endl;
  std::cout << "     - Both should now align perfectly (within scale ambiguity)"
            << std::endl;

  std::cout << "\n🎯 In Rerun, ground truth (green) and optimized (blue) "
               "cameras should overlap!"
            << std::endl;
  std::cout << "   Yellow and gray points should also overlap (within ~7% "
               "scale difference)"
            << std::endl;
}

// ============================================================================
// INCREMENTAL STRUCTURE FROM MOTION (SfM) WITH TRACK-BASED APPROACH
// ============================================================================
//
// This function implements a production-grade incremental SfM pipeline using
// a track-based data structure, following approaches used in:
//   - COLMAP: https://colmap.github.io/
//   - OpenMVG: https://github.com/openMVG/openMVG
//   - Bundler: http://www.cs.cornell.edu/~snavely/bundler/
//
// KEY CONCEPTS:
//
// 1. TRACK: A track represents a single physical 3D point observed across
//    multiple camera views. Each track contains:
//    - A unique 3D point coordinate (once triangulated)
//    - A list of 2D feature observations from different cameras
//
// 2. PARTIAL VISIBILITY: Unlike naive approaches that assume all points are
//    visible in all cameras, this handles realistic scenarios where:
//    - Some points are only visible in a subset of cameras
//    - New points can appear in later camera pairs
//    - Points can be lost as cameras move
//
// 3. INCREMENTAL ADDITION: Cameras are added one at a time:
//    - Start with Camera 0 as world reference (R=I, t=0)
//    - For each new camera i:
//        a. Compute relative pose from camera i-1 to i (Essential Matrix)
//        b. Chain transformations to get global pose w.r.t. Camera 0
//        c. Triangulate 3D points from matched features
//        d. Build/extend tracks with new observations
//
// 4. TRACK MANAGEMENT:
//    - featureToTrack: Maps (camera_idx, feature_idx) → track_id
//    - When processing matches between camera i-1 and i:
//        * If camera i-1's feature already has a track: EXTEND that track
//        * If camera i-1's feature is new: CREATE new track
//
// 5. BUNDLE ADJUSTMENT: After all cameras are added, optimize:
//    - Camera poses: {R_i, t_i} for i=1,2,...,N-1 (Camera 0 is fixed)
//    - 3D points: {P_j} for j=1,2,...,M
//    - Minimize reprojection error across all observations
//
// ADVANTAGES OF THIS APPROACH:
// ✅ Handles partial visibility (real-world scenarios)
// ✅ No duplicate 3D points
// ✅ Consistent track identification across views
// ✅ Scales to arbitrary number of cameras
// ✅ Matches production SfM systems
//
// ============================================================================
void virtualCamIncrementalSfMFixedCam() {

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

  // ===== SCALE AMBIGUITY IN SfM =====
  // SfM reconstruction has inherent scale ambiguity: we can only recover
  // geometry up to an arbitrary scale factor. To properly demonstrate this,
  // BOTH cameras AND 3D points must be scaled together.
  //
  // Scaled by 1.5x to demonstrate scale ambiguity:
  // Original: txCam1=0.9, tyCam1=0.1, tzCam1=0.3, txCam2=1.6, tyCam2=-0.1,
  // tzCam2=0.4 Original ellipsoid: (-1.5, 0, -4)
  txCam0 = 0.0;
  tyCam0 = 0.0;
  tzCam0 = 0.0;

  txCam1 = 1.35;  // 0.9 * 1.5
  tyCam1 = +0.15; // 0.1 * 1.5
  tzCam1 = +0.45; // 0.3 * 1.5

  txCam2 = +2.4;  // 1.6 * 1.5
  tyCam2 = -0.15; // -0.1 * 1.5
  tzCam2 = +0.6;  // 0.4 * 1.5

  // Note: Ellipsoid center below is also scaled by 1.5x: (-2.25, 0, -6)
  // This demonstrates SfM scale ambiguity - same images, different absolute
  // scale!

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

  float ellipsoidCenterX = -2.25; // -1.5 * 1.5 (scaled to match camera scaling)
  float ellipsoidCenterY = 0;     // 0 * 1.5
  float ellipsoidCenterZ = -6;    // -4 * 1.5
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

  // ============================================================================
  // TRANSFORM GROUND TRUTH 3D POINTS TO CAMERA 0 REFERENCE FRAME
  // ============================================================================
  // Similar to cameras, the ground truth points are in the "original world"
  // frame. Transform them to Camera 0's frame to match the reconstruction.
  // ============================================================================

  float x, y, z;

  for (std::size_t i = 0; i < objectPointsInWorldCoordinate.size(); i++) {
    // Transform point from original world to Camera 0 frame
    cv::Mat pt_world =
        (cv::Mat_<double>(3, 1) << objectPointsInWorldCoordinate[i][0],
         objectPointsInWorldCoordinate[i][1],
         objectPointsInWorldCoordinate[i][2]);

    cv::Mat pt_cam0 = rotation_Cam0_in_world.t() * (pt_world - t_Cam0_in_world);

    x = static_cast<float>(pt_cam0.at<double>(0, 0));
    y = static_cast<float>(pt_cam0.at<double>(1, 0));
    z = static_cast<float>(pt_cam0.at<double>(2, 0));

    point3d_positions.push_back({x, y, z});
    point_sizes.push_back(0.05);
  }

  rec.log("world/points_ground_truth",
          rerun::Points3D(point3d_positions).with_radii(point_sizes));

  std::cout << "Ground truth 3D points logged to Rerun (transformed to Camera "
               "0 frame)"
            << std::endl;

  // ============================================================================
  // VISUALIZE GROUND TRUTH CAMERAS RELATIVE TO CAMERA 0
  // ============================================================================
  // IMPORTANT: Transform ground truth cameras to Camera 0's reference frame
  // so they align with the optimized cameras for proper comparison in Rerun.
  //
  // Original ground truth is in the "original world" frame where Camera 0 has
  // a rotation. But our optimization treats Camera 0 AS the world (R=I, t=0).
  //
  // To visualize both in the same frame, we transform:
  //   R_cam_relative = R_cam0^T * R_cam
  //   t_cam_relative = R_cam0^T * (t_cam - t_cam0)
  // ============================================================================

  std::cout
      << "\nTransforming ground truth cameras to Camera 0 reference frame..."
      << std::endl;

  // Transform all cameras relative to Camera 0
  std::vector<cv::Mat> gt_R_relative(3);
  std::vector<cv::Mat> gt_t_relative(3);

  // Camera 0: Identity in its own frame
  gt_R_relative[0] = cv::Mat::eye(3, 3, CV_64F);
  gt_t_relative[0] = cv::Mat::zeros(3, 1, CV_64F);

  // Camera 1: Relative to Camera 0
  gt_R_relative[1] = rotation_Cam0_in_world.t() * rotation_Cam1_in_world;
  gt_t_relative[1] =
      rotation_Cam0_in_world.t() * (t_Cam1_in_world - t_Cam0_in_world);

  // Camera 2: Relative to Camera 0
  gt_R_relative[2] = rotation_Cam0_in_world.t() * rotation_Cam2_in_world;
  gt_t_relative[2] =
      rotation_Cam0_in_world.t() * (t_Cam2_in_world - t_Cam0_in_world);

  std::cout << "Ground truth cameras transformed to match optimization "
               "reference frame."
            << std::endl;

  // Convert to Rerun format
  rerun::Vec3D rr_translation_cam0_gt =
      getRerunTranslationFromCvMat<double>(gt_t_relative[0]);
  rerun::Vec3D rr_translation_cam1_gt =
      getRerunTranslationFromCvMat<double>(gt_t_relative[1]);
  rerun::Vec3D rr_translation_cam2_gt =
      getRerunTranslationFromCvMat<double>(gt_t_relative[2]);

  rerun::Mat3x3 rr_rotation_matrix_cam0_gt =
      getRerunRotationFromCvMat<double>(gt_R_relative[0]);
  rerun::Mat3x3 rr_rotation_matrix_cam1_gt =
      getRerunRotationFromCvMat<double>(gt_R_relative[1]);
  rerun::Mat3x3 rr_rotation_matrix_cam2_gt =
      getRerunRotationFromCvMat<double>(gt_R_relative[2]);

  std::string camera_name_cam0_gt = "world/ground_truth/cam0";
  std::string camera_name_cam1_gt = "world/ground_truth/cam1";
  std::string camera_name_cam2_gt = "world/ground_truth/cam2";

  // Camera 0 - Ground Truth (now at identity to match optimization)
  rec.log(camera_name_cam0_gt,
          rerun::Pinhole::from_focal_length_and_resolution(
              {float(focalLength * mx), float(focalLength * my)},
              {float(numberOfPixelInWidth), float(numberOfPixelInHeight)}));
  rec.log(camera_name_cam0_gt, rerun::Transform3D(rr_translation_cam0_gt,
                                                  rr_rotation_matrix_cam0_gt));

  // Camera 1 - Ground Truth (relative to Camera 0)
  rec.log(camera_name_cam1_gt,
          rerun::Pinhole::from_focal_length_and_resolution(
              {float(focalLength * mx), float(focalLength * my)},
              {float(numberOfPixelInWidth), float(numberOfPixelInHeight)}));
  rec.log(camera_name_cam1_gt, rerun::Transform3D(rr_translation_cam1_gt,
                                                  rr_rotation_matrix_cam1_gt));

  // Camera 2 - Ground Truth (relative to Camera 0)
  rec.log(camera_name_cam2_gt,
          rerun::Pinhole::from_focal_length_and_resolution(
              {float(focalLength * mx), float(focalLength * my)},
              {float(numberOfPixelInWidth), float(numberOfPixelInHeight)}));
  rec.log(camera_name_cam2_gt, rerun::Transform3D(rr_translation_cam2_gt,
                                                  rr_rotation_matrix_cam2_gt));

  std::cout
      << "Ground truth cameras logged to Rerun: world/ground_truth/cam[0,1,2]"
      << std::endl;
  std::cout
      << "  (Transformed to Camera 0 reference frame for proper comparison)"
      << std::endl;

  ////////////////////////////////////////////////////////////////////////

  // ============================================================================
  // CREATE AND LOG IMAGES FOR EACH CAMERA
  // ============================================================================
  // These images show the projected points from the scene
  // They are logged under the ground truth camera hierarchy
  // ============================================================================

  std::string fileName;
  fileName = std::string("image_cam0") + std::to_string(focalLength) +
             std::string("_.png");
  cv::Mat img_cam0 =
      createImage(focalLength, numberOfPixelInHeight, numberOfPixelInWidth,
                  imagePointsCam0, fileName);
  // Log the image to the ground truth camera entity in the hierarchy
  rec.log("world/ground_truth/cam0/image/rgb",
          rerun::Image::from_grayscale8(
              img_cam0, {numberOfPixelInWidth, numberOfPixelInHeight}));

  fileName = std::string("image_cam1") + std::to_string(focalLength) +
             std::string("_.png");
  cv::Mat img_cam1 =
      createImage(focalLength, numberOfPixelInHeight, numberOfPixelInWidth,
                  imagePointsCam1, fileName);
  // Log the image to the ground truth camera entity in the hierarchy
  rec.log("world/ground_truth/cam1/image/rgb",
          rerun::Image::from_grayscale8(
              img_cam1, {numberOfPixelInWidth, numberOfPixelInHeight}));

  fileName = std::string("image_cam2") + std::to_string(focalLength) +
             std::string("_.png");
  cv::Mat img_cam2 =
      createImage(focalLength, numberOfPixelInHeight, numberOfPixelInWidth,
                  imagePointsCam2, fileName);
  // Log the image to the ground truth camera entity in the hierarchy
  rec.log("world/ground_truth/cam2/image/rgb",
          rerun::Image::from_grayscale8(
              img_cam2, {numberOfPixelInWidth, numberOfPixelInHeight}));

  std::cout << "Camera images logged to Rerun: "
               "world/ground_truth/cam[0,1,2]/image/rgb"
            << std::endl;

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

    // Pass camera pair index to create unique window titles
    drawMatchesBetweenTheTwoFrames(images[i], images[i + 1], keypoints[i],
                                   keypoints[i + 1], matches,
                                   static_cast<int>(i));

    all_matches.push_back(matches);
  }

  // Make sure all OpenCV windows are closed after viewing matches
  cv::destroyAllWindows();
  cv::waitKey(1); // Process window events

  std::cout << "\nAll match visualization windows closed." << std::endl;

  // camera0  is the world,we set identity and zero
  std::vector<CameraExtrinsics> cameras(N_CAMERAS);
  cameras[0].R = cv::Mat::eye(3, 3, CV_64F);
  cameras[0].t = cv::Mat::zeros(3, 1, CV_64F);

  std::vector<double> cameraParams(6 * N_CAMERAS, 0.0);

  // Initialize camera0 in the bundle with R=I, t=0 => angle-axis=0, ...
  RtToAngleAxisAndTranslate(cameras[0].R, cameras[0].t, &cameraParams[0 * 6]);

  // All 3D points in world coords (only triangulated points)
  std::vector<cv::Point3f> globalPoints3D;

  // All 2D observations of these 3D points, camera_idx, 3d point_idx, x, y
  std::vector<Observation> observations;

  // ============================================================================
  // TRACK-BASED SfM STRUCTURE (Production Approach)
  // ============================================================================
  // Instead of assuming all points are visible in all cameras, we build tracks
  // incrementally. Each track represents one physical 3D point across views.
  //
  // tracks[track_id] contains:
  //   - All camera observations of this point
  //   - Index into globalPoints3D once triangulated
  //
  // We use a map to quickly find which track corresponds to a feature in a
  // camera:
  //   featureToTrack[(camera_idx, feature_idx)] = track_id
  // ============================================================================

  std::vector<Track> tracks;

  // Map from (camera_idx, feature_idx) → track_id
  std::map<std::pair<int, int>, int> featureToTrack;

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

    // Ensure translation is a column vector (3×1), not a row vector (1×3)
    if (t_im1_to_i.rows == 1 && t_im1_to_i.cols == 3) {
      t_im1_to_i = t_im1_to_i.t(); // Transpose to 3×1
      std::cout << "WARNING: recoverPose returned row vector, transposing to "
                   "column vector"
                << std::endl;
    }

    cv::Mat R_0_to_im1 = cameras[i - 1].R;
    cv::Mat t_0_to_im1 = cameras[i - 1].t;

    // DEBUG: Check matrix dimensions
    std::cout << "DEBUG iteration i=" << i << std::endl;
    std::cout << "  R_im1_to_i: " << R_im1_to_i.rows << "x" << R_im1_to_i.cols
              << std::endl;
    std::cout << "  t_im1_to_i: " << t_im1_to_i.rows << "x" << t_im1_to_i.cols
              << std::endl;
    std::cout << "  R_0_to_im1: " << R_0_to_im1.rows << "x" << R_0_to_im1.cols
              << std::endl;
    std::cout << "  t_0_to_im1: " << t_0_to_im1.rows << "x" << t_0_to_im1.cols
              << std::endl;

    cv::Mat R_0_to_i = R_im1_to_i * R_0_to_im1;
    cv::Mat t_0_to_i = R_im1_to_i * t_0_to_im1 + t_im1_to_i;

    std::cout << "  R_0_to_i: " << R_0_to_i.rows << "x" << R_0_to_i.cols
              << std::endl;
    std::cout << "  t_0_to_i: " << t_0_to_i.rows << "x" << t_0_to_i.cols
              << std::endl;

    cameras[i].R = R_0_to_i.clone();
    cameras[i].t = t_0_to_i.clone();

    RtToAngleAxisAndTranslate(cameras[i].R, cameras[i].t, &cameraParams[i * 6]);

    // Validate matrix dimensions before building projection matrices
    if (cameras[i - 1].R.rows != 3 || cameras[i - 1].R.cols != 3) {
      std::cerr << "ERROR: cameras[" << i - 1
                << "].R has wrong dimensions: " << cameras[i - 1].R.rows << "x"
                << cameras[i - 1].R.cols << " (expected 3x3)" << std::endl;
      return;
    }
    if (cameras[i - 1].t.rows != 3 || cameras[i - 1].t.cols != 1) {
      std::cerr << "ERROR: cameras[" << i - 1
                << "].t has wrong dimensions: " << cameras[i - 1].t.rows << "x"
                << cameras[i - 1].t.cols << " (expected 3x1)" << std::endl;
      return;
    }
    if (cameras[i].R.rows != 3 || cameras[i].R.cols != 3) {
      std::cerr << "ERROR: cameras[" << i
                << "].R has wrong dimensions: " << cameras[i].R.rows << "x"
                << cameras[i].R.cols << " (expected 3x3)" << std::endl;
      return;
    }
    if (cameras[i].t.rows != 3 || cameras[i].t.cols != 1) {
      std::cerr << "ERROR: cameras[" << i
                << "].t has wrong dimensions: " << cameras[i].t.rows << "x"
                << cameras[i].t.cols << " (expected 3x1)" << std::endl;
      return;
    }

    cv::Mat Rt_im1(3, 4, CV_64F);
    std::cout << "  Copying cameras[" << i - 1 << "].R ("
              << cameras[i - 1].R.rows << "x" << cameras[i - 1].R.cols
              << ") to Rt_im1" << std::endl;
    std::cout << "  Copying cameras[" << i - 1 << "].t ("
              << cameras[i - 1].t.rows << "x" << cameras[i - 1].t.cols
              << ") to Rt_im1" << std::endl;

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

    // if you want to display the camera position in th world use these
    cv::Mat R_im1_to_0 = cameras[i - 1].R.t();
    cv::Mat t_im1_to_0 = -cameras[i - 1].R.t() * cameras[i - 1].t;

    std::cout << "Position of cam" << i - 1 << " expressed in cam0 (aka cam_0_"
              << i - 1 << ") Rotation: " << "\n"
              << R_im1_to_0 << std::endl;
    std::cout << "Translation: \n" << t_im1_to_0 << std::endl;

    // ========================================================================
    // TRACK-BASED INCREMENTAL SfM (Production Approach)
    // ========================================================================
    // This approach handles PARTIAL VISIBILITY correctly:
    // - Some points may only be visible in a subset of cameras
    // - We identify points by their feature indices, not match order
    // - Tracks are created or extended as we add cameras
    //
    // For each match between camera (i-1) and camera i:
    //   1. Check if camera (i-1)'s feature already belongs to a track
    //   2. If YES: Extend that track with camera i's observation
    //   3. If NO: Create a new track with both observations
    // ========================================================================

    for (size_t k = 0; k < all_matches[i - 1].size(); k++) {

      const cv::DMatch &match = all_matches[i - 1][k];
      int feature_im1 = match.queryIdx; // Feature index in camera i-1
      int feature_i = match.trainIdx;   // Feature index in camera i

      // Check if camera (i-1)'s feature already belongs to a track
      std::pair<int, int> key_im1(i - 1, feature_im1);

      int track_id = -1;
      bool is_new_track = false;

      if (featureToTrack.find(key_im1) != featureToTrack.end()) {
        // CASE 1: This feature is already part of an existing track
        // → Extend the track with the new camera observation
        track_id = featureToTrack[key_im1];
        /*
        std::cout << "  [Iteration " << i << ", match " << k << "] "
                  << "Extending track " << track_id
                  << " (Cam" << i-1 << "[" << feature_im1 << "] already
        tracked)"
                  << std::endl;
        */
      } else {
        // CASE 2: This is a NEW track starting from this camera pair
        // → Create a new track
        track_id = static_cast<int>(tracks.size());
        tracks.emplace_back();
        is_new_track = true;

        /*
        std::cout << "  [Iteration " << i << ", match " << k << "] "
                  << "Creating NEW track " << track_id
                  << " (first seen in Cam" << i-1 << " and Cam" << i << ")"
                  << std::endl;
        */

        // Register camera (i-1)'s feature to this track
        tracks[track_id].addObservation(i - 1, feature_im1, pts_im1[k]);
        featureToTrack[key_im1] = track_id;
      }

      // Add camera i's observation to the track (always happens)
      std::pair<int, int> key_i(i, feature_i);
      tracks[track_id].addObservation(i, feature_i, pts_i[k]);
      featureToTrack[key_i] = track_id;

      // Triangulate and store 3D point if this is a new track
      if (is_new_track) {
        int point3D_idx = static_cast<int>(globalPoints3D.size());
        globalPoints3D.push_back(newPoints_in_cam0[k]);
        tracks[track_id].point3D_idx = point3D_idx;

        /*
        std::cout << "    → Triangulated as 3D point " << point3D_idx
                  << " at (" << newPoints_in_cam0[k].x << ", "
                  << newPoints_in_cam0[k].y << ", "
                  << newPoints_in_cam0[k].z << ")" << std::endl;
        */
        // ============================================================================
        // VISUALIZATION: Only add to Rerun when creating NEW tracks
        // ============================================================================
        // We only visualize newly triangulated points in this iteration.
        // Existing tracks that are being extended already have their 3D points
        // visualized from previous iterations.
        // ============================================================================
        rr_triangulated_pointsInCam0.push_back({newPoints_in_cam0[k].x,
                                                newPoints_in_cam0[k].y,
                                                newPoints_in_cam0[k].z});
      } else {
        /*
        std::cout << "    → Reusing existing 3D point "
                  << tracks[track_id].point3D_idx
                  << " (not added to visualization - already visualized)" <<
        std::endl;
        */
      }
    }

    // Print summary for this iteration
    std::cout << "\n=== Iteration " << i << " Summary ===" << std::endl;
    std::cout << "Total tracks: " << tracks.size() << std::endl;
    std::cout << "Total 3D points: " << globalPoints3D.size() << std::endl;

    // Count observations per camera
    std::map<int, int> obs_per_camera;
    for (const auto &track : tracks) {
      for (const auto &obs : track.observations) {
        obs_per_camera[obs.camera_idx]++;
      }
    }
    for (const auto &pair : obs_per_camera) {
      std::cout << "  Camera " << pair.first << ": " << pair.second
                << " observations" << std::endl;
    }
    std::cout << "================================\n" << std::endl;

    // ============================================================================
    // VISUALIZE NEWLY TRIANGULATED POINTS
    // ============================================================================
    // Only points from NEW tracks are added to rr_triangulated_pointsInCam0.
    // Extended tracks (already visualized) are not included.
    // This ensures we don't show duplicate points in the visualization.
    // ============================================================================

    if (rr_triangulated_pointsInCam0.size() > 0) {
      std::vector<rerun::Color> colors(rr_triangulated_pointsInCam0.size());
      for (size_t j = 0; j < rr_triangulated_pointsInCam0.size(); ++j) {

        if ((i % 2) == 0) {
          colors[j] = rerun::Color(255, 0, 0); // Red
        } else {
          colors[j] = rerun::Color(0, 255, 0); // Green, for example
        }
      }

      rec.log("world/new_triangulated_points_iteration_" + std::to_string(i),
              rerun::Points3D(rr_triangulated_pointsInCam0)
                  .with_radii(std::vector<float>(
                      rr_triangulated_pointsInCam0.size(), 0.05))
                  .with_colors(colors));

      std::cout << "Logged " << rr_triangulated_pointsInCam0.size()
                << " NEW triangulated points to Rerun" << std::endl;
    } else {
      std::cout << "No new points triangulated in this iteration (all tracks "
                   "extended)"
                << std::endl;
    }

    std::cout << "press any key to continue " << std::endl;
    std::cin.get();
  }

  // ============================================================================
  // BUILD OBSERVATIONS FROM TRACKS FOR BUNDLE ADJUSTMENT
  // ============================================================================
  // Now that all cameras have been added and tracks are complete, we convert
  // the track structure into the observations list for Ceres optimization.
  //
  // For each track:
  //   - Get the 3D point index
  //   - For each camera observation in that track:
  //       → Convert pixel coords to Snavely normalized coords
  //       → Create an Observation linking (camera, 3D point, 2D coords)
  // ============================================================================

  std::cout << "\n=== Building Observations for Bundle Adjustment ==="
            << std::endl;

  for (size_t track_id = 0; track_id < tracks.size(); track_id++) {
    const Track &track = tracks[track_id];

    if (track.point3D_idx < 0) {
      std::cerr << "ERROR: Track " << track_id << " has no 3D point!"
                << std::endl;
      continue;
    }

    // For each camera observation in this track
    for (const auto &feat_obs : track.observations) {
      Observation obs;
      obs.camera_idx = feat_obs.camera_idx;
      obs.point_idx = track.point3D_idx;

      // Convert from pixel coordinates to Snavely normalized coordinates
      // Snavely convention: negative sign and normalize by focal length
      obs.x = -(feat_obs.pixel.x - cx) / fx;
      obs.y = -(feat_obs.pixel.y - cy) / fy;

      observations.push_back(obs);
    }
  }

  std::cout << "Total observations created: " << observations.size()
            << std::endl;
  std::cout << "Total 3D points: " << globalPoints3D.size() << std::endl;
  std::cout << "Total tracks: " << tracks.size() << std::endl;

  // Verify observation counts per camera
  std::map<int, int> final_obs_per_camera;
  for (const auto &obs : observations) {
    final_obs_per_camera[obs.camera_idx]++;
  }
  std::cout << "\nObservations per camera:" << std::endl;
  for (const auto &pair : final_obs_per_camera) {
    std::cout << "  Camera " << pair.first << ": " << pair.second
              << " observations" << std::endl;
  }
  std::cout << "====================================================\n"
            << std::endl;

  // ============================================================================
  // PREPARE DATA FOR CERES OPTIMIZATION
  // ============================================================================

  std::vector<double> pointParams;
  pointParams.resize(3 * globalPoints3D.size());
  // Copy from cv::Point3f to the double vector
  for (size_t i = 0; i < globalPoints3D.size(); i++) {
    pointParams[3 * i + 0] = globalPoints3D[i].x;
    pointParams[3 * i + 1] = globalPoints3D[i].y;
    pointParams[3 * i + 2] = globalPoints3D[i].z;
  }

  ceres::Problem problem;

  std::cout << "Building Ceres problem with " << observations.size()
            << " residual blocks..." << std::endl;

  int id = 0;
  for (const Observation &obs : observations) {

    ceres::CostFunction *costFunc = SnavelyReprojectionErrorFixedCamera::Create(
        obs.x, obs.y, 1, k1, k2, id);
    id++;

    double *cameraParamsPtr = &cameraParams[obs.camera_idx * 6];
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
    std::cout << "Angle-axis = (" << cameraParams[i * 6] << ", "
              << cameraParams[i * 6 + 1] << ", " << cameraParams[i * 6 + 2]
              << ")\n";
    std::cout << "Translation = (" << cameraParams[i * 6 + 3] << ", "
              << cameraParams[i * 6 + 4] << ", " << cameraParams[i * 6 + 5]
              << ")\n";
  }

  // ============================================================================
  // FIX CAMERA 0 TO ELIMINATE GAUGE FREEDOM
  // ============================================================================
  // In Structure from Motion, we must fix at least one camera to eliminate
  // the gauge freedom (7 DOF: 3 rotation + 3 translation + 1 scale).
  //
  // Camera 0 is our WORLD REFERENCE FRAME:
  //   - R = I (identity rotation)
  //   - t = 0 (at origin)
  //
  // This fixes:
  //   ✅ Global rotation (3 DOF)
  //   ✅ Global translation (3 DOF)
  //   ⚠️ Scale remains ambiguous in monocular SfM (1 DOF)
  //
  // Without this constraint, the optimization would have infinite solutions!
  // ============================================================================

  problem.SetParameterBlockConstant(&cameraParams[0]);

  std::cout << "\n🔒 Camera 0 is FIXED (world reference frame):" << std::endl;
  std::cout << "   R = I (identity), t = (0,0,0)" << std::endl;
  std::cout << "   This eliminates gauge freedom in the optimization."
            << std::endl;
  std::cout << "   Only Camera 1 and Camera 2 poses will be optimized.\n"
            << std::endl;

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
    std::cout << "Angle-axis = (" << cameraParams[i * 6] << ", "
              << cameraParams[i * 6 + 1] << ", " << cameraParams[i * 6 + 2]
              << ")\n";
    std::cout << "Translation = (" << cameraParams[i * 6 + 3] << ", "
              << cameraParams[i * 6 + 4] << ", " << cameraParams[i * 6 + 5]
              << ")\n";
  }

  // ============================================================================
  // VISUALIZE OPTIMIZED 3D POINTS AFTER BUNDLE ADJUSTMENT
  // ============================================================================
  // After Ceres optimization, pointParams contains the refined 3D coordinates.
  // This shows the final optimized structure (yellow points).
  // Compare with:
  //   - "world/points" (ground truth - original ellipsoid)
  //   - "world/new_triangulated_points_iteration_*" (initial triangulation)
  // ============================================================================

  std::cout << "\n=== Visualizing Optimized 3D Points ===" << std::endl;
  std::cout << "Number of optimized points: " << (pointParams.size() / 3)
            << std::endl;

  std::vector<rerun::components::Position3D> point3d_positions_after_BA;
  std::vector<float> point_sizes_after_BA;
  std::vector<rerun::Color> colors_after_BA;

  for (std::size_t i = 0; i < pointParams.size(); i = i + 3) {
    x = pointParams[i + 0];
    y = pointParams[i + 1];
    z = pointParams[i + 2];
    point3d_positions_after_BA.push_back({x, y, z});
    point_sizes_after_BA.push_back(
        0.06); // Slightly larger to distinguish from initial
    colors_after_BA.push_back(
        rerun::Color(255, 255, 0)); // Yellow = Red + Green
  }

  rec.log("world/optimized_points_after_BA",
          rerun::Points3D(point3d_positions_after_BA)
              .with_radii(point_sizes_after_BA)
              .with_colors(colors_after_BA));

  std::cout << "Optimized points logged to Rerun as "
               "'world/optimized_points_after_BA' (yellow)"
            << std::endl;

  // ============================================================================
  // COMPARE OPTIMIZED RESULTS WITH GROUND TRUTH
  // ============================================================================

  std::cout << "\n" << std::string(80, '=') << std::endl;
  std::cout << "GROUND TRUTH COMPARISON" << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  // ============================================================================
  // IMPORTANT: Ground Truth Must Be Relative to Camera 0
  // ============================================================================
  // In the optimization, we set Camera 0 as the world reference (R=I, t=0).
  // This means our "world frame" IS Camera 0's frame.
  //
  // Therefore, ground truth comparisons must be:
  //   - Camera 0: R=I, t=0 (by definition)
  //   - Camera i: Relative transform from Cam0 to Cam i
  //
  // Original ground truth had:
  //   - rotation_world_in_CamX = R_world→camX
  //   - t_world_in_CamX = t_world→camX
  //
  // But our optimization uses Cam0 AS world, so we need:
  //   - R_cam0→camX = R_world→camX * R_cam0→world
  //                 = R_world→camX * R_world→cam0^T
  //   - t_cam0→camX = R_world→camX * (-R_world→cam0^T * t_world→cam0) +
  //   t_world→camX
  // ============================================================================

  std::vector<cv::Mat> groundTruth_R(N_CAMERAS);
  std::vector<cv::Mat> groundTruth_t(N_CAMERAS);

  // Camera 0 is the reference frame by definition
  groundTruth_R[0] = cv::Mat::eye(3, 3, CV_64F);
  groundTruth_t[0] = cv::Mat::zeros(3, 1, CV_64F);

  // For other cameras, compute relative transforms from Camera 0
  // R_0→i = R_world→i * R_0→world = R_world→i * R_world→0^T
  // t_0→i = R_world→i * (0 - t_world→0) + t_world→i = R_world→i * (-t_world→0)
  // + t_world→i

  cv::Mat R_world_to_cam0 = rotation_world_in_Cam0.clone();
  cv::Mat t_world_to_cam0 = t_world_in_Cam0.clone();

  for (size_t i = 1; i < N_CAMERAS; i++) {
    cv::Mat R_world_to_camI, t_world_to_camI;

    if (i == 1) {
      R_world_to_camI = rotation_world_in_Cam1.clone();
      t_world_to_camI = t_world_in_Cam1.clone();
    } else {
      R_world_to_camI = rotation_world_in_Cam2.clone();
      t_world_to_camI = t_world_in_Cam2.clone();
    }

    // Relative rotation: R_0→i = R_world→i * R_world→0^T
    groundTruth_R[i] = R_world_to_camI * R_world_to_cam0.t();

    // Relative translation: t_0→i = R_world→i * (-t_world→0) + t_world→i
    //                              = -R_world→i * t_world→0 + t_world→i
    groundTruth_t[i] = -R_world_to_camI * t_world_to_cam0 + t_world_to_camI;
  }

  std::cout << "\nGround truth transforms (relative to Camera 0):" << std::endl;
  std::cout << "  Camera 0: R=I (reference frame), t=0" << std::endl;
  for (size_t i = 1; i < N_CAMERAS; i++) {
    cv::Mat aa_gt;
    cv::Rodrigues(groundTruth_R[i], aa_gt);
    std::cout << "  Camera " << i << ": angle-axis=" << aa_gt.t()
              << ", t=" << groundTruth_t[i].t() << std::endl;
  }

  // Convert optimized angle-axis back to rotation matrices
  std::cout << "\n--- Camera Pose Errors ---\n" << std::endl;

  double total_rotation_error = 0.0;
  double total_translation_error = 0.0;

  for (size_t cam_id = 0; cam_id < N_CAMERAS; cam_id++) {
    // Extract optimized angle-axis and translation
    cv::Mat optimized_angleaxis =
        (cv::Mat_<double>(3, 1) << cameraParams[cam_id * 6 + 0],
         cameraParams[cam_id * 6 + 1], cameraParams[cam_id * 6 + 2]);

    cv::Mat optimized_t =
        (cv::Mat_<double>(3, 1) << cameraParams[cam_id * 6 + 3],
         cameraParams[cam_id * 6 + 4], cameraParams[cam_id * 6 + 5]);

    // Convert angle-axis to rotation matrix
    cv::Mat optimized_R;
    cv::Rodrigues(optimized_angleaxis, optimized_R);

    // Compute rotation error (geodesic distance on SO(3))
    cv::Mat R_diff = optimized_R.t() * groundTruth_R[cam_id];
    cv::Mat angleaxis_diff;
    cv::Rodrigues(R_diff, angleaxis_diff);
    double rotation_error_rad = cv::norm(angleaxis_diff);
    double rotation_error_deg = rotation_error_rad * 180.0 / M_PI;

    // Compute translation error (Euclidean distance)
    cv::Mat t_diff = optimized_t - groundTruth_t[cam_id];
    double translation_error = cv::norm(t_diff);

    std::cout << "Camera " << cam_id << ":" << std::endl;
    std::cout << "  Rotation error:    " << rotation_error_deg << " degrees ("
              << rotation_error_rad << " rad)" << std::endl;
    std::cout << "  Translation error: " << translation_error << " units"
              << std::endl;

    std::cout << "  Ground truth R:\n" << groundTruth_R[cam_id] << std::endl;
    std::cout << "  Optimized R:\n" << optimized_R << std::endl;
    std::cout << "  Ground truth t: " << groundTruth_t[cam_id].t() << std::endl;
    std::cout << "  Optimized t:    " << optimized_t.t() << std::endl;
    std::cout << std::endl;

    if (cam_id > 0) { // Don't count camera 0 (it's fixed)
      total_rotation_error += rotation_error_deg;
      total_translation_error += translation_error;
    }
  }

  std::cout << "Average rotation error (non-fixed cameras): "
            << total_rotation_error / (N_CAMERAS - 1) << " degrees"
            << std::endl;
  std::cout << "Average translation error (non-fixed cameras): "
            << total_translation_error / (N_CAMERAS - 1) << " units"
            << std::endl;

  // ============================================================================
  // VISUALIZE OPTIMIZED CAMERAS IN RERUN
  // ============================================================================
  std::cout << "\nVisualizing optimized camera poses in Rerun..." << std::endl;

  for (size_t cam_id = 0; cam_id < N_CAMERAS; cam_id++) {
    // Extract optimized pose
    cv::Mat optimized_angleaxis =
        (cv::Mat_<double>(3, 1) << cameraParams[cam_id * 6 + 0],
         cameraParams[cam_id * 6 + 1], cameraParams[cam_id * 6 + 2]);

    cv::Mat optimized_t =
        (cv::Mat_<double>(3, 1) << cameraParams[cam_id * 6 + 3],
         cameraParams[cam_id * 6 + 4], cameraParams[cam_id * 6 + 5]);

    cv::Mat optimized_R;
    cv::Rodrigues(optimized_angleaxis, optimized_R);

    // Convert to Camera-in-World (inverse transform) for visualization
    // If we have R_world→cam and t_world→cam, then:
    // R_cam→world = R_world→cam^T
    // t_cam→world = -R_world→cam^T * t_world→cam
    cv::Mat R_cam_in_world = optimized_R.t();
    cv::Mat t_cam_in_world = -optimized_R.t() * optimized_t;

    // Convert to Rerun format
    rerun::Vec3D rr_translation =
        getRerunTranslationFromCvMat<double>(t_cam_in_world);
    rerun::Mat3x3 rr_rotation =
        getRerunRotationFromCvMat<double>(R_cam_in_world);

    // Log optimized camera
    std::string opt_camera_name =
        "world/optimized/cam" + std::to_string(cam_id);

    rec.log(opt_camera_name,
            rerun::Pinhole::from_focal_length_and_resolution(
                {float(focalLength * mx), float(focalLength * my)},
                {float(numberOfPixelInWidth), float(numberOfPixelInHeight)}));

    rec.log(opt_camera_name, rerun::Transform3D(rr_translation, rr_rotation));
  }

  std::cout << "Optimized cameras logged to Rerun: world/optimized/cam[0,1,2]"
            << std::endl;
  std::cout << "  • Green = Ground truth cameras" << std::endl;
  std::cout << "  • Blue = Optimized cameras (should overlap ground truth)"
            << std::endl;

  // ============================================================================
  // COMPARE 3D POINTS WITH GROUND TRUTH
  // ============================================================================

  std::cout << "\n--- 3D Point Reconstruction Errors ---\n" << std::endl;

  std::cout << "NOTE: Transforming ground truth points from original world "
               "frame to Camera 0 frame..."
            << std::endl;

  // ============================================================================
  // Transform ground truth points to Camera 0's coordinate frame
  // ============================================================================
  // Our reconstructed points are in Camera 0's frame (since cameras[0] = I)
  // Ground truth points are in the original world frame
  // We need: P_cam0 = R_world→cam0 * P_world + t_world→cam0
  // ============================================================================

  std::vector<cv::Point3f> groundTruth_points_in_cam0;
  for (const auto &pt_world : objectPointsInWorldCoordinate) {
    cv::Mat pt_world_mat =
        (cv::Mat_<double>(3, 1) << pt_world[0], pt_world[1], pt_world[2]);
    cv::Mat pt_cam0 = rotation_world_in_Cam0 * pt_world_mat + t_world_in_Cam0;
    groundTruth_points_in_cam0.emplace_back(pt_cam0.at<double>(0, 0),
                                            pt_cam0.at<double>(1, 0),
                                            pt_cam0.at<double>(2, 0));
  }

  // Compute RMS error between optimized and ground truth 3D points
  if (globalPoints3D.size() != groundTruth_points_in_cam0.size()) {
    std::cout << "WARNING: Number of reconstructed points ("
              << globalPoints3D.size() << ") differs from ground truth ("
              << groundTruth_points_in_cam0.size() << ")" << std::endl;
  }

  size_t num_points_to_compare =
      std::min(globalPoints3D.size(), groundTruth_points_in_cam0.size());

  double sum_squared_error = 0.0;
  double max_error = 0.0;
  size_t max_error_idx = 0;

  for (size_t i = 0; i < num_points_to_compare; i++) {
    // Get optimized point from pointParams
    double opt_x = pointParams[3 * i + 0];
    double opt_y = pointParams[3 * i + 1];
    double opt_z = pointParams[3 * i + 2];

    // Get ground truth point (in Camera 0 frame)
    double gt_x = groundTruth_points_in_cam0[i].x;
    double gt_y = groundTruth_points_in_cam0[i].y;
    double gt_z = groundTruth_points_in_cam0[i].z;

    // Compute Euclidean distance
    double dx = opt_x - gt_x;
    double dy = opt_y - gt_y;
    double dz = opt_z - gt_z;
    double error = std::sqrt(dx * dx + dy * dy + dz * dz);

    sum_squared_error += error * error;

    if (error > max_error) {
      max_error = error;
      max_error_idx = i;
    }
  }

  double rms_error = std::sqrt(sum_squared_error / num_points_to_compare);
  double mean_error = std::sqrt(sum_squared_error) / num_points_to_compare;

  std::cout << "Number of points compared: " << num_points_to_compare
            << std::endl;
  std::cout << "RMS error: " << rms_error << " units" << std::endl;
  std::cout << "Mean error: " << mean_error << " units" << std::endl;
  std::cout << "Max error: " << max_error << " units (at point "
            << max_error_idx << ")" << std::endl;

  // Show a few example comparisons
  std::cout << "\nExample point comparisons (first 5 points in Camera 0 frame):"
            << std::endl;
  for (size_t i = 0; i < std::min(size_t(5), num_points_to_compare); i++) {
    double opt_x = pointParams[3 * i + 0];
    double opt_y = pointParams[3 * i + 1];
    double opt_z = pointParams[3 * i + 2];

    double gt_x = groundTruth_points_in_cam0[i].x;
    double gt_y = groundTruth_points_in_cam0[i].y;
    double gt_z = groundTruth_points_in_cam0[i].z;

    double dx = opt_x - gt_x;
    double dy = opt_y - gt_y;
    double dz = opt_z - gt_z;
    double error = std::sqrt(dx * dx + dy * dy + dz * dz);

    std::cout << "  Point " << i << ":" << std::endl;
    std::cout << "    Ground truth (Cam0): (" << gt_x << ", " << gt_y << ", "
              << gt_z << ")" << std::endl;
    std::cout << "    Optimized (Cam0):    (" << opt_x << ", " << opt_y << ", "
              << opt_z << ")" << std::endl;
    std::cout << "    Error:               " << error << " units" << std::endl;
  }

  // Print comprehensive reconstruction quality summary
  printReconstructionSummary(
      N_CAMERAS, globalPoints3D.size(), observations.size(),
      summary.initial_cost, summary.final_cost,
      static_cast<int>(summary.num_successful_steps +
                       summary.num_unsuccessful_steps),
      total_rotation_error / (N_CAMERAS - 1),
      total_translation_error / (N_CAMERAS - 1), rms_error, max_error);

  std::cout << "\nPress any key to exit..." << std::endl;
  std::cin.get();
}

void evaluateSnavelyReprojectionErrorFixedCamera() {

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

  double txCam0, tyCam0, tzCam0;
  cv::Vec3d thetaCam0;

  thetaCam0[0] = -M_PI / 24;
  thetaCam0[1] = +M_PI / 12;
  thetaCam0[2] = +M_PI / 18;

  cv::Mat rotation_Cam0_in_world;
  rotation_Cam0_in_world = eulerAnglesToRotationMatrix(thetaCam0);

  txCam0 = 0.0;
  tyCam0 = 0.0;
  tzCam0 = 0.0;

  cv::Mat t_Cam0_in_world = (cv::Mat_<double>(3, 1) << txCam0, tyCam0, tzCam0);

  double k1, k2;

  k1 = 0.;
  k2 = 0.;

  cv::Mat rotation_world_in_Cam0 = rotation_Cam0_in_world.t();
  cv::Mat t_world_in_Cam0 = -rotation_Cam0_in_world.t() * t_Cam0_in_world;

  std::vector<cv::Point2f> imagePointsCam0;

  std::vector<cv::Vec3f> objectPointsInWorldCoordinate = {
      {1.2, -0.7, 2}, {-1, 0.5, 1.0}, {-0.2, +0.4, 2.2}};

  cv::projectPoints(objectPointsInWorldCoordinate, rotation_world_in_Cam0,
                    t_world_in_Cam0, K, cv::noArray(), imagePointsCam0);

  double totalError = 0.0;

  ceres::Problem problem;
  double cameraParams[6] = {0, 0, 0, 0, 0, 0};

  std::vector<std::array<double, 3>> point_parameters(
      objectPointsInWorldCoordinate.size());

  for (std::size_t i = 0; i < objectPointsInWorldCoordinate.size(); i++) {

    // negative sign for Snavely

    double x_sn = -(imagePointsCam0[i].x - cx) / fx;
    double y_sn = -(imagePointsCam0[i].y - cy) / fy;
    SnavelyReprojectionErrorFixedCamera errorModel(x_sn, y_sn, 1, k1, k2,
                                                   static_cast<int>(i));

    RtToAngleAxisAndTranslate(rotation_world_in_Cam0, t_world_in_Cam0,
                              cameraParams);

    point_parameters[i][0] = objectPointsInWorldCoordinate[i][0];
    point_parameters[i][1] = objectPointsInWorldCoordinate[i][1];
    point_parameters[i][2] = objectPointsInWorldCoordinate[i][2];

    double residuals[2];

    // Evaluate the residual using the error model
    errorModel(cameraParams, point_parameters[i].data(), residuals);

    double error =
        std::sqrt(residuals[0] * residuals[0] + residuals[1] * residuals[1]);

    totalError += error;

    std::cout << "error: " << error << std::endl;

    std::cout << "---------------------------------------" << std::endl;

    ceres::CostFunction *costFunc = SnavelyReprojectionErrorFixedCamera::Create(
        x_sn, y_sn, 1, k1, k2, static_cast<int>(i));

    // Add a residual block:
    problem.AddResidualBlock(costFunc,
                             nullptr, // squared loss
                             cameraParams, point_parameters[i].data());
    std::cout << "Adding residual with block id:  " << static_cast<int>(i)
              << " 3D Point: " << point_parameters[i][0] << ", "
              << point_parameters[i][1] << ", " << point_parameters[i][2]
              << " Observed: " << x_sn << ", " << y_sn << std::endl;
  }

  std::cout << "\n";

  // Prepare for evaluation
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.apply_loss_function = false; // Apply loss function if used
  eval_options.num_threads = 1; // Use a single thread for simplicity

  // Output containers
  double total_cost = 0.0;
  std::vector<double> residuals;
  std::vector<double> gradient;
  ceres::CRSMatrix jacobian;

  // Evaluate the problem
  problem.Evaluate(eval_options, &total_cost, &residuals, &gradient, &jacobian);

  // Print the total cost
  std::cout << "Total cost: " << total_cost << std::endl;

  // Print residuals
  std::cout << "Residuals: ";
  for (size_t i = 0; i < residuals.size(); ++i) {
    std::cout << residuals[i] << " ";
  }
  std::cin.get();
  std::cout << std::endl;

  // Print gradient (if needed)
  std::cout << "Gradient: ";
  for (size_t i = 0; i < gradient.size(); ++i) {
    std::cout << gradient[i] << " ";
  }
  std::cin.get();
  std::cout << std::endl;

  // Print Jacobian (if needed)
  std::cout << "Jacobian: " << std::endl;
  for (int i = 0; i < jacobian.num_rows; ++i) {
    for (int j = jacobian.rows[i]; j < jacobian.rows[i + 1]; ++j) {
      std::cout << "Row " << i << ", Col " << jacobian.cols[j] << ": "
                << jacobian.values[j] << std::endl;
    }
  }
}

int main() { virtualCamIncrementalSfMFixedCam(); }
