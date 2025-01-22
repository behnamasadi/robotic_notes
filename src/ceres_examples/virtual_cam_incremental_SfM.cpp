#include "../collection_adapters.hpp"
#include "../transformation.hpp"
#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <rerun/archetypes/arrows3d.hpp>
#include <rerun/datatypes/rgba32.hpp>

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
// ) Simple struct to hold per-camera data in a more readable form (OpenCV
// style)
//    We'll transform them into the 9-parameter "bundle" format for Ceres.
//------------------------------------------------------------------------------
struct CameraExtrinsics {
  cv::Mat R; // 3x3
  cv::Mat t; // 3x1
};

void virtualCamIncrementalSfM() {

  double txCam0, txCam1, txCam2, tyCam0, tyCam1, tyCam2, tzCam0, tzCam1, tzCam2;
  cv::Vec3d thetaCam0, thetaCam1, thetaCam2;

  ///////////////// cameras extrinsic /////////////////
  // clang-format off
    /*




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
  thetaCam0[1] = 0;
  thetaCam0[2] = 0;

  thetaCam1[0] = 0;
  thetaCam1[1] = 0;
  thetaCam1[2] = 0;

  thetaCam2[0] = 0;
  thetaCam2[1] = 0;
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

  std::cout << "///////// ground truth /////////" << std::endl;

  std::cout << "Cam0 translation in world:\n" << t_Cam0_in_world << std::endl;

  std::cout << "Cam0 roll, pitch , yaw in world:\n" << thetaCam0 << std::endl;

  std::cout << "Cam1 translation in world:\n" << t_Cam1_in_world << std::endl;

  std::cout << "Cam1 roll, pitch , yaw in world:\n" << thetaCam1 << std::endl;

  std::cout << "Cam2 translation in world:\n" << t_Cam2_in_world << std::endl;

  std::cout << "Cam2 roll, pitch , yaw in world:\n" << thetaCam2 << std::endl;

  ///////// creating ellipsoid in the world coordinate /////////
  std::vector<cv::Vec3f> objectPointsInWorldCoordinate;

  float ellipsoidCenterX = -1.5;
  float ellipsoidCenterY = 0;
  float ellipsoidCenterZ = -4;
  objectPointsInWorldCoordinate = createEllipsoidInWorldCoordinate<cv::Vec3f>(
      ellipsoidCenterX, ellipsoidCenterY, ellipsoidCenterZ);

  ///////// camera intrinsic parameters/////////

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

    // Draw matches between the two frames
    drawMatchesBetweenTheTwoFrames(images[i], images[i + 1], keypoints[i],
                                   keypoints[i + 1], matches);

    all_matches.push_back(matches);
  }

  // camera0  is the world,we set identity and zero
  std::vector<CameraExtrinsics> cameras(N_CAMERAS);
  cameras[0].R = cv::Mat::eye(3, 3, CV_64F);
  cameras[0].t = cv::Mat::zeros(3, 1, CV_64F);

  //  std::vector<cv::Point3f> triangulated_pointsInCam0;

  // Log the triangulated points to Rerun
  std::vector<rerun::components::Position3D> rr_triangulated_pointsInCam0;
  for (size_t i = 1; i < N_CAMERAS; ++i) {
    std::cout << "i is : " << i << ", prcessing camera: " << i - 1
              << " and camera: " << i << std::endl;
    //    std::cout << "all_matches[i].size(): " << all_matches[i].size()<<
    //    std::endl;

    // 1) Extract matched points, We have matched points between camera (i-1)
    // and camera i:
    std::vector<cv::Point2f> pts_im1, pts_i;

    //    std::cout << "keypoints.size()" << keypoints.size() << std::endl;
    //    std::cout << "all_matches.size()" << all_matches.size() << std::endl;

    for (const auto &match : all_matches[i - 1]) {

      /*
    std::cout << "keypoints[" << i - 1 << "][" << match.queryIdx
              << "].pt: " << keypoints[i - 1][match.queryIdx].pt << std::endl;
    std::cout << "keypoints[" << i << "][" << match.trainIdx
              << "].pt: " << keypoints[i][match.trainIdx].pt << std::endl;

    std::cout << "------------------------------------------" << std::endl;
    */
      pts_im1.push_back(keypoints[i - 1][match.queryIdx].pt);
      pts_i.push_back(keypoints[i][match.trainIdx].pt);
    }

    // 2) Compute Essential matrix & recoverPose
    cv::Mat mask;
    cv::Mat E =
        cv::findEssentialMat(pts_im1, pts_i, K, cv::RANSAC, 0.999, 1.0, mask);

    cv::Mat R_im1_to_i, t_im1_to_i;
    cv::recoverPose(E, pts_im1, pts_i, K, R_im1_to_i, t_im1_to_i, mask);

    /*
    std::cout << "E:\n" << E << std::endl;

    std::cout << "R_" << i - 1 << "_to_" << i << ":\n"
              << R_im1_to_i << std::endl;
    std::cout << "t_" << i - 1 << "_to_" << i << ":\n"
              << t_im1_to_i << std::endl;
    */

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

    //--------------------------------------------------------------------------
    // 10) Convert to 9-parameter form for Ceres and put in cameraParams,
    // cameraParams[i * 9] is the starting address and up to address
    // cameraParams[i * 9+6] will be populated by rotation and translation
    //--------------------------------------------------------------------------

    //    RtToAngleAxisAndTranslate(cameras[i].R, cameras[i].t, &cameraParams[i
    //    * 9]);

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
      //      std::cout << "pts_im1[" << i << "]" << pts_im1[i] << std::endl;
      //      std::cout << "pts_i[" << i << "]" << pts_i[i] << std::endl;
    }

    /*
    std::cout << "Rt_" << i - 1 << ":\n" << Rt_im1 << std::endl;
    std::cout << "Rt_" << i << ":\n" << Rt_i << std::endl;
    */

    std::cout << "P_" << i - 1 << ":\n" << P_im1 << std::endl;
    std::cout << "P_" << i << ":\n" << P_i << std::endl;

    cv::Mat points4D_;
    //    cv::triangulatePoints(P_im1, P_i, pts_im1, pts_i, points4D);

    cv::triangulatePoints(P_im1, P_i, pts_im1, pts_i, points4D_);
    std::vector<cv::Point3f> newPoints_ = convertHomogeneous(points4D_);

    // to display points in 3d, we need to transform them into cam0 coordinate,
    // we know that traiangulated points are in cami-1 coordinates, and
    // cameras[i - 1] transform the point from cam0 to cami-1, so we need the
    // inverse of cami-1 to transform the triangulated points from cami-1 into
    // cam0

    cv::Mat R_im1_to_0 = cameras[i - 1].R.t();
    cv::Mat t_im1_to_0 = -cameras[i - 1].R.t() * cameras[i - 1].t;

    std::cout << "Rotation from cam" << i - 1 << " to cam0\n"
              << R_im1_to_0 << std::endl;
    std::cout << "Translation from cam" << i - 1 << " to cam0\n"
              << t_im1_to_0 << std::endl;

    for (const auto &point_in_cam_im0 : newPoints_) {

      //      cv::Mat pointMat = (cv::Mat_<double>(3, 1) << point_in_cam_im1.x,
      //                          point_in_cam_im1.y, point_in_cam_im1.z);
      //      cv::Mat rotatedPoint = R_im1_to_0 * pointMat + t_im1_to_0;

      //      cv::Point3f point_in_cam0;
      //      point_in_cam0.x = rotatedPoint.at<double>(0);
      //      point_in_cam0.y = rotatedPoint.at<double>(1);
      //      point_in_cam0.z = rotatedPoint.at<double>(2);
      //      //      triangulated_pointsInCam0.push_back(point_in_cam0);
      //      rr_triangulated_pointsInCam0.push_back(
      //          {point_in_cam0.x, point_in_cam0.y, point_in_cam0.z});

      rr_triangulated_pointsInCam0.push_back(
          {point_in_cam_im0.x, point_in_cam_im0.y, point_in_cam_im0.z});
    }

    std::vector<rerun::Color> colors(rr_triangulated_pointsInCam0.size());

    for (size_t j = 0; j < rr_triangulated_pointsInCam0.size(); ++j) {

      if ((i % 2) == 0) {
        colors[j] = rerun::Color(255, 0, 0); // Red
      } else {
        colors[j] = rerun::Color(0, 255, 0); // Green, for example
      }
    }

    rec.log("world/triangulated_pointsInCam0",
            rerun::Points3D(rr_triangulated_pointsInCam0)
                .with_radii(std::vector<float>(
                    rr_triangulated_pointsInCam0.size(), 0.05))
                .with_colors(colors));
    std::cin.get();
  }
}

int main() { virtualCamIncrementalSfM(); }
