#include "../arguments_parser.hpp"
#include "../collection_adapters.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rerun.hpp>

void simplePointsLogger() {

  std::vector<rerun::components::Position3D> point3d_positions;
  std::vector<rerun::components::Color> point_colors;
  std::vector<float> point_sizes; // Define a vector for point sizes

  float x, y, z;

  for (int i = 0; i < 30; ++i) {

    for (int j = 0; j < 10; ++j) {

      for (int k = 0; k < 3; ++k) {
        x = i;
        y = j;
        z = k;
        point3d_positions.push_back({x, y, z});
        // Assign a color based on the point's position
        rerun::components::Color color = {static_cast<uint8_t>(x * 8),
                                          static_cast<uint8_t>(y * 25),
                                          static_cast<uint8_t>((x + y) * 5)};
        point_colors.push_back(color);
        point_sizes.push_back(1);
      }
    }
  }
  const auto rec = rerun::RecordingStream("RIGHT_HAND_Y_DOWN");
  rec.spawn().exit_on_failure();

  rec.log_static("world",
                 rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN); // Set an up-axis
  rec.log("world/xyz",
          rerun::Arrows3D::from_vectors(
              {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}})
              .with_colors({{255, 0, 0}, {0, 255, 0}, {0, 0, 255}}));

  //   rec.log("world/points", rerun::Points3D(point3d_positions));
  // Add points with individual colors and sizes
  rec.log("world/points", rerun::Points3D(point3d_positions)
                              .with_colors(point_colors)
                              .with_radii(point_sizes));
}

int main(int argc, char **argv) {
  std::cout << "Rerun SDK Version: " << rerun::version_string() << std::endl;

  // Parse command line arguments
  ArgumentsParser parser(argc, argv);

  // Get image path from command line arguments
  std::string image_path = parser.getArg("--image");
  if (image_path.empty()) {
    image_path = parser.getArg("-i");
  }
  if (image_path.empty()) {
    // Default to the original path if no argument provided
    image_path = "../image.jpg";
    std::cout << "No image path provided, using default: " << image_path
              << std::endl;
    std::cout << "Usage: " << argv[0] << " --image <path> or -i <path>"
              << std::endl;
  }

  const auto rec = rerun::RecordingStream("rerun_example_cpp");
  rec.spawn().exit_on_failure();

  // Set up a world with coordinates
  rec.log_static("world",
                 rerun::ViewCoordinates::RIGHT_HAND_Z_UP); // Set an up-axis

  // Log a posed pinhole camera
  rec.log("world/camera", rerun::Pinhole::from_focal_length_and_resolution(
                              {500.0, 500.0}, {640.0, 480.0}));

  const Eigen::Vector3f camera_position{0.0, -1.0, 0.0};
  Eigen::Matrix3f camera_orientation;
  // clang-format off
     camera_orientation <<
         +1.0, +0.0, +0.0,
         +0.0, +0.0, +1.0,
         +0.0, -1.0, +0.0;
  // clang-format on

  rec.log("world/camera",
          rerun::Transform3D(rerun::Vec3D(camera_position.data()),
                             rerun::Mat3x3(camera_orientation.data())));

  // Read image
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  if (img.empty()) {
    std::cout << "Could not read the image: " << image_path << std::endl;
    return 1;
  }

  uint32_t width = img.cols;
  uint32_t height = img.rows;

  // Convert image to RGB as Rerun expects this format
  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

  // Log the image to the camera entity in the hierarchy
  rec.log("world/camera/image/rgb",
          rerun::Image::from_rgb24(img, {width, height}));

  // Prepare keypoints for logging
  std::vector<rerun::components::Position2D> keypoint_positions;
  for (int i = 0; i < 100; i++) {
    keypoint_positions.push_back({float(i), float(i)});
  }

  //    std::cout << "keypoint_positions.size(): " <<
  //    keypoint_positions.size()
  //              << std::endl;

  // Log the keypoints to the Rerun Viewer
  rec.log("world/camera/image/keypoints", rerun::Points2D(keypoint_positions));

  // Optional: Log using a borrowed pointer if required
  rec.log("world/camera/image/rgb_borrowed",
          rerun::Image::from_rgb24(
              rerun::borrow(img.data, img.total() * img.elemSize()),
              {width, height}));

  return 0;
}
