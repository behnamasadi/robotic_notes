/*
 * Minimal C++ test for Rerun camera rays
 * This is the absolute minimum code needed to test camera rays
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <rerun.hpp>

int main() {
  // Initialize Rerun
  const auto rec = rerun::RecordingStream("Camera Ray Test");
  rec.spawn().exit_on_failure();

  // Set coordinate system
  rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);

  std::cout << "\n=== Minimal Camera Ray Test ===" << std::endl;
  std::cout << "Testing Rerun C++ SDK version 0.28.1" << std::endl;

  // ========================================================================
  // STEP 1: Create a test image (checkerboard)
  // ========================================================================
  int width = 640;
  int height = 480;
  cv::Mat img = cv::Mat::zeros(height, width, CV_8UC1);

  // Draw checkerboard
  int square_size = 40;
  for (int i = 0; i < height; i += square_size) {
    for (int j = 0; j < width; j += square_size) {
      if (((i / square_size) + (j / square_size)) % 2 == 0) {
        cv::rectangle(img, cv::Point(j, i),
                      cv::Point(j + square_size, i + square_size),
                      cv::Scalar(255), -1);
      }
    }
  }

  // Add crosshair at center
  cv::line(img, cv::Point(width / 2, 0), cv::Point(width / 2, height),
           cv::Scalar(128), 2);
  cv::line(img, cv::Point(0, height / 2), cv::Point(width, height / 2),
           cv::Scalar(128), 2);

  std::cout << "✅ Created " << width << "x" << height << " checkerboard image"
            << std::endl;

  // ========================================================================
  // STEP 2: Log camera with Pinhole model
  // ========================================================================
  double focal_length = 800.0;

  std::cout << "\n📷 Logging camera with:" << std::endl;
  std::cout << "   Focal length: " << focal_length << std::endl;
  std::cout << "   Resolution: " << width << "x" << height << std::endl;

  rec.log(
      "world/camera",
      rerun::Pinhole::from_focal_length_and_resolution(
          {static_cast<float>(focal_length), static_cast<float>(focal_length)},
          {static_cast<float>(width), static_cast<float>(height)}));

  std::cout << "✅ Logged Pinhole to: world/camera" << std::endl;

  // ========================================================================
  // STEP 3: Log camera transform (at origin)
  // ========================================================================
  rec.log("world/camera", rerun::Transform3D(rerun::components::Translation3D(
                              0.0f, 0.0f, 0.0f)));

  std::cout << "✅ Logged Transform3D to: world/camera" << std::endl;

  // ========================================================================
  // STEP 4: Log image as CHILD of camera
  // ========================================================================
  std::cout << "\n🖼️  Logging image to: world/camera/image" << std::endl;

  // Convert cv::Mat to std::vector for Rerun
  std::vector<uint8_t> image_data(img.data, img.data + (img.rows * img.cols));

  rec.log("world/camera/image", // Child of camera!
          rerun::Image::from_grayscale8(
              image_data, // Pass vector directly, not .data()
              {static_cast<uint32_t>(width), static_cast<uint32_t>(height)}));

  std::cout << "✅ Image logged as child of camera" << std::endl;

  // ========================================================================
  // Add some 3D points for reference
  // ========================================================================
  std::vector<rerun::components::Position3D> points;
  for (float z = 2.0; z < 10.0; z += 2.0) {
    points.push_back({0.0f, 0.0f, z});
    points.push_back({1.0f, 0.0f, z});
    points.push_back({-1.0f, 0.0f, z});
  }

  rec.log("world/points", rerun::Points3D(points));
  std::cout << "✅ Logged " << points.size() << " reference points"
            << std::endl;

  // ========================================================================
  // Instructions
  // ========================================================================
  std::cout << "\n" << std::string(70, '=') << std::endl;
  std::cout << "🎯 HOW TO SEE CAMERA RAYS:" << std::endl;
  std::cout << std::string(70, '=') << std::endl;
  std::cout << "1. Rerun viewer should have opened" << std::endl;
  std::cout << "2. In the entity tree (left), find: world/camera" << std::endl;
  std::cout << "3. Expand it to see: world/camera/image" << std::endl;
  std::cout << "4. Make sure you can see BOTH:" << std::endl;
  std::cout << "   - A 3D view showing the camera and points" << std::endl;
  std::cout << "   - An image view showing the checkerboard" << std::endl;
  std::cout << "5. HOVER your mouse over the checkerboard image" << std::endl;
  std::cout << "6. Look in the 3D view - you should see a YELLOW RAY!"
            << std::endl;
  std::cout << std::endl;
  std::cout << "If you see the ray: ✅ Camera rays work!" << std::endl;
  std::cout << "If you don't see the ray: ❌ May be SDK/viewer version issue"
            << std::endl;
  std::cout << std::string(70, '=') << std::endl;

  // Keep the program running so Rerun viewer stays open
  std::cout << "\nPress Enter to exit..." << std::endl;
  std::cin.get();

  return 0;
}
