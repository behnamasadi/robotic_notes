cmake_minimum_required(VERSION 3.20)
project(robotic-notes LANGUAGES CXX)

# ------------------------------------------------------------------
# vcpkg toolchain
# ------------------------------------------------------------------
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()

set(VCPKG_TARGET_TRIPLET "x64-linux-release" CACHE STRING "")

# ------------------------------------------------------------------
# Options
# ------------------------------------------------------------------
option(BUILD_RERUN_EXAMPLES "Download Rerun C++ SDK and build rerun-dependent targets" OFF)

# ------------------------------------------------------------------
# Dependencies via vcpkg
# ------------------------------------------------------------------
find_package(Boost REQUIRED COMPONENTS iostreams filesystem)
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(manif CONFIG REQUIRED)
find_package(Sophus CONFIG REQUIRED)
find_package(Threads REQUIRED)

# ------------------------------------------------------------------
# Rerun SDK via FetchContent (use vcpkg's Arrow)
# ------------------------------------------------------------------
if(BUILD_RERUN_EXAMPLES)
    find_package(Arrow CONFIG REQUIRED)

    # Configure rerun_sdk to use vcpkg's Arrow instead of downloading it
    # Must be set before FetchContent_Declare
    set(RERUN_DOWNLOAD_AND_BUILD_ARROW OFF CACHE BOOL "" FORCE)

    include(FetchContent)

    FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )

    FetchContent_MakeAvailable(rerun_sdk)
endif()

# ------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------

# Ceres examples
add_executable(ceres_automatic_diff_dual_number src/ceres_examples/automatic_diff_dual_number.cpp)
target_link_libraries(ceres_automatic_diff_dual_number PRIVATE Ceres::ceres)

add_executable(ceres_jet_class src/ceres_examples/ceres_jet_class.cpp)
target_link_libraries(ceres_jet_class PRIVATE Ceres::ceres)

add_executable(ceres_example src/ceres_examples/ceres_example.cpp)
target_link_libraries(ceres_example PRIVATE Ceres::ceres)

add_executable(ceres_exponential_curve_fitting_example src/ceres_examples/ceres_exponential_curve_fitting_example.cpp)
target_link_libraries(ceres_exponential_curve_fitting_example PRIVATE Ceres::ceres)

if(BUILD_RERUN_EXAMPLES)
    add_executable(bundle_adjuster src/ceres_examples/bundle_adjuster.cpp)
    target_link_libraries(bundle_adjuster PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)

    add_executable(snavely_reprojection_error src/ceres_examples/snavely_reprojection_error.cpp)
    target_link_libraries(snavely_reprojection_error PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)

    add_executable(incremental_SfM src/ceres_examples/incremental_SfM.cpp)
    target_link_libraries(incremental_SfM PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)

    add_executable(virtual_cam_incremental_SfM src/ceres_examples/virtual_cam_incremental_SfM.cpp)
    target_link_libraries(virtual_cam_incremental_SfM PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)

    add_executable(real_data_incremental_SfM src/ceres_examples/real_data_incremental_SfM.cpp)
    target_link_libraries(real_data_incremental_SfM PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)
endif()

add_executable(simple_snavely_reprojection_error src/ceres_examples/simple_snavely_reprojection_error.cpp)
target_link_libraries(simple_snavely_reprojection_error PRIVATE Ceres::ceres ${OpenCV_LIBS})

# Pose graph 2D (needs absl flags and logging)
add_executable(pose_graph_2d
    src/ceres_examples/slam/pose_graph_2d/angle_manifold.h
    src/ceres_examples/slam/pose_graph_2d/normalize_angle.h
    src/ceres_examples/slam/pose_graph_2d/pose_graph_2d.cc
    src/ceres_examples/slam/pose_graph_2d/pose_graph_2d_error_term.h
    src/ceres_examples/slam/pose_graph_2d/types.h
)
target_link_libraries(pose_graph_2d PRIVATE Ceres::ceres absl::flags absl::flags_parse absl::log absl::log_initialize)

# Rerun examples
if(BUILD_RERUN_EXAMPLES)
    add_executable(rerun src/rerun_examples/rerun.cpp)
    target_link_libraries(rerun PRIVATE rerun_sdk ${OpenCV_LIBS} Eigen3::Eigen)

    add_executable(minimal_camera_ray_test src/rerun_examples/minimal_camera_ray_test.cpp)
    target_link_libraries(minimal_camera_ray_test PRIVATE rerun_sdk ${OpenCV_LIBS} Eigen3::Eigen)
endif()

# manif examples (workaround for missing <cassert> in older manif versions)
set(MANIF_EXTRA_FLAGS "")
if(manif_VERSION VERSION_LESS_EQUAL "0.0.5")  # Adjust version if needed
    set(MANIF_EXTRA_FLAGS "-include;cassert")
endif()

add_executable(manif_se2_sam src/manif_examples/se2_sam.cpp)
target_compile_options(manif_se2_sam PRIVATE ${MANIF_EXTRA_FLAGS})
target_link_libraries(manif_se2_sam PRIVATE MANIF::manif)

add_executable(manif_SE3_examples src/manif_examples/SE3_examples.cpp)
target_compile_options(manif_SE3_examples PRIVATE ${MANIF_EXTRA_FLAGS})
target_link_libraries(manif_SE3_examples PRIVATE MANIF::manif)

add_executable(manif_se2_localization src/manif_examples/se2_localization.cpp)
target_compile_options(manif_se2_localization PRIVATE ${MANIF_EXTRA_FLAGS})
target_link_libraries(manif_se2_localization PRIVATE MANIF::manif)

# Sophus
add_executable(sophus_examples src/sophus_examples/useSophus.cpp)
target_link_libraries(sophus_examples PRIVATE Sophus::Sophus)

# Camera examples
# Conceptual zero-copy demo (portable, no kernel APIs)
add_executable(zero_copy_concept src/camera_examples/zero_copy_concept.cpp)
target_compile_features(zero_copy_concept PRIVATE cxx_std_20)
target_link_libraries(zero_copy_concept PRIVATE Threads::Threads)

# Single-producer multi-consumer frame fan-out (portable)
add_executable(spmc src/camera_examples/spmc.cpp)
target_compile_features(spmc PRIVATE cxx_std_20)
target_link_libraries(spmc PRIVATE Threads::Threads)

# Disruptor-style SPMC ring buffer (portable)
add_executable(spmc_ring src/camera_examples/spmc_ring.cpp)
target_compile_features(spmc_ring PRIVATE cxx_std_20)
target_link_libraries(spmc_ring PRIVATE Threads::Threads)

# V4L2 zero-copy capture (Linux only)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(v4l2_zero_copy_capture src/camera_examples/v4l2_zero_copy_capture.cpp)
    target_compile_features(v4l2_zero_copy_capture PRIVATE cxx_std_20)
endif()