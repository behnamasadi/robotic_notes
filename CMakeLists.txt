cmake_minimum_required(VERSION 3.20)  # Higher version for better vcpkg integration
project(robotic-notes LANGUAGES CXX)

# ------------------------------------------------------------------
# vcpkg toolchain (standard way)
# ------------------------------------------------------------------
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()

# Use release-only triplet to save time/space (optional but recommended)
set(VCPKG_TARGET_TRIPLET "x64-linux-release" CACHE STRING "")

# ------------------------------------------------------------------
# Dependencies via vcpkg + FetchContent for rerun_sdk
# ------------------------------------------------------------------
find_package(Boost REQUIRED COMPONENTS iostreams filesystem)
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(manif CONFIG REQUIRED)
find_package(Sophus CONFIG REQUIRED)
find_package(Arrow CONFIG REQUIRED)     # Should work out-of-the-box now
find_package(Threads REQUIRED)

include(FetchContent)

FetchContent_Declare(
    rerun_sdk
    URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)

# Prefer vcpkg Arrow if available, otherwise let rerun build it
if(Arrow_FOUND)
    set(RERUN_DOWNLOAD_AND_BUILD_ARROW OFF CACHE BOOL "Use system/vcpkg Arrow")
else()
    set(RERUN_DOWNLOAD_AND_BUILD_ARROW ON CACHE BOOL "Build Arrow inside rerun_sdk")
endif()

FetchContent_MakeAvailable(rerun_sdk)

# ------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------

# Helper macro to reduce repetition
macro(add_app name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)
    target_include_directories(${name} PRIVATE "${rerun_sdk_SOURCE_DIR}/src")
    target_compile_features(${name} PRIVATE cxx_std_17)
endmacro()

# Ceres examples
add_executable(ceres_automatic_diff_dual_number src/ceres_examples/automatic_diff_dual_number.cpp)
target_link_libraries(ceres_automatic_diff_dual_number PRIVATE Ceres::ceres)

add_executable(ceres_jet_class src/ceres_examples/ceres_jet_class.cpp)
target_link_libraries(ceres_jet_class PRIVATE Ceres::ceres)

add_executable(ceres_example src/ceres_examples/ceres_example.cpp)
target_link_libraries(ceres_example PRIVATE Ceres::ceres)

add_executable(ceres_exponential_curve_fitting_example src/ceres_examples/ceres_exponential_curve_fitting_example.cpp)
target_link_libraries(ceres_exponential_curve_fitting_example PRIVATE Ceres::ceres)

add_app(bundle_adjuster src/ceres_examples/bundle_adjuster.cpp)
add_app(snavely_reprojection_error src/ceres_examples/snavely_reprojection_error.cpp)
add_app(incremental_SfM src/ceres_examples/incremental_SfM.cpp)
add_app(virtual_cam_incremental_SfM src/ceres_examples/virtual_cam_incremental_SfM.cpp)
add_app(real_data_incremental_SfM src/ceres_examples/real_data_incremental_SfM.cpp)

add_executable(simple_snavely_reprojection_error src/ceres_examples/simple_snavely_reprojection_error.cpp)
target_link_libraries(simple_snavely_reprojection_error PRIVATE Ceres::ceres ${OpenCV_LIBS})

# Pose graph 2D (needs absl flags)
add_executable(pose_graph_2d
    src/ceres_examples/slam/pose_graph_2d/angle_manifold.h
    src/ceres_examples/slam/pose_graph_2d/normalize_angle.h
    src/ceres_examples/slam/pose_graph_2d/pose_graph_2d.cc
    src/ceres_examples/slam/pose_graph_2d/pose_graph_2d_error_term.h
    src/ceres_examples/slam/pose_graph_2d/types.h
)
target_link_libraries(pose_graph_2d PRIVATE Ceres::ceres absl::flags absl::flags_parse)

# Rerun examples
add_executable(rerun src/rerun_examples/rerun.cpp)
target_link_libraries(rerun PRIVATE rerun_sdk ${OpenCV_LIBS} Eigen3::Eigen)
target_include_directories(rerun PRIVATE "${rerun_sdk_SOURCE_DIR}/src")
target_compile_features(rerun PRIVATE cxx_std_17)

add_executable(minimal_camera_ray_test src/rerun_examples/minimal_camera_ray_test.cpp)
target_link_libraries(minimal_camera_ray_test PRIVATE rerun_sdk ${OpenCV_LIBS} Eigen3::Eigen)
target_include_directories(minimal_camera_ray_test PRIVATE "${rerun_sdk_SOURCE_DIR}/src")
target_compile_features(minimal_camera_ray_test PRIVATE cxx_std_17)

# manif examples (workaround for missing <cassert> in older manif versions)
set(MANIF_EXTRA_FLAGS "")
if(manif_VERSION VERSION_LESS "0.0.5")  # Adjust version if needed
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