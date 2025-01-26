cmake_minimum_required(VERSION 3.16...3.27)
project(slam-tutorials CXX)


if (NOT DEFINED CMAKE_TOOLCHAIN_FILE)
        set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE PATH "toolchain file")
endif()
message("toolchain file: ${CMAKE_TOOLCHAIN_FILE}")


find_package(Boost REQUIRED)
message("Boost_VERSION: " ${Boost_VERSION})

include(FetchContent)
FetchContent_Declare(rerun_sdk URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)


find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
message("OpenCV_VERSION: " ${OpenCV_VERSION})

find_package(Ceres CONFIG REQUIRED)
find_package(manif CONFIG REQUIRED)
find_package(Sophus CONFIG REQUIRED)



############################ ceres ############################

add_executable(ceres_example src/ceres_examples/ceres_example.cpp)
target_link_libraries(ceres_example PRIVATE Ceres::ceres)



add_executable(ceres_exponential_curve_fitting_example src/ceres_examples/ceres_exponential_curve_fitting_example.cpp)
target_link_libraries(ceres_exponential_curve_fitting_example PRIVATE Ceres::ceres)

add_executable(bundle_adjuster src/ceres_examples/bundle_adjuster.cpp)
target_link_libraries(bundle_adjuster PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)


add_executable(snavely_reprojection_error src/ceres_examples/snavely_reprojection_error.cpp)
target_link_libraries(snavely_reprojection_error PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)



add_executable(simple_snavely_reprojection_error src/ceres_examples/simple_snavely_reprojection_error.cpp)
target_link_libraries(simple_snavely_reprojection_error PRIVATE Ceres::ceres ${OpenCV_LIBS} )


add_executable(incremental_SfM  src/ceres_examples/incremental_SfM.cpp)
target_link_libraries(incremental_SfM  PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)

add_executable(virtual_cam_incremental_SfM  src/ceres_examples/virtual_cam_incremental_SfM.cpp)
target_link_libraries(virtual_cam_incremental_SfM  PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)



add_executable(pose_graph_2d
  src/ceres_examples/slam/pose_graph_2d/angle_manifold.h
  src/ceres_examples/slam/pose_graph_2d/normalize_angle.h
  src/ceres_examples/slam/pose_graph_2d/pose_graph_2d.cc
  src/ceres_examples/slam/pose_graph_2d/pose_graph_2d_error_term.h
  src/ceres_examples/slam/pose_graph_2d/types.h)
target_link_libraries(pose_graph_2d PRIVATE absl::flags absl::flags_parse absl::log absl::check absl::log_initialize Ceres::ceres)


############################ rerun ############################

add_executable(rerun src/rerun_examples/rerun.cpp)
set_property(TARGET rerun PROPERTY CXX_STANDARD 17)
target_link_libraries(rerun PRIVATE rerun_sdk  ${OpenCV_LIBS})

############################ manif ############################
add_executable(manif_se2_sam src/manif_examples/se2_sam.cpp)
target_link_libraries(manif_se2_sam PRIVATE MANIF::manif)

add_executable(manif_SE3_examples src/manif_examples/SE3_examples.cpp)
target_link_libraries(manif_SE3_examples PRIVATE MANIF::manif)

add_executable(manif_se2_localization src/manif_examples/se2_localization.cpp)
target_link_libraries(manif_se2_localization PRIVATE MANIF::manif)


#add_executable(so2_average src/manif_examples/so2_average.cpp)
#add_executable(se2_average src/manif_examples/se2_average.cpp)
#add_executable(se2_interpolation src/manif_examples/se2_interpolation.cpp)
#add_executable(se2_decasteljau src/manif_examples/se2_DeCasteljau.cpp)
#add_executable(se2_localization src/manif_examples/se2_localization.cpp)
#add_executable(se2_localization_ukfm src/manif_examples/se2_localization_ukfm.cpp)
#add_executable(se2_sam src/manif_examples/se2_sam.cpp)
#add_executable(se3_localization src/manif_examples/se3_localization.cpp)
#add_executable(se3_sam src/manif_examples/se3_sam.cpp)
#add_executable(se3_sam_selfcalib src/manif_examples/se3_sam_selfcalib.cpp)
#add_executable(se_2_3_localization src/manif_examples/se_2_3_localization.cpp)

############################ sophus ############################

add_executable(sophus_examples src/sophus_examples/useSophus.cpp)
target_link_libraries(sophus_examples PRIVATE Sophus::Sophus)
