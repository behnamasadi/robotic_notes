cmake_minimum_required(VERSION 3.16...3.27)
project(robotic-notes CXX)


if (NOT DEFINED CMAKE_TOOLCHAIN_FILE)
        get_filename_component(VCPKG_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake" ABSOLUTE)
        set(CMAKE_TOOLCHAIN_FILE "${VCPKG_TOOLCHAIN_FILE}" CACHE PATH "toolchain file")
else()
        # Ensure CMAKE_TOOLCHAIN_FILE is always absolute
        get_filename_component(CMAKE_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_FILE}" ABSOLUTE)
        set(CMAKE_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_FILE}" CACHE PATH "toolchain file" FORCE)
endif()
message("toolchain file: ${CMAKE_TOOLCHAIN_FILE}")


find_package(Boost REQUIRED)
message("Boost_VERSION: " ${Boost_VERSION})

# Ensure CMAKE_TOOLCHAIN_FILE is absolute before rerun_sdk uses it
# This is critical because rerun_sdk passes it to arrow_cpp, which needs an absolute path
if(CMAKE_TOOLCHAIN_FILE)
    get_filename_component(CMAKE_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_FILE}" ABSOLUTE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_FILE}" CACHE PATH "toolchain file" FORCE)
    message(STATUS "Using absolute toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
endif()

include(FetchContent)
FetchContent_Declare(rerun_sdk URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
set(FETCHCONTENT_QUIET OFF)

# Patch rerun_sdk's download_and_build_arrow.cmake to ensure absolute toolchain path
# This is needed because ExternalProject_Add passes CMAKE_TOOLCHAIN_FILE as-is,
# and arrow_cpp can't resolve relative paths from its build directory
FetchContent_GetProperties(rerun_sdk)
if(NOT rerun_sdk_POPULATED)
    FetchContent_Populate(rerun_sdk)
    if(CMAKE_TOOLCHAIN_FILE)
        get_filename_component(ABS_TOOLCHAIN "${CMAKE_TOOLCHAIN_FILE}" ABSOLUTE)
        # Read the file
        file(READ "${rerun_sdk_SOURCE_DIR}/download_and_build_arrow.cmake" ARROW_CMAKE_CONTENT)
        # Replace the CMAKE_TOOLCHAIN_FILE variable reference with the absolute path
        string(REPLACE 
            "-DCMAKE_TOOLCHAIN_FILE=\${CMAKE_TOOLCHAIN_FILE}"
            "-DCMAKE_TOOLCHAIN_FILE=${ABS_TOOLCHAIN}"
            ARROW_CMAKE_CONTENT "${ARROW_CMAKE_CONTENT}")
        # Write it back
        file(WRITE "${rerun_sdk_SOURCE_DIR}/download_and_build_arrow.cmake" "${ARROW_CMAKE_CONTENT}")
        message(STATUS "Patched rerun_sdk to use absolute toolchain path: ${ABS_TOOLCHAIN}")
    endif()
endif()

# Configure rerun_sdk to use Arrow from vcpkg instead of building it
# This avoids the abseil/GCC 13 compatibility issue
set(RERUN_DOWNLOAD_AND_BUILD_ARROW OFF CACHE BOOL "Disable Arrow download and build to use vcpkg version")

# Use FetchContent_Populate and add_subdirectory to ensure target is available
FetchContent_GetProperties(rerun_sdk)
if(NOT rerun_sdk_POPULATED)
    FetchContent_Populate(rerun_sdk)
    if(CMAKE_TOOLCHAIN_FILE)
        get_filename_component(ABS_TOOLCHAIN "${CMAKE_TOOLCHAIN_FILE}" ABSOLUTE)
        # Read the file
        file(READ "${rerun_sdk_SOURCE_DIR}/download_and_build_arrow.cmake" ARROW_CMAKE_CONTENT)
        # Replace the CMAKE_TOOLCHAIN_FILE variable reference with the absolute path
        string(REPLACE 
            "-DCMAKE_TOOLCHAIN_FILE=\${CMAKE_TOOLCHAIN_FILE}"
            "-DCMAKE_TOOLCHAIN_FILE=${ABS_TOOLCHAIN}"
            ARROW_CMAKE_CONTENT "${ARROW_CMAKE_CONTENT}")
        # Write it back
        file(WRITE "${rerun_sdk_SOURCE_DIR}/download_and_build_arrow.cmake" "${ARROW_CMAKE_CONTENT}")
        message(STATUS "Patched rerun_sdk to use absolute toolchain path: ${ABS_TOOLCHAIN}")
    endif()
    # Add the subdirectory to make the target available
    add_subdirectory(${rerun_sdk_SOURCE_DIR} ${rerun_sdk_BINARY_DIR})
else()
    # Already populated, just add subdirectory
    add_subdirectory(${rerun_sdk_SOURCE_DIR} ${rerun_sdk_BINARY_DIR})
endif()

# Verify that rerun_sdk target exists
if(NOT TARGET rerun_sdk)
    message(FATAL_ERROR "rerun_sdk target not found after adding subdirectory")
endif()

# Get rerun_sdk source directory for include paths (needed for targets that use rerun)
FetchContent_GetProperties(rerun_sdk)
if(rerun_sdk_SOURCE_DIR)
    set(RERUN_SDK_INCLUDE_DIR "${rerun_sdk_SOURCE_DIR}/src" CACHE INTERNAL "Rerun SDK include directory")
else()
    # Fallback if not populated yet
    set(RERUN_SDK_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/rerun_sdk-src/src" CACHE INTERNAL "Rerun SDK include directory")
endif()


find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
message("OpenCV_VERSION: " ${OpenCV_VERSION})

find_package(Ceres CONFIG REQUIRED)
find_package(manif CONFIG REQUIRED)
find_package(Sophus CONFIG REQUIRED)



############################ ceres ############################

add_executable(ceres_automatic_diff_dual_number src/ceres_examples/automatic_diff_dual_number.cpp)
target_link_libraries(ceres_automatic_diff_dual_number PRIVATE Ceres::ceres)

add_executable(ceres_jet_class src/ceres_examples/ceres_jet_class.cpp)
target_link_libraries(ceres_jet_class PRIVATE Ceres::ceres)

add_executable(ceres_example src/ceres_examples/ceres_example.cpp)
target_link_libraries(ceres_example PRIVATE Ceres::ceres)

add_executable(ceres_exponential_curve_fitting_example src/ceres_examples/ceres_exponential_curve_fitting_example.cpp)
target_link_libraries(ceres_exponential_curve_fitting_example PRIVATE Ceres::ceres)

add_executable(bundle_adjuster src/ceres_examples/bundle_adjuster.cpp)
target_include_directories(bundle_adjuster PRIVATE ${RERUN_SDK_INCLUDE_DIR})
target_link_libraries(bundle_adjuster PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)

add_executable(snavely_reprojection_error src/ceres_examples/snavely_reprojection_error.cpp)
target_include_directories(snavely_reprojection_error PRIVATE ${RERUN_SDK_INCLUDE_DIR})
target_link_libraries(snavely_reprojection_error PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)



add_executable(simple_snavely_reprojection_error src/ceres_examples/simple_snavely_reprojection_error.cpp)
target_link_libraries(simple_snavely_reprojection_error PRIVATE Ceres::ceres ${OpenCV_LIBS} )


add_executable(incremental_SfM  src/ceres_examples/incremental_SfM.cpp)
target_include_directories(incremental_SfM PRIVATE ${RERUN_SDK_INCLUDE_DIR})
target_link_libraries(incremental_SfM  PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)

add_executable(virtual_cam_incremental_SfM  src/ceres_examples/virtual_cam_incremental_SfM.cpp)
target_include_directories(virtual_cam_incremental_SfM PRIVATE ${RERUN_SDK_INCLUDE_DIR})
target_link_libraries(virtual_cam_incremental_SfM  PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)

add_executable(real_data_incremental_SfM  src/ceres_examples/real_data_incremental_SfM.cpp)
target_include_directories(real_data_incremental_SfM PRIVATE ${RERUN_SDK_INCLUDE_DIR})
target_link_libraries(real_data_incremental_SfM  PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)



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
target_include_directories(rerun PRIVATE ${RERUN_SDK_INCLUDE_DIR})
target_link_libraries(rerun PRIVATE rerun_sdk ${OpenCV_LIBS})


add_executable(minimal_camera_ray_test src/rerun_examples/minimal_camera_ray_test.cpp)
set_property(TARGET minimal_camera_ray_test PROPERTY CXX_STANDARD 17)
target_include_directories(minimal_camera_ray_test PRIVATE ${RERUN_SDK_INCLUDE_DIR})
target_link_libraries(minimal_camera_ray_test PRIVATE rerun_sdk ${OpenCV_LIBS})





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
