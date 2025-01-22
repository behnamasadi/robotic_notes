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





############################ ceres ############################

add_executable(ceres_example src/ceres_examples/ceres_example.cpp)
target_link_libraries(ceres_example PRIVATE Ceres::ceres)

add_executable(bundle_adjuster src/ceres_examples/bundle_adjuster.cpp)
target_link_libraries(bundle_adjuster PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)


add_executable(snavely_reprojection_error src/ceres_examples/snavely_reprojection_error.cpp)
target_link_libraries(snavely_reprojection_error PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)


add_executable(incremental_SfM  src/ceres_examples/incremental_SfM.cpp)
target_link_libraries(incremental_SfM  PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)

add_executable(virtual_cam_incremental_SfM  src/ceres_examples/virtual_cam_incremental_SfM.cpp)
target_link_libraries(virtual_cam_incremental_SfM  PRIVATE Ceres::ceres ${OpenCV_LIBS} rerun_sdk)



############################ rerun ############################

add_executable(rerun src/rerun_examples/rerun.cpp)
set_property(TARGET rerun PROPERTY CXX_STANDARD 17)
target_link_libraries(rerun PRIVATE rerun_sdk  ${OpenCV_LIBS})


