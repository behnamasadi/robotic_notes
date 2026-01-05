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

# Configure vcpkg to build only release packages (not debug) to save build time and disk space
if(NOT DEFINED VCPKG_TARGET_TRIPLET)
    set(VCPKG_TARGET_TRIPLET "x64-linux-release" CACHE STRING "vcpkg target triplet")
    message(STATUS "Setting vcpkg triplet to: ${VCPKG_TARGET_TRIPLET} (release only)")
endif()

# Arrow's config file references OpenSSL::Crypto, so we need to ensure OpenSSL is found
# before Arrow's config is processed. We'll patch Arrow's config to find OpenSSL first.
# This is done after vcpkg installs packages, so the config files are available.

find_package(Boost REQUIRED)
message("Boost_VERSION: " ${Boost_VERSION})

# Ensure Arrow dependencies' targets exist before Arrow is used
# Arrow's ArrowTargets.cmake references: OpenSSL, Brotli, BZip2, LZ4, Snappy, ZLIB, zstd
# Create them manually to avoid find_package recursion issues
set(VCPKG_LIB_DIR "${CMAKE_BINARY_DIR}/vcpkg_installed/x64-linux-release/lib")
set(VCPKG_INCLUDE_DIR "${CMAKE_BINARY_DIR}/vcpkg_installed/x64-linux-release/include")

# OpenSSL targets
if(NOT TARGET OpenSSL::Crypto AND EXISTS "${VCPKG_LIB_DIR}/libcrypto.a")
    add_library(OpenSSL::Crypto STATIC IMPORTED)
    set_target_properties(OpenSSL::Crypto PROPERTIES
        IMPORTED_LOCATION "${VCPKG_LIB_DIR}/libcrypto.a"
        INTERFACE_INCLUDE_DIRECTORIES "${VCPKG_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "dl;pthread")
    add_library(OpenSSL::SSL STATIC IMPORTED)
    set_target_properties(OpenSSL::SSL PROPERTIES
        IMPORTED_LOCATION "${VCPKG_LIB_DIR}/libssl.a"
        INTERFACE_INCLUDE_DIRECTORIES "${VCPKG_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "OpenSSL::Crypto")
    message(STATUS "Created OpenSSL targets manually")
endif()

# Brotli targets
if(NOT TARGET Brotli::brotlienc AND EXISTS "${VCPKG_LIB_DIR}/libbrotlienc.a")
    add_library(Brotli::brotlienc STATIC IMPORTED)
    set_target_properties(Brotli::brotlienc PROPERTIES
        IMPORTED_LOCATION "${VCPKG_LIB_DIR}/libbrotlienc.a"
        INTERFACE_INCLUDE_DIRECTORIES "${VCPKG_INCLUDE_DIR}")
    add_library(Brotli::brotlidec STATIC IMPORTED)
    set_target_properties(Brotli::brotlidec PROPERTIES
        IMPORTED_LOCATION "${VCPKG_LIB_DIR}/libbrotlidec.a"
        INTERFACE_INCLUDE_DIRECTORIES "${VCPKG_INCLUDE_DIR}")
    add_library(Brotli::brotlicommon STATIC IMPORTED)
    set_target_properties(Brotli::brotlicommon PROPERTIES
        IMPORTED_LOCATION "${VCPKG_LIB_DIR}/libbrotlicommon.a"
        INTERFACE_INCLUDE_DIRECTORIES "${VCPKG_INCLUDE_DIR}")
    message(STATUS "Created Brotli targets manually")
endif()

# BZip2 target
if(NOT TARGET BZip2::BZip2 AND EXISTS "${VCPKG_LIB_DIR}/libbz2.a")
    add_library(BZip2::BZip2 STATIC IMPORTED)
    set_target_properties(BZip2::BZip2 PROPERTIES
        IMPORTED_LOCATION "${VCPKG_LIB_DIR}/libbz2.a"
        INTERFACE_INCLUDE_DIRECTORIES "${VCPKG_INCLUDE_DIR}")
    message(STATUS "Created BZip2 target manually")
endif()

# LZ4 target
if(NOT TARGET LZ4::lz4 AND EXISTS "${VCPKG_LIB_DIR}/liblz4.a")
    add_library(LZ4::lz4 STATIC IMPORTED)
    set_target_properties(LZ4::lz4 PROPERTIES
        IMPORTED_LOCATION "${VCPKG_LIB_DIR}/liblz4.a"
        INTERFACE_INCLUDE_DIRECTORIES "${VCPKG_INCLUDE_DIR}")
    message(STATUS "Created LZ4 target manually")
endif()

# Snappy target
if(NOT TARGET Snappy::snappy-static AND EXISTS "${VCPKG_LIB_DIR}/libsnappy.a")
    add_library(Snappy::snappy-static STATIC IMPORTED)
    set_target_properties(Snappy::snappy-static PROPERTIES
        IMPORTED_LOCATION "${VCPKG_LIB_DIR}/libsnappy.a"
        INTERFACE_INCLUDE_DIRECTORIES "${VCPKG_INCLUDE_DIR}")
    message(STATUS "Created Snappy target manually")
endif()

# ZLIB target (usually already exists, but ensure it's there)
if(NOT TARGET ZLIB::ZLIB AND EXISTS "${VCPKG_LIB_DIR}/libz.a")
    add_library(ZLIB::ZLIB STATIC IMPORTED)
    set_target_properties(ZLIB::ZLIB PROPERTIES
        IMPORTED_LOCATION "${VCPKG_LIB_DIR}/libz.a"
        INTERFACE_INCLUDE_DIRECTORIES "${VCPKG_INCLUDE_DIR}")
    message(STATUS "Created ZLIB target manually")
endif()

# zstd target
if(NOT TARGET zstd::libzstd_static AND EXISTS "${VCPKG_LIB_DIR}/libzstd.a")
    add_library(zstd::libzstd_static STATIC IMPORTED)
    set_target_properties(zstd::libzstd_static PROPERTIES
        IMPORTED_LOCATION "${VCPKG_LIB_DIR}/libzstd.a"
        INTERFACE_INCLUDE_DIRECTORIES "${VCPKG_INCLUDE_DIR}")
    message(STATUS "Created zstd target manually")
endif()

# Threads target (standard CMake, should be available but ensure it's found)
find_package(Threads REQUIRED)

# Workaround for vcpkg absl installation issue where multiple absl libraries are missing
# This creates minimal stub libraries to satisfy CMake's file existence check
# Known missing libraries: libabsl_log_severity.a, libabsl_raw_logging_internal.a, and potentially others
set(ABSL_TARGETS_FILE "${CMAKE_BINARY_DIR}/vcpkg_installed/x64-linux-release/share/absl/abslTargets.cmake")
if(EXISTS "${ABSL_TARGETS_FILE}")
    # Function to create a stub library if it's missing
    function(create_absl_stub_lib lib_name)
        set(lib_path "${VCPKG_LIB_DIR}/${lib_name}")
        if(NOT EXISTS "${lib_path}")
            # Create a minimal stub C file and compile it to an object, then add to archive
            get_filename_component(lib_basename "${lib_name}" NAME_WE)
            set(STUB_SRC "${CMAKE_BINARY_DIR}/${lib_basename}_stub.c")
            set(STUB_OBJ "${CMAKE_BINARY_DIR}/${lib_basename}_stub.o")
            file(WRITE "${STUB_SRC}" "/* Stub for missing ${lib_name} - vcpkg absl installation workaround */\nvoid __attribute__((weak)) ${lib_basename}_stub(void) {}\n")
            
            # Try to compile the stub and create the archive
            find_program(CC_TOOL ${CMAKE_C_COMPILER} cc gcc)
            find_program(AR_TOOL ar)
            if(CC_TOOL AND AR_TOOL)
                execute_process(
                    COMMAND ${CC_TOOL} -c "${STUB_SRC}" -o "${STUB_OBJ}"
                    RESULT_VARIABLE CC_RESULT
                    ERROR_QUIET OUTPUT_QUIET
                )
                if(CC_RESULT EQUAL 0 AND EXISTS "${STUB_OBJ}")
                    execute_process(
                        COMMAND ${AR_TOOL} -rc "${lib_path}" "${STUB_OBJ}"
                        RESULT_VARIABLE AR_RESULT
                        ERROR_QUIET OUTPUT_QUIET
                    )
                    if(AR_RESULT EQUAL 0 AND EXISTS "${lib_path}")
                        message(STATUS "Created stub ${lib_name} to work around vcpkg absl installation issue")
                        # Clean up temporary files
                        file(REMOVE "${STUB_SRC}" "${STUB_OBJ}")
                        return()
                    endif()
                endif()
                # Clean up on failure
                file(REMOVE "${STUB_SRC}")
                if(EXISTS "${STUB_OBJ}")
                    file(REMOVE "${STUB_OBJ}")
                endif()
            endif()
        endif()
    endfunction()
    
    # Create stubs for known missing libraries
    # vcpkg's absl installation appears to be missing multiple library files
    # We create stubs for all known missing libraries
    create_absl_stub_lib("libabsl_log_severity.a")
    create_absl_stub_lib("libabsl_raw_logging_internal.a")
    create_absl_stub_lib("libabsl_spinlock_wait.a")
    
    message(WARNING "Created stub absl libraries - this is a workaround for vcpkg absl installation bug")
endif()

# Create/update ArrowConfig.cmake (vcpkg Arrow package sometimes doesn't include it)
# ArrowConfig.cmake is needed for find_package(Arrow) to work
# We create it with minimal dependencies since we create targets manually above
# Always regenerate to ensure correct content (overwrites any manual edits)
set(ARROW_CONFIG_FILE "${CMAKE_BINARY_DIR}/vcpkg_installed/x64-linux-release/share/arrow/ArrowConfig.cmake")
set(ARROW_TARGETS_FILE "${CMAKE_BINARY_DIR}/vcpkg_installed/x64-linux-release/share/arrow/ArrowTargets.cmake")
if(EXISTS "${ARROW_TARGETS_FILE}")
    file(WRITE "${ARROW_CONFIG_FILE}" 
"# ArrowConfig.cmake - Auto-generated to fix missing/broken config file
# DO NOT EDIT MANUALLY - This file is regenerated by CMakeLists.txt
set(Arrow_VERSION \"21.0.0\")
set(Arrow_INCLUDE_DIR \"\${CMAKE_CURRENT_LIST_DIR}/../../include\")
set(Arrow_LIB_DIR \"\${CMAKE_CURRENT_LIST_DIR}/../../lib\")
# Dependencies (OpenSSL, Brotli, BZip2, LZ4, Snappy, ZLIB, zstd, Threads) are created
# manually in main CMakeLists.txt to avoid recursion and missing CONFIG files
# Just include ArrowTargets which references the manually created targets
include(\"\${CMAKE_CURRENT_LIST_DIR}/ArrowTargets.cmake\")
set(Arrow_FOUND TRUE)
")
    message(STATUS "Created/updated ArrowConfig.cmake (dependencies created manually in CMakeLists.txt)")
endif()

# Fix arrow-config.cmake to prevent infinite recursion
# arrow-config.cmake calls find_package(Arrow CONFIG) which causes recursion with vcpkg
set(ARROW_CONFIG_DEPRECATED "${CMAKE_BINARY_DIR}/vcpkg_installed/x64-linux-release/share/arrow/arrow-config.cmake")
if(EXISTS "${ARROW_CONFIG_DEPRECATED}")
    file(READ "${ARROW_CONFIG_DEPRECATED}" ARROW_DEPRECATED_CONTENT)
    # Replace find_package(Arrow CONFIG) with direct include to avoid recursion
    if(ARROW_DEPRECATED_CONTENT MATCHES "find_package\\(Arrow CONFIG\\)")
        string(REPLACE 
            "find_package(Arrow CONFIG)"
            "include(\"${CMAKE_CURRENT_LIST_DIR}/ArrowConfig.cmake\")"
            ARROW_DEPRECATED_CONTENT "${ARROW_DEPRECATED_CONTENT}")
        file(WRITE "${ARROW_CONFIG_DEPRECATED}" "${ARROW_DEPRECATED_CONTENT}")
        message(STATUS "Patched arrow-config.cmake to prevent recursion")
    endif()
endif()

# Set Arrow_DIR so CMake can find Arrow's config file
# This must be done BEFORE rerun_sdk tries to find Arrow
# Use ARROW_TARGETS_FILE which was set earlier - if it exists, Arrow is installed
set(ARROW_SHARE_DIR "${CMAKE_BINARY_DIR}/vcpkg_installed/x64-linux-release/share/arrow")
if(EXISTS "${ARROW_TARGETS_FILE}")
    # Arrow is installed, ensure Arrow_DIR is set so CMake can find it
    # Set in cache so it's visible to all subdirectories (including rerun_sdk)
    set(Arrow_DIR "${ARROW_SHARE_DIR}" CACHE PATH "Arrow config directory" FORCE)
    # Also set as a normal variable so it's immediately available
    set(Arrow_DIR "${ARROW_SHARE_DIR}")
    list(APPEND CMAKE_PREFIX_PATH "${ARROW_SHARE_DIR}")
    message(STATUS "Set Arrow_DIR to: ${Arrow_DIR}")
    message(STATUS "ArrowConfig.cmake should exist at: ${ARROW_CONFIG_FILE}")
    
    # Verify config file exists (we created it above)
    if(EXISTS "${ARROW_CONFIG_FILE}")
        message(STATUS "ArrowConfig.cmake exists")
    else()
        message(WARNING "ArrowConfig.cmake not found at ${ARROW_CONFIG_FILE} even though ArrowTargets.cmake exists")
    endif()
    
    # Find Arrow package BEFORE rerun_sdk tries to use it
    # This ensures Arrow is available when rerun_sdk's CMakeLists.txt runs
    find_package(Arrow CONFIG QUIET)
    if(Arrow_FOUND)
        message(STATUS "Arrow found successfully before rerun_sdk")
    else()
        message(WARNING "Arrow not found via find_package, but ArrowTargets.cmake exists - rerun_sdk may fail")
        message(STATUS "Arrow_DIR is: ${Arrow_DIR}")
        message(STATUS "CMAKE_PREFIX_PATH includes: ${CMAKE_PREFIX_PATH}")
    endif()
else()
    message(WARNING "ArrowTargets.cmake not found at ${ARROW_TARGETS_FILE} - Arrow may not be installed by vcpkg")
endif()

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

# Configure rerun_sdk to use Arrow from vcpkg if available, otherwise let it build Arrow
# Check if Arrow is actually installed - if not, let rerun_sdk build it
if(EXISTS "${ARROW_TARGETS_FILE}")
    # Arrow is installed by vcpkg, use it
    set(RERUN_DOWNLOAD_AND_BUILD_ARROW OFF CACHE BOOL "Disable Arrow download and build to use vcpkg version")
    message(STATUS "Arrow found in vcpkg, disabling rerun_sdk Arrow build")
else()
    # Arrow is NOT installed, let rerun_sdk build it
    set(RERUN_DOWNLOAD_AND_BUILD_ARROW ON CACHE BOOL "Enable Arrow download and build since vcpkg version not available")
    message(STATUS "Arrow NOT found in vcpkg, enabling rerun_sdk Arrow build")
endif()

# Note: OpenSSL is in vcpkg.json dependencies and will be installed by vcpkg
# Arrow requires OpenSSL::Crypto, which will be available when vcpkg processes packages
# We'll patch Arrow's config to find OpenSSL before it's used (see below after vcpkg install)

# Use FetchContent_Populate and add_subdirectory to ensure target is available
FetchContent_GetProperties(rerun_sdk)
if(NOT rerun_sdk_POPULATED)
    FetchContent_Populate(rerun_sdk)
    if(CMAKE_TOOLCHAIN_FILE)
        get_filename_component(ABS_TOOLCHAIN "${CMAKE_TOOLCHAIN_FILE}" ABSOLUTE)
        # Patch rerun_sdk's download_and_build_arrow.cmake to ensure absolute toolchain path
        # Only patch if the file exists (it might not if Arrow is from vcpkg)
        if(EXISTS "${rerun_sdk_SOURCE_DIR}/download_and_build_arrow.cmake")
            file(READ "${rerun_sdk_SOURCE_DIR}/download_and_build_arrow.cmake" ARROW_CMAKE_CONTENT)
            string(REPLACE 
                "-DCMAKE_TOOLCHAIN_FILE=\${CMAKE_TOOLCHAIN_FILE}"
                "-DCMAKE_TOOLCHAIN_FILE=${ABS_TOOLCHAIN}"
                ARROW_CMAKE_CONTENT "${ARROW_CMAKE_CONTENT}")
            file(WRITE "${rerun_sdk_SOURCE_DIR}/download_and_build_arrow.cmake" "${ARROW_CMAKE_CONTENT}")
            message(STATUS "Patched rerun_sdk to use absolute toolchain path: ${ABS_TOOLCHAIN}")
        endif()
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
target_link_libraries(rerun PRIVATE rerun_sdk ${OpenCV_LIBS} Eigen3::Eigen)


add_executable(minimal_camera_ray_test src/rerun_examples/minimal_camera_ray_test.cpp)
set_property(TARGET minimal_camera_ray_test PROPERTY CXX_STANDARD 17)
target_include_directories(minimal_camera_ray_test PRIVATE ${RERUN_SDK_INCLUDE_DIR})
target_link_libraries(minimal_camera_ray_test PRIVATE rerun_sdk ${OpenCV_LIBS} Eigen3::Eigen)





############################ manif ############################
# Workaround for manif library missing #include <cassert> in headers
# Force include cassert before any other headers
set(MANIF_COMPILE_FLAGS "-include" "cassert")

add_executable(manif_se2_sam src/manif_examples/se2_sam.cpp)
target_compile_options(manif_se2_sam PRIVATE ${MANIF_COMPILE_FLAGS})
target_link_libraries(manif_se2_sam PRIVATE MANIF::manif)

add_executable(manif_SE3_examples src/manif_examples/SE3_examples.cpp)
target_compile_options(manif_SE3_examples PRIVATE ${MANIF_COMPILE_FLAGS})
target_link_libraries(manif_SE3_examples PRIVATE MANIF::manif)

add_executable(manif_se2_localization src/manif_examples/se2_localization.cpp)
target_compile_options(manif_se2_localization PRIVATE ${MANIF_COMPILE_FLAGS})
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
