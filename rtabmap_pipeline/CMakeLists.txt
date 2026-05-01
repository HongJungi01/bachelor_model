cmake_minimum_required(VERSION 3.14)
project(rtabmap_pipeline)

set(CMAKE_CXX_STANDARD 17)

# Set paths
set(VCPKG_PATH "C:/dev/vcpkg_export/installed/x64-windows-release")
set(RTABMAP_PATH "C:/dev/rtabmap")

# Add RTABMap build directory to CMake module path for finding dependencies
list(APPEND CMAKE_PREFIX_PATH
    "${RTABMAP_PATH}/build"
    "${VCPKG_PATH}"
    "${RTABMAP_PATH}/cmake_modules"
)

# Find OpenCV first
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs)

# JSON for the semantic VLM sidecar (cpp-httplib is header-only and lives at
# ${VCPKG_PATH}/include/httplib.h — already covered by target_include_directories).
find_package(nlohmann_json CONFIG REQUIRED)

# Link against RTABMap libraries directly
add_executable(rtabmap_pipeline
    main.cpp
    GridPublisher.h
    CameraUnityTCP.h
    SemanticBackproject.h
    SemanticMaskStore.h
    SemanticWorker.h
)

target_include_directories(rtabmap_pipeline PRIVATE
    "${RTABMAP_PATH}/corelib/include"
    "${RTABMAP_PATH}/utilite/include"
    "${RTABMAP_PATH}/build/corelib/src/include"   # generated rtabmap_core_export.h, Version.h
    "${RTABMAP_PATH}/build/utilite/src/include"   # generated utilite_export.h
    "${VCPKG_PATH}/include"
    "${VCPKG_PATH}/include/eigen3"
    "${VCPKG_PATH}/include/opencv4"
)

target_link_libraries(rtabmap_pipeline
    "${RTABMAP_PATH}/build/lib/rtabmap_core.lib"
    "${RTABMAP_PATH}/build/lib/rtabmap_utilite.lib"
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
    # Additional PCL and other dependencies that rtabmap_core may need
    "${VCPKG_PATH}/lib/pcl_common.lib"
    "${VCPKG_PATH}/lib/pcl_io.lib"
    "${VCPKG_PATH}/lib/pcl_kdtree.lib"
    "${VCPKG_PATH}/lib/pcl_search.lib"
    "${VCPKG_PATH}/lib/pcl_registration.lib"
    "${VCPKG_PATH}/lib/pcl_features.lib"
    "${VCPKG_PATH}/lib/pcl_filters.lib"
    "${VCPKG_PATH}/lib/pcl_segmentation.lib"
    "${VCPKG_PATH}/lib/pcl_surface.lib"
)

# POSIX threads for std::thread in CameraUnityTCP
if(NOT WIN32)
    find_package(Threads REQUIRED)
    target_link_libraries(rtabmap_pipeline Threads::Threads)
endif()

if(WIN32)
    add_custom_command(TARGET rtabmap_pipeline POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "Build complete: rtabmap_pipeline.exe"
    )
endif()
