cmake_minimum_required(VERSION 3.14)
project(rtabmap_unity_bridge)

set(CMAKE_CXX_STANDARD 17)

# Find RTAB-Map (core + utilite only, no GUI)
find_package(RTABMap REQUIRED COMPONENTS core)

# Find OpenCV (for imshow visualization)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs)

add_executable(rtabmap_unity_offline main.cpp)

target_link_libraries(rtabmap_unity_offline
    rtabmap::core
    rtabmap::utilite
    ${OpenCV_LIBS}
)

# Windows: post-build info
if(WIN32)
    add_custom_command(TARGET rtabmap_unity_offline POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "Build complete: rtabmap_unity_offline.exe"
    )
endif()
