cmake_minimum_required(VERSION 3.14)
project(rtabmap_pipeline)

set(CMAKE_CXX_STANDARD 17)

# Find RTAB-Map (core + utilite only, no GUI)
find_package(RTABMap REQUIRED COMPONENTS core)

# Find OpenCV (for imshow visualization)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs)

add_executable(rtabmap_pipeline main.cpp)

target_link_libraries(rtabmap_pipeline
    rtabmap::core
    rtabmap::utilite
    ${OpenCV_LIBS}
)

if(WIN32)
    add_custom_command(TARGET rtabmap_pipeline POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "Build complete: rtabmap_pipeline.exe"
    )
endif()
