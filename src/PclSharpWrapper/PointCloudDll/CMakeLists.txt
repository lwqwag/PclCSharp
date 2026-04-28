cmake_minimum_required(VERSION 3.20)
project(PointCloudDll CXX)

# Source files for the PointCloudDll DLL
set(SOURCES
    PointCloud.cpp
)

add_library(PointCloudDll SHARED ${SOURCES})

target_compile_definitions(PointCloudDll PRIVATE
    POINTCLOUDDLL_EXPORTS
    _WINDOWS
    _USRDLL
    FLANN_STATIC
    _USE_MATH_DEFINES
    NOMINMAX
)

target_include_directories(PointCloudDll PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(PointCloudDll PRIVATE ${PCL_LIBRARIES})
target_link_directories(PointCloudDll PRIVATE ${PCL_LIBRARY_DIRS})

if(MSVC)
    target_compile_options(PointCloudDll PRIVATE /W3 /MP)
    # Suppress noisy warnings common in PCL headers
    target_compile_options(PointCloudDll PRIVATE /wd4996 /wd4251 /wd4275)
endif()
