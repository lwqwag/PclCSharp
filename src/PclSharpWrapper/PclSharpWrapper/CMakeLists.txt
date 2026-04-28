cmake_minimum_required(VERSION 3.20)
project(PclDll CXX)

find_package(PCL REQUIRED)

# Source files for the PclSharpWrapper DLL
set(SOURCES
    filter.cpp
    io.cpp
    pcl_util.cpp
    sampleConsensus.cpp
    segmentation.cpp
    util_module.cpp
)

set(HEADERS
    filter.h
    io.h
    pcl_util.h
    sampleConsensus.h
    segmentation.h
    util_module.h
)

add_library(PclDll SHARED ${SOURCES} ${HEADERS})

target_compile_options(PclDll PRIVATE ${PCL_DEFINITIONS})

target_compile_definitions(PclDll PRIVATE
    PCLSHARPWRAPPER_EXPORTS
    _WINDOWS
    _USRDLL
    FLANN_STATIC
    _USE_MATH_DEFINES
    NOMINMAX
)

target_include_directories(PclDll PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(PclDll PRIVATE ${PCL_LIBRARIES})
target_link_directories(PclDll PRIVATE ${PCL_LIBRARY_DIRS})

if(MSVC)
    target_compile_options(PclDll PRIVATE /W3 /MP)
    # Suppress noisy warnings common in PCL headers
    target_compile_options(PclDll PRIVATE /wd4996 /wd4251 /wd4275)
endif()
