cmake_minimum_required(VERSION 3.20)
project(PclCSharp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Output all DLLs/libs to bin/ so the C# projects can find them
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

# PCL discovery strategy (in order of priority):
#
#   1. Explicit -DPCL_DIR=<path> passed to cmake – used as-is.
#   2. Explicit -DPCL_ROOT=<path> or PCL_ROOT env var – PCLConfig.cmake is
#      searched in the standard sub-directories of that root.
#   3. vcpkg toolchain (CMAKE_TOOLCHAIN_FILE contains "vcpkg") – vcpkg sets
#      CMAKE_PREFIX_PATH so find_package() will locate PCL automatically;
#      no manual PCL_DIR is needed.
#   4. Windows AllInOne installer default path C:/PCL.

if(NOT DEFINED PCL_ROOT)
  if(DEFINED ENV{PCL_ROOT})
    set(PCL_ROOT "$ENV{PCL_ROOT}")
  elseif(WIN32 AND EXISTS "C:/PCL")
    set(PCL_ROOT "C:/PCL")
  endif()
endif()

# Derive PCL_DIR from PCL_ROOT only when the cmake config actually exists there.
# This avoids pointing find_package() at a missing directory when vcpkg is used.
if(NOT DEFINED PCL_DIR AND DEFINED PCL_ROOT)
  foreach(_candidate "${PCL_ROOT}/cmake" "${PCL_ROOT}/lib/cmake/pcl")
    if(EXISTS "${_candidate}/PCLConfig.cmake")
      set(PCL_DIR "${_candidate}")
      break()
    endif()
  endforeach()
endif()

find_package(PCL REQUIRED COMPONENTS common io filters kdtree search
             segmentation sample_consensus surface features)

add_subdirectory(src/PclSharpWrapper/PclSharpWrapper)
add_subdirectory(src/PclSharpWrapper/PointCloudDll)
