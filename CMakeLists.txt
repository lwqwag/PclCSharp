cmake_minimum_required(VERSION 3.20)
project(PclCSharp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Output all DLLs/libs to bin/ so the C# projects can find them
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

# PCL root – prefer PCL_ROOT env variable, fall back to C:/PCL
if(NOT DEFINED PCL_ROOT)
  if(DEFINED ENV{PCL_ROOT})
    set(PCL_ROOT "$ENV{PCL_ROOT}")
  else()
    set(PCL_ROOT "C:/PCL")
  endif()
endif()

# Allow callers to point cmake at the PCL cmake config via -DPCL_DIR=...
# If not set, derive it from PCL_ROOT.
if(NOT DEFINED PCL_DIR)
  set(PCL_DIR "${PCL_ROOT}/cmake")
endif()

find_package(PCL REQUIRED COMPONENTS common io filters kdtree search
             segmentation sample_consensus surface features)

add_subdirectory(src/PclSharpWrapper/PclSharpWrapper)
add_subdirectory(src/PclSharpWrapper/PointCloudDll)
