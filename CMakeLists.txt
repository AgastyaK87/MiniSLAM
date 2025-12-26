cmake_minimum_required(VERSION 3.10)
project(MiniSLAM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
# 1. Eigen3 (Math)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 2. OpenCV (Vision)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 3. Pangolin (Visualization)
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# --- Include Directories ---
include_directories(include)

# --- Library: minislam_core ---
# This holds all our math/logic (camera, optimization, etc.)
add_library(minislam_core
    src/camera.cpp
)

# Link dependencies to our library
target_link_libraries(minislam_core
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    Eigen3::Eigen
)

# --- Executable: run_vo ---
# This is the program we actually run
add_executable(run_vo app/run_vo.cpp)
target_link_libraries(run_vo minislam_core)