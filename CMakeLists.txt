cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("Main")

# Set cmake prefix path to enable cmake to find freenect2
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /home/javi/freenect2/lib/cmake/freenect2)

find_package(freenect2 REQUIRED) # pcl
find_package(OpenCV REQUIRED) # opencv
find_package(PCL 1.2 REQUIRED) # pcl

include_directories(
  "/usr/include/libusb-1.0/" # libfreenect2
  ${freenect2_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} # pcl
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_library(timer timer.cpp)
add_library(coarse_align coarse_align.cpp)
add_library(acquisition acquisition.cpp)
add_library(visualization visualization.cpp)
add_library(filterCloud filterCloud.cpp)
add_library(kp_detectors kp_detectors.cpp)
add_library(fine_align fine_align.cpp)
add_library(surface_reconstruction surface_reconstruction.cpp)

add_executable(main main.cpp)

target_link_libraries(kp_detectors timer)
target_link_libraries(coarse_align timer)
target_link_libraries(
main ${OpenCV_LIBS} ${PCL_LIBRARIES} ${freenect2_LIBRARIES} timer coarse_align
kp_detectors visualization fine_align acquisition filterCloud surface_reconstruction
)
