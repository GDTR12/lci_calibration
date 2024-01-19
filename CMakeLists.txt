cmake_minimum_required(VERSION 3.15.1)

project(lci_calibration)

set(CMAKE_CXX_FLAGS "-std=c++17 -O2")   #这里！！！

# list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(KONTIKI_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Kontiki/include")

message("${KONTIKI_INCLUDE} ===============")
# message("${CMAKE_MODULE_PATH} ===============")

find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(GTest REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(fmt REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/root/workspace/packages/ceres-solver/bin/lib/cmake")
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

set(BASALT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/basalt/include)
set(LIE_SPLINE_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lie_spline/include)

message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Found ceres headers in: ${CERES_INCLUDE_DIRS}")

file(GLOB_RECURSE LIBRARY_SRCS "./*.cpp")
file(GLOB_RECURSE NODE_SRCS "./app/*_node.cpp")
file(GLOB_RECURSE TEST_SRCS "./test/*.cpp")
# file(GLOB_RECURSE THIRD_PARTY_SRCS "../../third_party/*.cpp")
list(REMOVE_ITEM LIBRARY_SRCS ${NODE_SRCS})
list(REMOVE_ITEM LIBRARY_SRCS ${TEST_SRCS})
# list(REMOVE_ITEM LIBRARY_SRCS ${THIRD_PARTY_SRCS})
message("LIBRARY_SRCS: ${LIBRARY_SRCS}\n")

set(INCLUDE_PATH "./include")
add_executable(lci_calibration ${LIBRARY_SRCS} ${NODE_SRCS})
target_include_directories(lci_calibration PUBLIC 
  ${INCLUDE_PATH} ${KONTIKI_INCLUDE} ${CERES_INCLUDE_DIRS} ${LIE_SPLINE_INCLUDE} ${BASALT_INCLUDE}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(lci_calibration PUBLIC 
    ${KONTIKI_INCLUDE})
target_link_libraries(lci_calibration
    ${CERES_LIBRARIES} 
    ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES}
    ceres fmt
    )


ament_target_dependencies(
  lci_calibration
  "rclcpp"
  "geometry_msgs"
  "visualization_msgs"
  "nav_msgs"
  "rosbag2_cpp"
  "Eigen3"
  "tf2_ros"
  "PCL"
  "pcl_conversions"
)





set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fopenmp")
target_compile_features(lci_calibration PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS lci_calibration 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch urdf config rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
