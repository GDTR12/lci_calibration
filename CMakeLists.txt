cmake_minimum_required(VERSION 3.15.1)

project(lci_calibration)

set(CMAKE_CXX_FLAGS "-std=c++17 -O2")   #这里！！！

# 模块路径
# list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# list(APPEND CMAKE_MODULE_PATH "/root/workspace/packages/pcl/install")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ndt_omp")

# 查找XXXConfig.cmake的路径添加
# list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ndt_omp")
# list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party")


set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g")

set(KONTIKI_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Kontiki/include")
set(BASALT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/basalt/include)
set(LIE_SPLINE_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lie_spline/include)
set(NDT_OMP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ndt_omp/share/ndt_omp/cmake)
set(NDT_OMP_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ndt_omp/include)
set(NDT_OMP_LINK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ndt_omp/lib)
set(INCLUDE_PATH "./include")


# include(cmake/FindPCL.cmake)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)

# common io surface visualization registration segmentation filters features recognition tracking ml search sample_consensus range_image octree geometry keypoints 2d simulation outofcore

# find_package(PCL REQUIRED COMPONENTS ${PCL_COMPONENTS})
find_package(GTest REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(fmt REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Pangolin REQUIRED)
find_package(yaml-cpp REQUIRED)
# set(ndt_omp_DIR ${NDT_OMP_DIR})
find_package(ndt_omp REQUIRED)
find_package(Ceres REQUIRED)



file(GLOB_RECURSE LIBRARY_SRCS "./*.cpp")
file(GLOB_RECURSE NODE_SRCS "./src/app/*_node.cpp")
file(GLOB_RECURSE TEST_SRCS "./test/*.cpp")
# file(GLOB_RECURSE THIRD_PARTY_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/*.cpp")
list(REMOVE_ITEM LIBRARY_SRCS ${NODE_SRCS})
list(REMOVE_ITEM LIBRARY_SRCS ${TEST_SRCS})
# list(REMOVE_ITEM LIBRARY_SRCS ${THIRD_PARTY_SRCS})

if(ndt_omp_FOUND)
  message("ndt_omp found")
endif()
message("KONTIKI_INCLUDE: ${KONTIKI_INCLUDE}")
message("Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")
message("Found ceres headers in: ${CERES_INCLUDE_DIRS}")
message("LIBRARY_SRCS: ${LIBRARY_SRCS}")
message("NDT_OMP_LINK_PATH ${NDT_OMP_LINK_PATH}")
message("PCL_LIBRARY_DIRS ${PCL_LIBRARY_DIRS}")
message("Pangolin_LIBRARIES: ${Pangolin_LIBRARIES}")
message("Pangolin_CMAKEMODULES: ${Pangolin_CMAKEMODULES}")

set(MY_PROJECT_INCLUDE  
  ${INCLUDE_PATH} 
  ${KONTIKI_INCLUDE} 
  ${CERES_INCLUDE_DIRS} 
  ${LIE_SPLINE_INCLUDE} 
  ${BASALT_INCLUDE} 
  ${NDT_OMP_INCLUDE}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(MY_PROJECT_LIBS_INCLUDE
  ${NDT_OMP_LINK_PATH}
  ${PCL_LIBRARY_DIRS}
)

set(MY_PROJECT_LIBS
  ${CERES_LIBRARIES} 
  ${PCL_LIBRARIES}
  ceres 
  fmt
  ${Pangolin_LIBRARY}
  ndt_omp
  yaml-cpp
  ${OpenCV_LIBS}
)

set(AMENT_TARGET_DEPS
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

if(${CMAKE_NODE_NAME} STREQUAL "all")
  message("=========================")
  message("[Make]: all node")
  message("=========================")
  foreach(NODE_PATH ${NODE_SRCS})
    # message("node path:  ${NODE_PATH}")
    get_filename_component(NODE_FILE ${NODE_PATH} NAME_WE)
    # message("node file:  ${NODE_FILE}")
    add_executable(${NODE_FILE} ${LIBRARY_SRCS} ${NODE_PATH})
    target_include_directories(${NODE_FILE} PUBLIC ${MY_PROJECT_INCLUDE})
    target_link_directories(${NODE_FILE} PUBLIC ${MY_PROJECT_LIBS_INCLUDE})
    target_link_libraries(${NODE_FILE} ${MY_PROJECT_LIBS})
    ament_target_dependencies(${NODE_FILE} ${AMENT_TARGET_DEPS})

    target_compile_features(${NODE_FILE} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
    install(TARGETS ${NODE_FILE} 
      DESTINATION lib/${PROJECT_NAME}) 
  endforeach()
else()
  message("=========================")
  message("[Make]: ${CMAKE_NODE_NAME}")
  message("=========================")
  foreach(NODE_PATH ${NODE_SRCS})
    get_filename_component(NODE_FILE ${NODE_PATH} NAME_WE)
    if(${NODE_FILE} STREQUAL ${CMAKE_NODE_NAME})

      add_executable(${CMAKE_NODE_NAME}  ${LIBRARY_SRCS} ${NODE_PATH})
      target_include_directories(${CMAKE_NODE_NAME} PUBLIC ${MY_PROJECT_INCLUDE})
      target_link_directories(${CMAKE_NODE_NAME} PUBLIC ${MY_PROJECT_LIBS_INCLUDE})
      target_link_libraries(${CMAKE_NODE_NAME} ${MY_PROJECT_LIBS})
      ament_target_dependencies(${CMAKE_NODE_NAME} ${AMENT_TARGET_DEPS})

      target_compile_features(${CMAKE_NODE_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
      install(TARGETS ${CMAKE_NODE_NAME} 
        DESTINATION lib/${PROJECT_NAME})    
    endif()
  endforeach()
  


endif()




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


