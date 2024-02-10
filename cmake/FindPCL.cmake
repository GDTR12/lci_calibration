# 在子目录中设置 CMAKE_PREFIX_PATH
# set(pre_CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH})
# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/root/workspace/packages/pcl/install")

# 调用 find_package
find_package(PCL REQUIRED)

# message("find PCL in ${CMAKE_PREFIX_PATH}")
# message("PCL_LIBRARY_DIRS ${PCL_LIBRARY_DIRS}")

# set(CMAKE_PREFIX_PATH ${pre_CMAKE_PREFIX_PATH})
