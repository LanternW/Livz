cmake_minimum_required(VERSION 3.0.2)
project(example)

# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)                                 # 必要的

# 添加O3编译优化选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fPIC")

# 找到catkin宏和库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

# 找到Eigen3
find_package(Eigen3 REQUIRED)                                  # 必要的
# 找到PCL
find_package(PCL 1.7 REQUIRED COMPONENTS common io)            # 必要的

# 向catkin_package添加依赖库
catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy
  DEPENDS Eigen3 PCL
)

# 找到lrosvislib库
# find_library(LROSVIS_LIBRARY NAMES lrosvis PATHS /usr/lib)
# 有可能安装在/usr/local/lib , 根据自己的电脑
find_library(LIVZ_LIBRARY NAMES livz PATHS /usr/local/lib)   # 必要的

# 包括Eigen3和PCL目录
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}  # Eigen3包括目录                          # 必要的
  ${PCL_INCLUDE_DIRS}    # PCL包括目录                              # 必要的
)

# 用main.cpp生成可执行目标
add_executable(main_node 
              src/main.cpp
              src/demo1.cpp
              src/demo2.cpp
              src/demo3.cpp
              src/demo4.cpp
              src/demo5.cpp
)

# 链接lrosvislib.a静态库和Eigen3、PCL库
target_link_libraries(main_node
  ${LIVZ_LIBRARY}  # 可视化静态库                                # 必要的
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}    # PCL库
)

# 添加PCL定义，以解决与VTK的兼容性问题
add_definitions(${PCL_DEFINITIONS})