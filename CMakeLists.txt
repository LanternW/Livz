cmake_minimum_required(VERSION 3.0.2)
project(livz)

# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fPIC")

# 添加ROS include目录
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic")
find_package(roscpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 寻找PCL库
find_package(PCL REQUIRED)


# 寻找Eigen库
find_package(Eigen3 REQUIRED)

include_directories(
  ./
  ${roscpp_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
)

add_definitions(-DINSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}")
add_definitions(${PCL_DEFINITIONS}) # 添加PCL定义

execute_process(COMMAND bash -c "echo $HOME" OUTPUT_VARIABLE HOME_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(
    COMMAND bash -c 
    "if grep -q 'export LIVZ_TOUCH_LIB_VAR=' ${HOME_DIR}/.bashrc; then \
        sed -i 's|export LIVZ_TOUCH_LIB_VAR=.*|export LIVZ_TOUCH_LIB_VAR=${CMAKE_INSTALL_PREFIX}/lib/LiTouch/scripts|' ${HOME_DIR}/.bashrc; \
    else \
        echo 'export LIVZ_TOUCH_LIB_VAR=${CMAKE_INSTALL_PREFIX}/lib/LiTouch/scripts' >> ${HOME_DIR}/.bashrc; \
    fi"
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_library(${PROJECT_NAME} STATIC 
                src/livz.cpp
                src/livz_animate.cpp)

# 添加ROS链接库
target_link_libraries( ${PROJECT_NAME} 
    ${roscpp_LIBRARIES} 
    ${visualization_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES} 
    ${PCL_LIBRARIES} 
)

# 安装静态链接库
install(TARGETS ${PROJECT_NAME} 
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

# 安装头文件
install(DIRECTORY include/
        DESTINATION include/livz)

# 安装LivzTouch
install(DIRECTORY LiTouch/
        DESTINATION lib/LiTouch)