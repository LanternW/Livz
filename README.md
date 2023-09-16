### Livz 轻量可视化库
---
Livz是基于ROS1框架的针对Rviz可视化的轻量级工具，封装了Rviz中常用的可视化类型，能够快捷地显示基础组件，免去维护发布者的麻烦。除了基础组件，Livz可以方便地融合用户代码，便捷地实现非线性动画和自定义事件。目前，Livz仍处于开发阶段，欢迎来寻找bug、提出改进意见！

### 新功能： LiTouch
---
在beta1.2版本中，将Livz与LiTouch进行了浅度集成。

移步 [LiTouch中文文档](./LITOUCH.md)

### 编译安装:
---

安装依赖（除了ROS环境外的）
```
sudo apt-get install wmctrl
```

首先克隆仓库
```
git clone https://github.com/LanternW/Livz.git
```
编译与安装：
```
cd Livz
mkdir build & cd build
cmake ..
make 
sudo make install
```

### 运行演示程序
---
```
cd Livz/example
catkin_make
source devel/setup.bash
roslaunch example run_demo.launch
```

### 使用
---
在ROS工程下使用Livz ,需要的依赖为：Eigen3 , PCL

在所有需要调用Livz的功能包的CMakeList.txt中，至少添加以下内容：
```CMAKE
# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17) 
# 找到Eigen3
find_package(Eigen3 REQUIRED)                                 
# 找到PCL
find_package(PCL 1.7 REQUIRED COMPONENTS common io)           

# 找到Livz
find_library(LIVZ_LIBRARY NAMES livz PATHS /usr/local/lib) 
# 有可能需要在/usr/lib寻找,具体位置取决于CMake的安装前缀或目标系统的库文件存放约定

# 包括Eigen3和PCL目录
include_directories(
  ... # 其他
  ${EIGEN3_INCLUDE_DIR}  # Eigen3包括目录                         
  ${PCL_INCLUDE_DIRS}    # PCL包括目录                            
)

# 链接lrosvislib.a静态库和Eigen3、PCL库
target_link_libraries( your_target
  ${LIVZ_LIBRARY}     # Livz （需要在Eigen和PCL前，否则可能会有bug）
  ${catkin_LIBRARIES} # Eigen
  ${PCL_LIBRARIES}    # PCL
  ... #其他
)
```
#### 最简示例
```C++
#include <ros/ros.h>
#include <livz/livz.hpp>

int main(int argc, char** argv) {
  
    Livz::draw2DRect("rect", Eigen::Vector3d(5,0,2), 6,6);
    return 0;
}
```

### 文档
---
[Livz中文文档](DOC.md)