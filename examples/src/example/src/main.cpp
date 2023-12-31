#include <ros/ros.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <livz/livz.hpp>

void demoBasicItem();
void demoPointCloud();
void demoAnimate();
void demoUpdater();
void demoField();

Eigen::Vector3d curveFunction(double t){
     return V3D(t, std::sin(t), t);
}

void litouchAction(const int cmd_id, const std::vector<double> params)
{
  if(cmd_id == 1){ demoBasicItem(); }
  if(cmd_id == 2){ demoPointCloud(); }
  if(cmd_id == 3){ demoAnimate(); }
  if(cmd_id == 4){ demoUpdater(); }
  if(cmd_id == 5){ demoField(); }

  // slider show
  if(cmd_id == 6){
    Livz::drawParametricCurve("parametric_curve", curveFunction, INTERVAL(0, 10), LCOLOR::GREEN);
  }

  // clear screen
  if(cmd_id == 7){
    Livz::clearScreen();
  }

  // 调整滑块
  if(cmd_id == 8){
    double duration = params[0] / 10.0 ; // [0,100] -> [0,10]
    Livz::drawOneSphere("sphere", curveFunction(duration), 0.7, LCOLOR::YELLOW, -1.0, "map",1);

    // 向不同的terminal发布内容
    Livz::LTout("terminal1")<<"prog: "<<duration <<LTEndl;
    Livz::LTout("terminal2")<<"ball pos: " << curveFunction(duration).transpose()<<LTEndl;
  }
}

int main(int argc, char** argv) {

    // 找到ui.json的绝对路径。利用此cpp文件的绝对路径拼接相对路径。
    std::string absolute_path    = __FILE__;
    size_t found                 = absolute_path.find_last_of("/\\");
    std::string parent_dir       = absolute_path.substr(0, found);
    std::string ui_conf_abs_path = parent_dir + "/../config/touch_layout.json";

    // XX ，启动 ! 
    Livz::launchLiTouch(ui_conf_abs_path);

    // 重新设置litouch_cmd的响应函数，litouch_cmd由python中interface.pubLiTouchCmd发布
    Livz::setLitouchAction(litouchAction);
    

    ros::spin();
    return 0;
}
