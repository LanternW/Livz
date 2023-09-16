#include <ros/ros.h>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <livz/livz.hpp>


std::vector<Eigen::Vector3d> L_set;
std::vector<Eigen::Vector3d> I_set;
std::vector<Eigen::Vector3d> V_set;
std::vector<Eigen::Vector3d> Z_set;
std::vector<Eigen::Vector3d> LIVZ_set;

void initShapeSets()
{
    L_set.push_back(Eigen::Vector3d(0, 0, 0));
    L_set.push_back(Eigen::Vector3d(0, 10, 0));
    L_set.push_back(Eigen::Vector3d(2, 10, 0));
    L_set.push_back(Eigen::Vector3d(2, 2, 0));
    L_set.push_back(Eigen::Vector3d(8, 2, 0));
    L_set.push_back(Eigen::Vector3d(8, 0, 0));

    I_set.push_back(Eigen::Vector3d(0, 0, 0));
    I_set.push_back(Eigen::Vector3d(0, 10, 0));
    I_set.push_back(Eigen::Vector3d(2, 10, 0));
    I_set.push_back(Eigen::Vector3d(2, 0, 0));


    V_set.push_back(Eigen::Vector3d(0, 0, 0));
    V_set.push_back(Eigen::Vector3d(-4, 10, 0));
    V_set.push_back(Eigen::Vector3d(-2, 10, 0));
    V_set.push_back(Eigen::Vector3d(0, 4, 0));
    V_set.push_back(Eigen::Vector3d(2, 10, 0));
    V_set.push_back(Eigen::Vector3d(4, 10, 0));

    Z_set.push_back(Eigen::Vector3d(0, 0, 0));
    Z_set.push_back(Eigen::Vector3d(0, 2, 0));
    Z_set.push_back(Eigen::Vector3d(4, 8, 0));
    Z_set.push_back(Eigen::Vector3d(0, 8, 0));
    Z_set.push_back(Eigen::Vector3d(0, 10, 0));
    Z_set.push_back(Eigen::Vector3d(8, 10, 0));
    Z_set.push_back(Eigen::Vector3d(2, 2, 0));
    Z_set.push_back(Eigen::Vector3d(8, 2, 0));
    Z_set.push_back(Eigen::Vector3d(8, 0, 0));

    LIVZ_set.push_back(Eigen::Vector3d(-15, 0, 0));
    LIVZ_set.push_back(Eigen::Vector3d(-15, 10, 0));
    LIVZ_set.push_back(Eigen::Vector3d(-13, 10, 0));
    LIVZ_set.push_back(Eigen::Vector3d(-13, 2, 0));

    LIVZ_set.push_back(Eigen::Vector3d(-7, 2,  0));
    LIVZ_set.push_back(Eigen::Vector3d(-7, 10, 0));

    LIVZ_set.push_back(Eigen::Vector3d(-7, 10, 0));
    LIVZ_set.push_back(Eigen::Vector3d(-3, 10, 0));
    LIVZ_set.push_back(Eigen::Vector3d(-1, 4, 0));
    LIVZ_set.push_back(Eigen::Vector3d(1, 10, 0));
    LIVZ_set.push_back(Eigen::Vector3d(11, 10, 0));
    LIVZ_set.push_back(Eigen::Vector3d(5, 2, 0));
    LIVZ_set.push_back(Eigen::Vector3d(11, 2, 0));
    LIVZ_set.push_back(Eigen::Vector3d(11, 0, 0));
    LIVZ_set.push_back(Eigen::Vector3d(3, 0, 0));
    LIVZ_set.push_back(Eigen::Vector3d(3, 2, 0));
    LIVZ_set.push_back(Eigen::Vector3d(7, 8, 0));
    LIVZ_set.push_back(Eigen::Vector3d(3, 8, 0));
    LIVZ_set.push_back(Eigen::Vector3d(-1, 0, 0));
    LIVZ_set.push_back(Eigen::Vector3d(-5, 8, 0));
    LIVZ_set.push_back(Eigen::Vector3d(-5, 10,0));
    LIVZ_set.push_back(Eigen::Vector3d(-5, 0, 0));
    LIVZ_set.push_back(Eigen::Vector3d(-7, 0, 0));
    LIVZ_set.push_back(Eigen::Vector3d(-7, 2, 0));
    LIVZ_set.push_back(Eigen::Vector3d(-7, 0, 0));
    LIVZ_set.push_back(Eigen::Vector3d(-9, 0, 0));


}

//动画功能演示
void demoAnimate()
{
  initShapeSets();
  //动画属性
  LAnimateParam ani_param("polygon", 2.0, RATE_FUNC::sine, false);

  Livz::draw2DPolygon("polygon", L_set, LCOLOR::YELLOW, 0.1,    -1.0, "map", 20 );

  //协同延时
  Livz::Delay(2000);

  //创建动画 （使用LPARAMS创建参数时，省略话题名，但是必须给出全部参数，无法使用默认参数）
  Livz::createAnimate( ani_param, Livz::draw2DPolygon, 
                       LPARAMS(L_set     , LCOLOR::YELLOW, 0.1,    -1.0, "map", 20) , 
                       LPARAMS(I_set     , LCOLOR::RED   , 0.2,    -1.0, "map", 20) );

  Livz::DelayAccordingtoAnimate(ani_param);
  Livz::Delay(700);

  Livz::createAnimate( ani_param, Livz::draw2DPolygon, 
                       LPARAMS(I_set     , LCOLOR::RED, 0.1,    -1.0, "map", 20) , 
                       LPARAMS(V_set     , LCOLOR::BLUE   , 0.2,    -1.0, "map", 20) );

  Livz::DelayAccordingtoAnimate(ani_param);
  Livz::Delay(700);

  Livz::createAnimate( ani_param, Livz::draw2DPolygon, 
                       LPARAMS(V_set     , LCOLOR::BLUE, 0.1,    -1.0, "map", 20) , 
                       LPARAMS(Z_set     , LCOLOR::GREEN   , 0.2,    -1.0, "map", 20) );  

  Livz::DelayAccordingtoAnimate(ani_param);
  Livz::Delay(700);

  Livz::createAnimate( ani_param, Livz::draw2DPolygon, 
                       LPARAMS(Z_set     , LCOLOR::GREEN, 0.1,    -1.0, "map", 20) , 
                       LPARAMS(LIVZ_set  , LCOLOR::YELLOW   , 0.2,    -1.0, "map", 20) );  

  Livz::DelayAccordingtoAnimate(ani_param);
  Livz::Delay(2000);

  Livz::clearAll("polygon");

    //莫比乌斯带
  auto mobius_strip = SURF_FUNC( [](double u, double v) {
    double a = 1.0; // 带的半径
    double w = 1.0; // 带的宽度
    double x = a * (1 + w * std::cos(v / 2) * std::cos(u)) * std::cos(u);
    double y = a * (1 + w * std::cos(v / 2) * std::sin(u)) * std::sin(u);
    double z = a * w * std::sin(v / 2);
    return z;
  });
  
  //双曲抛物面
  auto hyperbolic_paraboloid = SURF_FUNC( [](double u, double v) {
    double a = 1.0;
    double b = 1.0;
    double x = a * u;
    double y = b * v;
    double z = u * v;
    return z;
  });

  ani_param.topic_name_ = "surf";
  Livz::createAnimate(ani_param, Livz::renderSurface, 
                            LPARAMS( hyperbolic_paraboloid , Eigen::Vector2d(-3,3), Eigen::Vector2d(-3,3), 0.1,0.1, LCOLOR::YELLOW, 0.5 , "map", 50),
                            LPARAMS( mobius_strip , Eigen::Vector2d(-10,10), Eigen::Vector2d(-10,10), 0.3, 0.3 , LCOLOR::PINK,    0.5 , "map", 50) );
  Livz::DelayAccordingtoAnimate(ani_param);
  Livz::Delay(2000);
  Livz::clearAll("surf");

  //创建2个点云对象
  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  // 填充矩形区域的点云数据
  for (float x = -2; x < 2; x += 0.01) {
      for (float y = -2; y < 2; y += 0.01) {
          pcl::PointXYZ point;
          point.x = 3*x;
          point.y = 3*y;
          point.z = x*x + y*y;
          cloud1.points.push_back(point);
          point.x = x;
          point.y = y;
          point.z = 5-x*x-y*y;
          cloud2.points.push_back(point);
      }
  }

  ani_param.topic_name_ = "pointcloud";
  ani_param.duration_   = 4.0;
  ani_param.rate_func_  = RATE_FUNC::linear;
  Livz::createAnimate(ani_param, Livz::drawPointcloud, 
                            LPARAMS(cloud2     , LCOLOR::CYAN,     "map") , 
                            LPARAMS(cloud1     , LCOLOR::ORANGE,   "map") );

  Livz::DelayAccordingtoAnimate(ani_param);
  Livz::Delay(2000);

  Livz::clearAll("pointcloud");
}