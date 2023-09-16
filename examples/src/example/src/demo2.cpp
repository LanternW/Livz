#include <ros/ros.h>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <livz/livz.hpp>


Eigen::Vector3d customShader(const Eigen::Vector3d pos) {

  std::complex<double> c( (pos[0] - 6)/2, (pos[1] + 6)/2);
  std::complex<double> z(0, 0);
  int max_iterations = 100;
  int iteration = 0;

  while (std::abs(z) <= 2.0 && iteration < max_iterations) {
    z = z * z + c;
    iteration++;
  }

  if (iteration == max_iterations) {
    return Eigen::Vector3d(0, 0, 0); // 黑色，表示在曼德尔布罗特集合内
  } else {
    // 根据发散速度返回颜色
    double ratio = double(iteration) / max_iterations;
    return Eigen::Vector3d(255 * ratio, 255 * (1 - ratio), 0);
  }
}

//点云绘制
void demoPointCloud()
{   
    //普通点云1
    pcl::PointCloud<pcl::PointXYZ> cloud;
    //普通点云2
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    //带颜色的点云
    pcl::PointCloud<pcl::PointXYZRGB> colorCloud;
    //带强度信息的点云
    pcl::PointCloud<pcl::PointXYZI> intensityCloud;

    pcl::PointXYZRGB point_rgb;
    pcl::PointXYZI point_i;
    const double gap = 6;
    for (float i = -5; i <= 5; i += 0.02) {
        for (float j = -5; j <= 5; j += 0.02) {
            cloud.push_back(pcl::PointXYZ(i-gap, j-gap, 0));
            cloud2.push_back(pcl::PointXYZ(i+gap, j-gap, 0));

            point_rgb.x = i-gap; point_rgb.y = j+gap; point_rgb.z = 0;
            point_rgb.r = (uint8_t)(255 * fabs(sin(i)));
            point_rgb.g = (uint8_t)(255 * fabs(cos(j)));
            point_rgb.b = 128;
            colorCloud.push_back(point_rgb);

            point_i.x = i+gap; point_i.y = j+gap; point_i.z = 0;
            point_i.intensity = sqrt(i * i + j * j);
            intensityCloud.push_back(point_i);
        }
    }

    //绘制普通点云
    Livz::drawPointcloud("pointcloud", cloud, LCOLOR::WHITE);

    //绘制彩色点云
    Livz::drawPointcloudRGB("color_pointcloud", colorCloud);

    //绘制带强度信息的点云
    Livz::drawPointcloudI("intensity_pointcloud", intensityCloud);

    //使用自定义着色器绘制点云
    Livz::drawPointcloudWithShader("shader_pointcloud", cloud2, customShader);

}