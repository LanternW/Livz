#include <ros/ros.h>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <livz/livz.hpp>

//基础绘制
void demoBasicItem()
{
    //绘制扇形 
    Livz::draw2DCone("cone", Eigen::Vector3d(0,0,0), 10, 0.89, 0.0,      LCOLOR::PINK, 0.1 , -1.0, "map", 1);
    Livz::draw2DCone("cone", Eigen::Vector3d(0,0,0), 10, 0.89, M_PI_2,   LCOLOR::PINK, 0.2 , -1.0, "map", 2);
    Livz::draw2DCone("cone", Eigen::Vector3d(0,0,0), 10, 0.89, M_PI,     LCOLOR::PINK, 0.3 , -1.0, "map", 3);
    Livz::draw2DCone("cone", Eigen::Vector3d(0,0,0), 10, 0.89, 3*M_PI_2, LCOLOR::PINK, 0.4 , -1.0, "map", 4);

    //绘制矩形
    Livz::draw2DRect("rect", Eigen::Vector3d(5,0,3), 5,5,   0,     LCOLOR::GOLD,  0.2, 0.8, "map", 1);
    Livz::draw2DRect("rect", Eigen::Vector3d(5,0,3), 10,10, 0.4,   LCOLOR::GOLD,  0.2, 0.6, "map", 2);
    Livz::draw2DRect("rect", Eigen::Vector3d(5,0,3), 5,7,   0.8,   LCOLOR::GOLD,  0.2, 0.4, "map", 3);
    Livz::draw2DRect("rect", Eigen::Vector3d(5,0,3), 10,12, 1.2,   LCOLOR::GOLD,  0.2, 0.2, "map", 4);

    // 绘制圆
    Livz::draw2DCircle("circle", Eigen::Vector3d(-5,5,0), 3, LCOLOR::RED);

    // 绘制椭圆
    Livz::draw2DEllipse("ellipse", Eigen::Vector3d(5,5,0), 3, 1.5);

    // 绘制线段群
    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d(10,10,0));
    points.push_back(Eigen::Vector3d(10,-10,0));
    points.push_back(Eigen::Vector3d(-10,-10,0));
    points.push_back(Eigen::Vector3d(-10,10,0));
    Livz::drawLineList("lines", points, 0.5, LCOLOR::BLUE);

    //绘制圆柱体
    Livz::drawOneCylinder("cylinder", Eigen::Vector3d(0,0,-5) , 4, 10, LCOLOR::CYAN , 0.2);

    // 绘制立方体
    Livz::drawOneCube("cube", Eigen::Vector3d(10,10,5), Eigen::Vector3d(2,2,2), LCOLOR::ORANGE  , 0.5, "map", 1);
    Livz::drawOneCube("cube", Eigen::Vector3d(-10,-10,5), Eigen::Vector3d(3,3,3), LCOLOR::ORANGE, 0.5, "map", 2);
    Livz::drawOneCube("cube", Eigen::Vector3d(10,-10,5), Eigen::Vector3d(4,4,4), LCOLOR::ORANGE , 0.5, "map", 3);
    Livz::drawOneCube("cube", Eigen::Vector3d(-10,10,5), Eigen::Vector3d(5,5,5), LCOLOR::ORANGE , 0.5, "map", 4);


    // 绘制参数化曲线
    auto curveFunction = [](double t) -> Eigen::Vector3d {
        return Eigen::Vector3d(t, std::sin(t), t);
    };
    Livz::drawParametricCurve("parametric_curve", curveFunction, Eigen::Vector2d(-10, 10), LCOLOR::GREEN);

    // 绘制参数化曲面
    auto saddleSurface = [](double u, double v) -> double {
        double z = 0.1 *u * v; // 马鞍曲面
        return z + 20;
    };

    Livz::renderSurface("surf", saddleSurface, Eigen::Vector2d(-10, 10), Eigen::Vector2d(-10, 10), 0.1, 0.1, LCOLOR::GOLD, 0.5, "map", 1);

    // 绘制视窗文字
    Livz::drawViewText("view_text", "Welcome to LIVZ", Eigen::Vector3d(0,0,10), 2, LCOLOR::MAGENTA);

}