#include <ros/ros.h>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <livz/livz.hpp>


Eigen::Vector3d sin3D(double t){
     return Eigen::Vector3d(t, std::sin(t), t);
}

bool moveBallUpdater(double duration){
    // duration表示自创建任务开始，经过的秒数
    Livz::drawOneSphere("sphere", sin3D(duration), 0.7, LCOLOR::YELLOW, -1.0, "map",1);

    if(duration > 10.0){ 
        Livz::clearAll("sphere");
        return true; //这将结束任务，自动删除此updater
    } 
    return false;
}

bool waveUpdater(double duration){
    
    //控制帧率为10
    static int last_ms = -100;
    int ms = duration*1000;
    if(ms - last_ms < 100) {
        return false;
    }
    last_ms = ms;

    static int id = 1;
    // 结合animate功能，实现水波扩散效果
    Eigen::Vector3d current_pos(0.4*duration*duration - 20, 0.3*duration*duration - 10, 0);
    LAnimateParam ani_param("circle", 3.0);
    Livz::createAnimate(ani_param, Livz::draw2DCircle,
                        LPARAMS(current_pos, 0.05  , LCOLOR::RED,    0.1, 0.8,"map",id, 20 ),
                        LPARAMS(current_pos, 15.0  , LCOLOR::YELLOW, 0.3, 0.0,"map",id, 50 ) ); //随着圆变大，采样密度也逐渐增大

    Livz::draw2DCircle("circle", current_pos, 0.05  , LCOLOR::RED,    0.1, 1.0,"map",id+100000, 20);
    id++;

    if(duration > 10.0){ 
        return true; //这将结束任务，自动删除此updater
    } 
    return false;

}

bool threeBody(double duration)
{
    static double last_duration = 0.0;
    static Eigen::Vector3d pos1(-10, 0, 0), pos2(10, 0, 0), pos3(0, 5, 10);
    static Eigen::Vector3d vel1(0, 1, 0), vel2(0, -1, 0), vel3(1, 0, 0);

    const double M1 = 1.0;
    const double M2 = 2.0;
    const double M3 = 3.0;
    const double G = 6.67430e1; // 赛博万有引力常数

    // 时间步长
    double dt = duration - last_duration;
    // 计算距离
    Eigen::Vector3d r12 = pos1 - pos2;
    Eigen::Vector3d r13 = pos1 - pos3;
    Eigen::Vector3d r23 = pos2 - pos3;
    // 计算引力
    Eigen::Vector3d F12 = -G * M1 * M2 * r12 / pow(r12.norm(), 3);
    Eigen::Vector3d F13 = -G * M1 * M3 * r13 / pow(r13.norm(), 3);
    Eigen::Vector3d F23 = -G * M2 * M3 * r23 / pow(r23.norm(), 3);

    // 计算加速度
    Eigen::Vector3d a1 = (F12 + F13) / M1;
    Eigen::Vector3d a2 = (-F12 + F23) / M2;
    Eigen::Vector3d a3 = (-F13 - F23) / M3;
    // 更新速度
    vel1 += a1 * dt;
    vel2 += a2 * dt;
    vel3 += a3 * dt;
    // 更新位置
    pos1 += vel1 * dt;
    pos2 += vel2 * dt;
    pos3 += vel3 * dt;
    last_duration = duration;

    Livz::drawOneSphere("sphere", pos1, 1,    LCOLOR::GREEN, 0.8, "map", 1);
    Livz::drawOneSphere("sphere", pos2, 1.4, LCOLOR::YELLOW, 0.8, "map", 2);
    Livz::drawOneSphere("sphere", pos3, 1.7, LCOLOR::BLUE,   0.8, "map", 3);

    Livz::drawOneArrow("arrow", pos1 , pos1 + a1 ,Eigen::Vector3d(0.1, 0.3,0.3), LCOLOR::GREEN, 0.6, "map", 1);
    Livz::drawOneArrow("arrow", pos2 , pos2 + a2 ,Eigen::Vector3d(0.1, 0.3,0.3), LCOLOR::YELLOW, 0.6, "map", 2);
    Livz::drawOneArrow("arrow", pos3 , pos3 + a3 ,Eigen::Vector3d(0.1, 0.3,0.3), LCOLOR::BLUE, 0.6, "map", 3);
    
    if(duration < 60){return false;}
    return true;
}

void demoUpdater()
{
    // 绘制参数化曲线
    Livz::drawParametricCurve("parametric_curve", sin3D, Eigen::Vector2d(0, 10), LCOLOR::GREEN);

    // 通过updater实现小球沿曲线运动
    Livz::addUpdater("task1", moveBallUpdater);
    Livz::Delay( 10000 );
    Livz::clearAll("parametric_curve");

    // 结合updater和animate功能，实现水波扩散效果
    Livz::addUpdater("task2", waveUpdater);
    Livz::Delay( 15000 );
    Livz::clearAll("circle");

    // 三体运动1分钟
    Livz::addUpdater("task3", threeBody);

}