// Copyright (c) [2023] [Lantern]
// 
// This file is part of [Livz]
// 
// This project is licensed under the MIT License.
// See LICENSE.txt for details.
#ifndef LROSVIS_LIB_H
#define LROSVIS_LIB_H

#include <map>
#include <cmath>
#include <tuple>
#include <cstdlib>
#include <dlfcn.h>
#include <unistd.h>
#include <sstream>
#include <bitset>
#include <sys/types.h>
#include <sys/wait.h>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/bind.hpp>

#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "livz_animate.hpp"

#ifndef INSTALL_PREFIX
#define INSTALL_PREFIX "Undefined"
#else
#define LIVZ_TOUCH_PATH INSTALL_PREFIX "/lib/LiTouch/scripts/main.py"
#endif
#define LPARAMS(...) std::make_tuple(__VA_ARGS__)

#define SURF_FUNC(obj) std::function<double(double, double)>(obj)
#define SURFFUNC_FROM_MEMBER(func) [this](double u,double v) -> double { return this->func(u,v); }

#define PARACURVE_FUNC(obj) std::function<Eigen::Vector3d(double)>(obj)
#define PARACURVEFUNC_FROM_MEMBER(func) [this](double t) -> Eigen::Vector3d { return this->func(t); }

#define UPDATER_FUNC(obj) std::function<bool(double)>(obj)
#define UPDATERFUNC_FROM_MEMBER(func) [this](double t) -> bool { return this->func(t); }

#define SHADER_FUNC(obj) std::function<Eigen::Vector3d(Eigen::Vector3d)>(obj)
#define SHADERFUNC_FROM_MEMBER(func) [this](Eigen::Vector3d p) -> Eigen::Vector3d { return this->func(p); }

#define VECTOR_FIELD(obj) std::function<Eigen::Vector3d(Eigen::Vector3d)>(obj)
#define VECTORFIELD_FROM_MEMBER(func) [this](Eigen::Vector3d p) -> Eigen::Vector3d { return this->func(p); }

#define LITOUCHCMD_ACTION_FUNC(obj) std::function<void(const int cmd_id, const std::vector<double> params)>(obj)
#define LAF_FROM_MEMBER(func) [this](const int cmd_id, const std::vector<double> params) -> void { this->func(cmd_id, params); }

#define V2D(x,y) Eigen::Vector2d(x,y)
#define V3D(x,y,z) Eigen::Vector3d(x,y,z)
#define V4D(x,y,z,w) Eigen::Vector4d(x,y,z,w)

#define INTERVAL(x,y) Eigen::Vector2d(x,y)

#define LTEndl LTEndlSignal()




namespace LCOLOR {
    const Eigen::Vector4d RED(1.0, 0.0, 0.0, 1.0);     //红色，不透明
    const Eigen::Vector4d GREEN(0.0, 1.0, 0.0, 1.0);   //绿色，不透明
    const Eigen::Vector4d BLUE(0.0, 0.0, 1.0, 1.0);    //蓝色，不透明
    const Eigen::Vector4d YELLOW(1.0, 1.0, 0.0, 1.0);  //黄色，不透明
    const Eigen::Vector4d CYAN(0.0, 1.0, 1.0, 1.0);    //青色，不透明
    const Eigen::Vector4d MAGENTA(1.0, 0.0, 1.0, 1.0); //洋红，不透明
    const Eigen::Vector4d WHITE(1.0, 1.0, 1.0, 1.0);   //白色，不透明
    const Eigen::Vector4d BLACK(0.0, 0.0, 0.0, 1.0);   //黑色，不透明
    const Eigen::Vector4d GREY(0.5, 0.5, 0.5, 1.0);    //灰色，不透明
    const Eigen::Vector4d ORANGE(1.0, 0.5, 0.0, 1.0);  //橙色，不透明
    const Eigen::Vector4d BROWN(0.6, 0.4, 0.2, 1.0);   //棕色，不透明
    const Eigen::Vector4d PINK(1.0, 0.8, 0.8, 1.0);    //粉红，不透明
    const Eigen::Vector4d PURPLE(0.5, 0.0, 0.5, 1.0);  //紫色，不透明
    const Eigen::Vector4d INDIGO(0.3, 0.0, 0.5, 1.0);  //靛蓝，不透明
    const Eigen::Vector4d VIOLET(0.9, 0.5, 0.9, 1.0);  //紫罗兰，不透明
    const Eigen::Vector4d LIME(0.7, 1.0, 0.3, 1.0);    //酸橙色，不透明
    const Eigen::Vector4d GOLD(1.0, 0.8, 0.0, 1.0);    //金色，不透明
    const Eigen::Vector4d SILVER(0.8, 0.8, 0.8, 1.0);  //银色，不透明
    const Eigen::Vector4d CORAL(1.0, 0.5, 0.3, 1.0);   //珊瑚色，不透明
    const Eigen::Vector4d BEIGE(0.6, 0.6, 0.4, 1.0);   //米色，不透明
    const Eigen::Vector4d MINT(0.3, 1.0, 0.6, 1.0);    //薄荷色，不透明
    const Eigen::Vector4d OLIVE(0.5, 0.5, 0.0, 1.0);   //橄榄色，不透明
    const Eigen::Vector4d APRICOT(0.9, 0.6, 0.5, 1.0); //杏色，不透明
    const Eigen::Vector4d NAVY(0.0, 0.0, 0.5, 1.0);    //海军蓝，不透明
    const Eigen::Vector4d LAVENDER(0.7, 0.5, 0.8, 1.0);//薰衣草紫，不透明
    const Eigen::Vector4d TURQUOISE(0.3, 0.9, 0.8, 1.0);//绿松石色，不透明
    const Eigen::Vector4d TAN(0.8, 0.7, 0.6, 1.0);     //棕褐色，不透明
    const Eigen::Vector4d TEAL(0.0, 0.5, 0.5, 1.0);    //鸭翅蓝，不透明
    const Eigen::Vector4d COFFEE(0.6, 0.3, 0.0, 1.0);  //咖啡色，不透明
    const Eigen::Vector4d ROSE(1.0, 0.0, 0.5, 1.0);    //玫瑰色，不透明
    const Eigen::Vector4d SIENNA(0.6, 0.3, 0.2, 1.0);  //赭色，不透明
    const Eigen::Vector4d PEACH(1.0, 0.9, 0.7, 1.0);   //桃色，不透明
    const Eigen::Vector4d MAROON(0.5, 0.0, 0.0, 1.0);  //褐红色，不透明
    const Eigen::Vector4d AQUA(0.0, 1.0, 1.0, 1.0);    //水绿色，不透明
    const Eigen::Vector4d SALMON(0.9, 0.5, 0.5, 1.0);  //鲑鱼色，不透明
    const Eigen::Vector4d PLUM(0.9, 0.7, 0.9, 1.0);    //李子色，不透明
    const Eigen::Vector4d ORCHID(0.9, 0.4, 0.7, 1.0);  //兰花色，不透明
    const Eigen::Vector4d CHERRY(0.9, 0.0, 0.1, 1.0);  //樱桃红，不透明
    const Eigen::Vector4d AZURE(0.0, 0.5, 1.0, 1.0);   //天蓝色，不透明
    const Eigen::Vector4d GRAY(0.5, 0.5, 0.5, 1.0);    //灰色，不透明
    const Eigen::Vector4d RASPBERRY(0.9, 0.0, 0.4, 1.0);//树莓色，不透明
    const Eigen::Vector4d EMERALD(0.3, 0.8, 0.6, 1.0); //绿宝石色，不透明
    const Eigen::Vector4d CHOCOLATE(0.8, 0.5, 0.3, 1.0);//巧克力色，不透明
    const Eigen::Vector4d AMBER(1.0, 0.8, 0.0, 1.0);   //琥珀色，不透明
    const Eigen::Vector4d CREAM(1.0, 0.9, 0.8, 1.0);   //奶油色，不透明
    const Eigen::Vector4d CRIMSON(0.9, 0.1, 0.1, 1.0); //深红色，不透明
    const Eigen::Vector4d HONEY(1.0, 0.9, 0.6, 1.0);   //蜜色，不透明

    Eigen::Vector4d HEX( const std::string& hex_color );
}

namespace POLYGON_TYPE{
    const int TYPE_MARKER  = 1;
    const int TYPE_POLYGON = 2;
}

namespace CONFIG{
    extern std::string default_frame_id;
}

typedef struct PublisherInfo {
  ros::Publisher publisher;
  std::string type;
} PublisherInfo;

class LTEndlSignal{
public:
    LTEndlSignal(){}
};

class Livz {

public:
    class LiTouchTerminalIS {
    public:
        LiTouchTerminalIS(const std::string& t_name) : terminal_name(t_name) {}
        LiTouchTerminalIS& operator<<(int val) {
            buffer << val;
            return *this;
        }
        LiTouchTerminalIS& operator<<(double val) {
            buffer << val;
            return *this;
        }
        LiTouchTerminalIS& operator<<(const std::string& val) {
            buffer << val;
            return *this;
        }
        LiTouchTerminalIS& operator<<(const char* val) {
            buffer << val;
            return *this;
        }
        LiTouchTerminalIS& operator<<(char val) {
            buffer << val;
            return *this;
        }
        LiTouchTerminalIS& operator<<(bool val) {
            buffer << std::boolalpha << val;
            return *this;
        }
        template<typename T, int Rows, int Cols>
        LiTouchTerminalIS& operator<<(const Eigen::Matrix<T, Rows, Cols>& mat) {
            buffer << mat;
            return *this;
        }
        template<typename T>
        LiTouchTerminalIS& operator<<(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& mat) {
            buffer << mat;
            return *this;
        }
        template<typename Derived>
        LiTouchTerminalIS& operator<<(const Eigen::DenseBase<Derived>& mat) {
            buffer << mat;
            return *this;
        }
        LiTouchTerminalIS& operator<<(LTEndlSignal end) {
            ros::Publisher& publisher = Livz::getInstance().getLTPublisher();
            std_msgs::String msg;
            std::string final_str = terminal_name + "#" + buffer.str() + "\n";
            msg.data = final_str;
            publisher.publish(msg);
            buffer.str("");
            buffer.clear();
            return *this;
        }
    private:
        std::string terminal_name;
        std::stringstream buffer;
    };
public:
    /**
     * @brief 协同延时函数，不会阻碍动画线程。
     * @param ms - double 延时的毫秒数
     */
    static void Delay(const double ms);
    static void DelayMicroseconds(const double microseconds);
    static void DelayAccordingtoAnimate(const LAnimateParam ani_param);


public:
    /**
     * @brief 自定义着色器类型。接受位置返回颜色(r,g,b,a)
     */
    typedef std::function<Eigen::Vector3d(const Eigen::Vector3d)> ShaderFunction;


    /**
     * @brief 参数化曲线。
     */
    typedef std::function<Eigen::Vector3d(const double)> ParametricCurveFunction;

    /**
     * @brief 参数化曲面。
     */
    typedef std::function<double(const double, const double )> ParametricSurfaceFunction;

    /**
     * @brief 向量场。
     */
    typedef std::function<Eigen::Vector3d(const Eigen::Vector3d)> VectorField;

    /**
     * @brief LitouchCmd Action函数。
     */
    typedef std::function<void(const int cmd_id, const std::vector<double> params)> LitouchCmdActionFunc;

    #define addUpdater(name, func) AddUpdater(name, func, __FILE__, __LINE__);
    #define removeUpdater(name) RemoveUpdater(name, __FILE__, __LINE__);

    /**
     * @brief 生成往litouch mini_terminal组件发送的流对象。
     * @param t_name std::string mini_terminal组件的标题（title)
     */
    static LiTouchTerminalIS LTout(const std::string& t_name){ return LiTouchTerminalIS(t_name); }

    /**
     * @brief 旋转一组点绕Z轴旋转给定的角度。
     * @param ori_points std::vector<Eigen::Vector3d>& - 要旋转的点的集合，结果也存储在此变量中。
     * @param rot_center Eigen::Vector3d - 旋转中心。
     * @param angle_phi double - 旋转的角度（弧度）。
     */
    static void rotatePointsSO1( std::vector<Eigen::Vector3d>& ori_points , const Eigen::Vector3d& rot_center, double angle_phi );

    /**
     * @brief 启动 livz touch 。
     * @param config_path string - 配置文件目录。
     */
    static void launchLiTouch(const std::string& config_path = "");

    /**
     * @brief litouch_cmd 话题回调
     * 
    */
    static void litouchCmdCbk(const std_msgs::Float64MultiArray::ConstPtr& msg);

    /**
     * @brief 解包后的 litouch_cmd 话题回调
    */
    static void defaultLitouchCmdAction(const int cmd_id, const std::vector<double> params);

    /**
     * @brief 设置LitouchAction函数
    */
    static void setLitouchAction(LitouchCmdActionFunc func);

    


    ////////////////////////////////////////////////////////////////////
    

    /**
     * @brief 设置多边形显示模式
     * @param type int - POLYGON_TYPE::TYPE_MARKER 使用Marker显示 ， POLYGON_TYPE::TYPE_POLYGON 使用Polygon显示
     */
    static void setPolygonType(const int type);

    /**
     * @brief 设置默认参考坐标系
     * @param default_frame_id std::string - 参考坐标系
     */
    static void setDefaultFrameId(const std::string& default_frame_id);



    template <typename Func, typename Params>
    static void createAnimate(const LAnimateParam& ani_param , Func func, Params params1, Params params2) { 
        // std::cout<<"create animate"<<std::endl;
        std::tuple topic_element = std::make_tuple(ani_param.topic_name_);
        std::tuple params1_f = std::tuple_cat( topic_element , params1 );
        std::tuple params2_f = std::tuple_cat( topic_element , params2 );


        std::shared_ptr<LAnimateTask<Func, decltype(params1_f)>> task = std::make_shared<LAnimateTask<Func, decltype(params1_f)>>(ani_param, func, params1_f, params2_f);
        getInstance().animate_tasks_.emplace_back( task );
    }

    static void AddUpdater( const std::string& updater_name, std::function<bool(const double)> update_function ,
                            const std::string& file = "Use addUpdater rather than AddUpdater to get more information", 
                            int line = -1);

    static void RemoveUpdater( const std::string& updater_name,
                               const std::string& file = "Use removeUpdater rather than RemoveUpdater to get more information", 
                               int line = -1);
    


    ///////////////// RVIZ 可视化部分
    ///////////////// RVIZ 可视化部分
    ///////////////// RVIZ 可视化部分

    /**
     * @brief 清除某个话题的所有已发布可视化内容。
     * @param topic_name string - 发布的主题名称。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     */
    static void clearAll(const std::string& topic_name ,const std::string& frame_id = CONFIG::default_frame_id);

    /**
     * @brief 清除屏幕中所有Livz已发布的内容。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     */
    static void clearScreen(const std::string& frame_id = CONFIG::default_frame_id);


    /**
     * @brief 绘制一个点。
     * @param topic_name string - 发布的主题名称。
     * @param position Eigen::Vector3d - 点的位置。
     * @param size double - 点的大小 (默认为0.5)。
     * @param color Eigen::Vector4d - 点的颜色（默认为黄色）。
     * @param opacity double - 点的不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 点的标识符（默认为114514）。
     */
    static void drawOnePoint( const std::string& topic_name, 
                               const Eigen::Vector3d& position, 
                               double size = 0.5, 
                               Eigen::Vector4d color = LCOLOR::YELLOW, 
                               double opacity = -1.0, 
                               std::string frame_id = CONFIG::default_frame_id, 
                               int id = 114514) ;
    
    /**
     * @brief 绘制多个点。
     * @param topic_name string - 发布的主题名称。
     * @param positions std::vector<Eigen::Vector3d> - 点的位置数组。
     * @param sizes std::vector<double> - 点的大小数组。
     * @param colors std::vector<Eigen::Vector4d> - 点的颜色数组。
     * @param opacities std::vector<double> - 点的不透明度数组。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 点的起始标识符（默认为1114514）。
     */
    static void drawPoints( const std::string& topic_name, 
                            const std::vector<Eigen::Vector3d>& positions, 
                            std::vector<double> sizes = {},
                            std::vector<Eigen::Vector4d> colors = {}, 
                            std::vector<double> opacities = {},
                            std::string frame_id = CONFIG::default_frame_id, 
                            int id = 1114514);
    
    /**
     * @brief 绘制一个球体。
     * @param topic_name string - 发布的主题名称。
     * @param position Eigen::Vector3d - 球心位置。
     * @param radius double - 球体半径。
     * @param color Eigen::Vector4d - 球体颜色（默认为黄色）。
     * @param opacity double - 球体不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 球体标识符（默认为124514）。
     */
    static void drawOneSphere(const std::string& topic_name,
                       const Eigen::Vector3d& position,
                       double radius,
                       Eigen::Vector4d color = LCOLOR::YELLOW,
                       double opacity = -1.0,
                       std::string frame_id = CONFIG::default_frame_id,
                       int id = 124514);
    
    /**
     * @brief 绘制圆柱体。
     * @param topic_name string - 发布的主题名称。
     * @param position Eigen::Vector3d - 圆柱体中心的位置（x, y, z坐标）。
     * @param radius double - 圆柱体的半径。
     * @param height double - 圆柱体的高度。
     * @param color Eigen::Vector4d - 圆柱体颜色（默认为金色）。
     * @param opacity double - 球体不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 圆柱体的标识号（默认为 104514）。
     */
    static void drawOneCylinder(const std::string& topic_name,
                                Eigen::Vector3d position,
                                double radius,
                                double height,
                                Eigen::Vector4d color = LCOLOR::GOLD,
                                double opacity = -1.0,
                                std::string frame_id = CONFIG::default_frame_id,
                                int id = 104514);

    /**
     * @brief 绘制立方体。
     * @param topic_name string - 发布的主题名称。
     * @param position Eigen::Vector3d - 立方体中心的位置（x, y, z坐标）。
     * @param scale Eigen::Vector3d - 立方体的尺寸（长度、宽度、高度）。
     * @param color Eigen::Vector4d - 立方体颜色（默认为金色）。
     * @param opacity double - 球体不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 立方体的标识号（默认为 204514）。
     */
    static void drawOneCube(const std::string& topic_name,
                        Eigen::Vector3d position,
                        Eigen::Vector3d scale,
                        Eigen::Vector4d color = LCOLOR::GOLD,
                        double opacity = -1.0,
                        std::string frame_id = CONFIG::default_frame_id,
                        int id = 94514);
    
    /**
     * @brief 绘制箭头。
     * @param topic_name string - 发布的主题名称。
     * @param start_point Eigen::Vector3d - 箭头起始点的位置（x, y, z坐标）。
     * @param end_point Eigen::Vector3d - 箭头终点的位置（x, y, z坐标）。
     * @param scale Eigen::Vector3d - 箭头的尺寸（柄宽，箭宽，箭长）。
     * @param color Eigen::Vector4d - 箭头颜色（默认为红色）。
     * @param opacity double - 球体不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 箭头的标识号（默认为 845146）。
     */
    static void drawOneArrow(const std::string& topic_name,
                            Eigen::Vector3d start_point,
                            Eigen::Vector3d end_point,
                            Eigen::Vector3d scale = Eigen::Vector3d(0.1, 0.3,0.3),
                            Eigen::Vector4d color = LCOLOR::RED,
                            double opacity = -1.0,
                            std::string frame_id = CONFIG::default_frame_id,
                            int id = 84514);

    /**
     * @brief 绘制一组箭头。
     * @attention 慎用，图形复杂，会导致卡顿.
     * @param topic_name string - 发布的主题名称。
     * @param start_points std::vector<Eigen::Vector3d> - 箭头起始点的位置（x, y, z坐标）的集合。
     * @param end_points std::vector<Eigen::Vector3d> - 箭头终点的位置（x, y, z坐标）的集合。
     * @param scales std::vector<Eigen::Vector3d> - 箭头的尺寸（柄宽，箭宽，箭长）的集合（默认值可能根据实际需求设置）。
     * @param color Eigen::Vector4d - 箭头颜色。
     * @param opacity double - 箭头不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 箭头的标识号（默认值54514）。
     */
    static void drawArrows(const std::string& topic_name,
                      const std::vector<Eigen::Vector3d>& start_points,
                      const std::vector<Eigen::Vector3d>& end_points,
                      const std::vector<Eigen::Vector3d>& scales,
                      const Eigen::Vector4d& color = LCOLOR::RED,
                      double opacity = -1.0,
                      const std::string& frame_id = CONFIG::default_frame_id,
                      int id = 54514);

    /**
     * @brief 绘制一组具有不同颜色的箭头。
     * @attention 慎用，图形复杂，会导致卡顿.
     * @param topic_name string - 发布的主题名称。
     * @param start_points std::vector<Eigen::Vector3d> - 箭头起始点的位置（x, y, z坐标）的集合。
     * @param end_points std::vector<Eigen::Vector3d> - 箭头终点的位置（x, y, z坐标）的集合。
     * @param scales std::vector<Eigen::Vector3d> - 箭头的尺寸（柄宽，箭宽，箭长）的集合（默认值可能根据实际需求设置）。
     * @param colors std::vector<Eigen::Vector4d> - 箭头颜色的集合。
     * @param opacity double - 箭头不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 箭头的标识号（默认值74514）。
     */
    static void drawArrowsWithColors(const std::string& topic_name,
                                    const std::vector<Eigen::Vector3d>& start_points,
                                    const std::vector<Eigen::Vector3d>& end_points,
                                    const std::vector<Eigen::Vector3d>& scales,
                                    const std::vector<Eigen::Vector4d>& colors,
                                    double opacity = -1.0,
                                    const std::string& frame_id = CONFIG::default_frame_id,
                                    int id = 74514);

    
    /**
     * @brief 绘制一条折线。
     * @param topic_name string - 发布的主题名称。
     * @param points std::vector<Eigen::Vector3d> - 折线的顶点。
     * @param width double - 折线的宽度。
     * @param color Eigen::Vector4d - 折线颜色（默认为黄色）。
     * @param opacity double - 折线的不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 折线标识符（默认为134514）。
     */
    static void drawLineStrip(const std::string& topic_name,
                          const std::vector<Eigen::Vector3d>& points,
                          double width,
                          Eigen::Vector4d color = LCOLOR::YELLOW,
                          double opacity = -1.0,
                          std::string frame_id = CONFIG::default_frame_id,
                          int id = 134514);
    
    /**
     * @brief 绘制线段群。
     * @param topic_name string - 发布的主题名称。
     * @param points std::vector<Eigen::Vector3d> - 线段群顶点。
     * @param width double - 线段的宽度。
     * @param color Eigen::Vector4d - 线段颜色（默认为黄色）。
     * @param opacity double - 线段的不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 线段标识符（默认为139514）。
     */
    static void drawLineList(    const std::string& topic_name,
                                 const std::vector<Eigen::Vector3d>& points,
                                 double width,
                                 Eigen::Vector4d color = LCOLOR::YELLOW,
                                 double opacity = -1.0,
                                 std::string frame_id = CONFIG::default_frame_id,
                                 int id = 139514);

    /**
     * @brief 绘制具有不同颜色的线段列表。
     * @param topic_name string - 发布的主题名称。
     * @param points std::vector<Eigen::Vector3d> - 线段起始和结束点的位置（x, y, z坐标）。每两个点代表一条线段。
     * @param colors std::vector<Eigen::Vector4d> - 线段的颜色列表。每个颜色对应于一条线段。
     * @param width double - 线段的宽度。
     * @param opacity double - 线段不透明度（默认使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 线段的标识号（默认为64514）。
     */
    static void drawLineListWithColors(const std::string& topic_name,
                                       const std::vector<Eigen::Vector3d>& points,
                                       const std::vector<Eigen::Vector4d>& colors,
                                       double width,
                                       double opacity = -1.0,
                                       const std::string& frame_id = CONFIG::default_frame_id,
                                       int id = 64514);

    /**
     * @brief 绘制一个2D多边形。
     * @param topic_name string - 发布的主题名称。
     * @param vertices std::vector<Eigen::Vector3d> - 多边形的顶点(第3维记录顶点所在平面高度)。
     * @param color Eigen::Vector4d - 多边形的颜色（默认为黄色）。
     * @param stroke_width double - 多边形描边宽度。
     * @param opacity double - 多边形的不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 多边形标识符（默认为144514）。
     */
    static void draw2DPolygon_MARKER(const std::string& topic_name,
                                     const std::vector<Eigen::Vector3d>& vertices,
                                     Eigen::Vector4d color = LCOLOR::YELLOW,
                                     double stroke_width    = 0.1,
                                     double opacity  = -1.0,
                                     std::string frame_id = CONFIG::default_frame_id,
                                     int id = 144514);
    
    /**
     * @brief 绘制一个2D多边形。
     * @param topic_name string - 发布的主题名称。
     * @param vertices std::vector<Eigen::Vector3d> - 多边形的顶点(第3维记录顶点所在平面高度)。
     * @param color Eigen::Vector4d - 多边形的颜色（默认为黄色）。
     * @param stroke_width double - 多边形描边宽度。
     * @param opacity double - 多边形的不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 多边形标识符（默认为144514）。
     */
    static void draw2DPolygon_POLYGON(const std::string& topic_name,
                                     const std::vector<Eigen::Vector3d>& vertices,
                                     Eigen::Vector4d color = LCOLOR::YELLOW,
                                     double stroke_width    = 0.1,
                                     double opacity  = -1.0,
                                     std::string frame_id = CONFIG::default_frame_id,
                                     int id = 144514);
    

    
    /**
     * @brief 绘制一个2D矩形。
     * @param topic_name string - 发布的主题名称。
     * @param top_left Eigen::Vector3d - 矩形的左上顶点。
     * @param width double - 矩形的宽度。
     * @param height double - 矩形的高度。
     * @param rot_z double - 旋转弧度（默认为0）
     * @param color Eigen::Vector4d - 矩形的颜色（默认为黄色）。
     * @param stroke_width double - 描边宽度。
     * @param opacity double - 矩形的不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 矩形标识符（默认为154514）。
     */
    static void draw2DRect( const std::string& topic_name,
                            const Eigen::Vector3d& top_left,
                            double width, double height,
                            double rot_z = 0.0,
                            Eigen::Vector4d color = LCOLOR::YELLOW,
                            double stroke_width    = 0.1,
                            double opacity = -1.0,
                            std::string frame_id = CONFIG::default_frame_id,
                            int id = 154514);

    /**
     * @brief 绘制一个2D圆。
     * @param topic_name string - 发布的主题名称。
     * @param center Eigen::Vector3d - 圆心位置。
     * @param radius double - 圆的半径。
     * @param color Eigen::Vector4d - 圆的颜色（默认为黄色）。
     * @param stroke_width double - 描边宽度。
     * @param opacity double - 圆的不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 圆的标识符（默认为164514）。
     * @param samples int - 圆的采样密度，决定圆的平滑度（默认为100）。
     */
    static void draw2DCircle(const std::string& topic_name,
                             const Eigen::Vector3d& center,
                             double radius,
                             Eigen::Vector4d color = LCOLOR::YELLOW,
                             double stroke_width    = 0.1,
                             double opacity = -1.0,
                             std::string frame_id = CONFIG::default_frame_id,
                             int id = 164514,
                             int samples = 100);
    
    /**
     * @brief 绘制一个2D椭圆。
     * @param topic_name string - 发布的主题名称。
     * @param center Eigen::Vector3d - 椭圆中心位置。
     * @param major_axis double - 椭圆沿x方向的轴长。
     * @param minor_axis double - 椭圆沿y方向的轴长。
     * @param rot_z double - 旋转弧度（默认为0）
     * @param color Eigen::Vector4d - 椭圆的颜色（默认为黄色）。
     * @param stroke_width double - 描边宽度。
     * @param opacity double - 椭圆的不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 椭圆的标识符（默认为174514）。
     * @param samples int - 椭圆的采样密度，决定椭圆的平滑度（默认为100）。
     */
    static void draw2DEllipse(const std::string& topic_name,
                          const Eigen::Vector3d& center,
                          double major_axis, double minor_axis, 
                          double rot_z = 0.0,
                          Eigen::Vector4d color = LCOLOR::YELLOW,
                          double stroke_width    = 0.1,
                          double opacity = -1.0,
                          std::string frame_id = CONFIG::default_frame_id,
                          int id = 174514,
                          int samples = 100);
    

    /**
     * @brief 绘制一个2D圆锥,默认锥轴沿x轴正向。
     * @param topic_name string - 发布的主题名称。
     * @param center Eigen::Vector3d - 圆锥顶点位置。
     * @param radius double - 半径。
     * @param fov double - 锥角（弧度）。
     * @param rot_z double - 旋转弧度（默认为0）
     * @param color Eigen::Vector4d - 圆锥的颜色（默认为黄色）。
     * @param stroke_width double - 描边宽度。
     * @param opacity double - 圆锥的不透明度（默认为-1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 标识符（默认为184514）。
     * @param samples int - 采样密度，决定圆锥的平滑度（默认为40）。
     */
    static void draw2DCone(const std::string& topic_name,
                          const Eigen::Vector3d& center,
                          double radius,
                          double fov,
                          double rot_z = 0.0,
                          Eigen::Vector4d color = LCOLOR::YELLOW,
                          double stroke_width    = 0.1,
                          double opacity = -1.0,
                          std::string frame_id = CONFIG::default_frame_id,
                          int id = 184514,
                          int samples = 40);
    
    /**
     * @brief 绘制点云。
     * @param topic_name string - 发布的主题名称。
     * @param cloud pcl::PointCloud<pcl::PointXYZ>::Ptr - 点云数据。
     * @param color Eigen::Vector4d - 点的颜色（默认为白色）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     */
    static void drawPointcloud(const std::string& topic_name,
                              pcl::PointCloud<pcl::PointXYZ>& cloud,
                              Eigen::Vector4d color = LCOLOR::WHITE,
                              std::string frame_id  = CONFIG::default_frame_id);
    
    /**
     * @brief 绘制彩色点云。
     * @param topic_name string - 发布的主题名称。
     * @param cloud pcl::PointCloud<pcl::PointXYZRGB>::Ptr - 已着色的点云数据。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     */
    static void drawPointcloudRGB( const std::string& topic_name,
                                pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                                std::string frame_id = CONFIG::default_frame_id );
    
    /**
     * @brief 绘制带强度信息点云。
     * @param topic_name string - 发布的主题名称。
     * @param cloud pcl::PointCloud<pcl::PointXYZI>::Ptr - 携带强度信息的点云。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     */
    static void drawPointcloudI( const std::string& topic_name,
                                pcl::PointCloud<pcl::PointXYZI>& cloud,
                                std::string frame_id = CONFIG::default_frame_id );
    
    /**
     * @brief 绘制自定义着色点云。
     * @param topic_name string - 发布的主题名称。
     * @param cloud pcl::PointCloud<pcl::PointXYZ>::Ptr - 点云数据。
     * @param shader ShaderFunction - 自定义的着色器函数。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     */
    static void drawPointcloudWithShader( const std::string& topic_name,
                                        pcl::PointCloud<pcl::PointXYZ>& cloud,
                                        ShaderFunction shader,
                                        std::string frame_id = CONFIG::default_frame_id );
    
    /**
     * @brief 绘制参数化曲线。
     * @param topic_name string - 发布的主题名称。
     * @param curve_function ParametricCurveFunction - 参数化曲线函数。
     * @param interval Eigen::Vector2d - 绘制的参数区间。
     * @param color Eigen::Vector4d - 曲线颜色（默认为金色）。
     * @param stroke_width double - 线条宽度（默认为 0.1）。
     * @param opacity double - 曲线不透明度（默认为 -1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 曲线的标识号（默认为 194514）。
     * @param sample_gap double - 空间采样间隔（默认为 0.1， 可能会不准）。
     */
    static void drawParametricCurve( const std::string& topic_name,
                                     ParametricCurveFunction curve_function,
                                     Eigen::Vector2d interval,
                                     Eigen::Vector4d color = LCOLOR::GOLD,
                                     double stroke_width    = 0.1,
                                     double opacity = -1.0,
                                     std::string frame_id = CONFIG::default_frame_id,
                                     int id = 194514,
                                     double sample_gap = 0.1 );
    
   /**
     * @brief 绘制始终朝向视口的文字。
     * @param topic_name string - 发布的主题名称。
     * @param text string - 要绘制的文本内容。
     * @param position Eigen::Vector3d - 文字绘制的位置（x, y , z坐标）。
     * @param size double - 文字的大小（高度 , 默认为1）。
     * @param color Eigen::Vector4d - 文字颜色（默认为金色）。
     * @param opacity double - 文字的不透明度（默认为 -1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 文字的标识号（默认为 204514）。
     */
    static void drawViewText( const std::string& topic_name,
                              const std::string& text,
                              Eigen::Vector3d position = Eigen::Vector3d(0,0,5),
                              double size = 1,
                              Eigen::Vector4d color = LCOLOR::GOLD,
                              double opacity = -1.0,
                              std::string frame_id = CONFIG::default_frame_id,
                              int id = 204514);

    /**
     * @brief 绘制不同颜色的三角面。
     * @param topic_name string - 发布的主题名称。
     * @param mesh_vertexes Eigen::Matrix3Xd - 三角顶点信息（按列排布）。
     * @param colors std::vector<Eigen::Vector4d> - 颜色。
     * @param opacity double - 不透明度（默认为 -1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 文字的标识号（默认为 904514）。
     */
    static void renderTrianglesWithColors(const std::string& topic_name, 
                                 Eigen::Matrix3Xd &mesh_vertexes, 
                                 const std::vector<Eigen::Vector4d>& colors, 
                                 double opacity = -1.0,
                                 std::string frame_id = CONFIG::default_frame_id,
                                 int id = 904514);

    /**
     * @brief 绘制三角网格体。
     * @param topic_name string - 发布的主题名称。
     * @param mesh_vertexes Eigen::Matrix3Xd - 网格体顶点信息（按列排布）。
     * @param color Eigen::Vector4d - 颜色（默认为天蓝）。
     * @param opacity double - 不透明度（默认为 -1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 文字的标识号（默认为 214514）。
     */
    static void renderTriangleMesh(const std::string& topic_name, 
                                 Eigen::Matrix3Xd &mesh_vertexes, 
                                 Eigen::Vector4d color = LCOLOR::AZURE, 
                                 double opacity = -1.0,
                                 std::string frame_id = CONFIG::default_frame_id,
                                 int id = 214514);

    /**
     * @brief 绘制三角网格体以及边棱（内测版本API）
     * @param topic_name string - 发布的主题名称。
     * @param U Eigen::MatrixXd - 顶点信息（按列排布）。
     * @param G Eigen::MatrixXi - 顶点索引。
     * @param color Eigen::Vector4d - 颜色（默认为天蓝）。
     * @param opacity double - 不透明度（默认为 -1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 文字的标识号（默认为 531241）。
     */
    static void renderTriangleMeshWithEdges(const std::string& meshtopic, 
                                            const std::string& edgetopic, 
                                            Eigen::MatrixXd &U, 
                                            Eigen::MatrixXi &G, 
                                            Eigen::Vector4d color= LCOLOR::AZURE, 
                                            double opacity = -1.0,
                                            std::string frame_id=CONFIG::default_frame_id,
                                            int id=531241);
    
     /**
     * @brief 绘制三角网格体的棱。
     * @param topic_name string - 发布的主题名称。
     * @param mesh_vertexes Eigen::Matrix3Xd - 网格体顶点信息（按列排布）。
     * @param color Eigen::Vector4d - 颜色（默认为黑色）。
     * @param stroke_width double - 线条宽度（默认为 0.1）。
     * @param opacity double - 不透明度（默认为 -1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 文字的标识号（默认为 224514）。
     */
    static void renderTriangleMeshEdges( const std::string& topic_name, 
                                        Eigen::Matrix3Xd &mesh_vertexes, 
                                        Eigen::Vector4d color = LCOLOR::BLACK, 
                                        double stroke_width = 0.1,
                                        double opacity = -1.0,
                                        std::string frame_id = CONFIG::default_frame_id,
                                        int id = 224514);
    
    /**
     * @brief 绘制二元函 z = f(u,v)（参数曲面）。
     * @param topic_name string - 发布的主题名称。
     * @param surf_function ParametricSurfaceFunction - 二元函数指针。
     * @param u_range Eigen::Vector2d - u的范围（默认[-1,1])
     * @param v_range Eigen::Vector2d - v的范围（默认[-1,1])
     * @param u_res double - u的采样精度（默认0.25)
     * @param v_res double - v的采样精度（默认0.25)
     * @param color Eigen::Vector4d - 颜色（默认为金色）。
     * @param opacity double - 不透明度（默认为 -1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 标识号（默认为 224514）。
     */
    static void renderSurface(const std::string& topic_name, 
                                        ParametricSurfaceFunction surf_function,
                                        Eigen::Vector2d u_range =  Eigen::Vector2d(-1,1),
                                        Eigen::Vector2d v_range =  Eigen::Vector2d(-1,1),
                                        double u_res = 0.25,
                                        double v_res = 0.25,
                                        Eigen::Vector4d color = LCOLOR::GOLD, 
                                        double opacity = -1.0,
                                        std::string frame_id = CONFIG::default_frame_id,
                                        int id = 234514);
    

    /**
     * @brief 可视化向量场。
     * @param topic_name string - 发布的主题名称。
     * @param field_function VectorField - 向量场函数指针。
     * @param box_min Eigen::Vector3d - 显示BOX的下边界（默认（-1,-1,-1））。
     * @param box_max Eigen::Vector3d - 显示BOX的上边界（默认（1,1,1））。
     * @param resolution Eigen::Vector3d - 分辨率，用于定义向量场的间距（默认为 (0.2, 0.2, 0.2)）。
     * @param only_2d bool - 是否仅渲染2D向量场,如果是，则渲染box_min.z()的一层（默认为 false）。
     * @param color_min Eigen::Vector4d - 颜色范围的最小值（默认为红色）。
     * @param color_max Eigen::Vector4d - 颜色范围的最大值（默认为金色）。
     * @param opacity double - 不透明度（默认为 -1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 标识号（默认为 244514）。
     */
    static void renderVectorField(  const std::string& topic_name, 
                                    VectorField field_function,
                                    Eigen::Vector3d box_min =  Eigen::Vector3d(-1,-1,-1),
                                    Eigen::Vector3d box_max =  Eigen::Vector3d( 1, 1, 1),
                                    Eigen::Vector3d resolution = Eigen::Vector3d( 0.2, 0.2, 0.2),
                                    const bool only_2d      = false,
                                    Eigen::Vector4d color_min = LCOLOR::RED, 
                                    Eigen::Vector4d color_max = LCOLOR::GOLD, 
                                    double opacity = -1.0,
                                    std::string frame_id = CONFIG::default_frame_id,
                                    int id = 244514);
    
    /**
     * @brief 可视化向量场的流线。
     * @param topic_name string - 发布的主题名称。
     * @param field_function VectorField - 向量场函数指针。
     * @param box_min Eigen::Vector3d - 显示BOX的下边界（默认（-1,-1,-1））。
     * @param box_max Eigen::Vector3d - 显示BOX的上边界（默认（1,1,1））。
     * @param resolution Eigen::Vector3d - 分辨率，用于定义向量场的间距（默认为 (0.2, 0.2, 0.2)）。
     * @param start_step int - 起始步
     * @param end_step int - 终止步
     * @param only_2d bool - 是否仅渲染2D向量场,如果是，则渲染box_min.z()的一层（默认为 false）。
     * @param color_min Eigen::Vector4d - 颜色范围的最小值（默认为红色）。
     * @param color_max Eigen::Vector4d - 颜色范围的最大值（默认为金色）。
     * @param opacity double - 不透明度（默认为 -1.0，使用颜色中的alpha值）。
     * @param frame_id string - 参考坐标系（默认为"/map"）。
     * @param id int - 标识号（默认为 244514）。
     */
    static void renderStreamLines(  const std::string& topic_name, 
                                    VectorField field_function,
                                    Eigen::Vector3d box_min =  Eigen::Vector3d(-1,-1,-1),
                                    Eigen::Vector3d box_max =  Eigen::Vector3d( 1, 1, 1),
                                    Eigen::Vector3d resolution = Eigen::Vector3d( 0.2, 0.2, 0.2),
                                    int start_step            = 0,
                                    int end_step              = 10,
                                    const bool only_2d        = false,
                                    Eigen::Vector4d color_min = LCOLOR::RED, 
                                    Eigen::Vector4d color_max = LCOLOR::GOLD, 
                                    double opacity = -1.0,
                                    std::string frame_id = CONFIG::default_frame_id,
                                    int id = 244514);


    static std::function<void(const std::string&, const std::vector<Eigen::Vector3d>&, Eigen::Vector4d, double, double , std::string, int)> draw2DPolygon;
    static std::function<void(const int cmd_id, const std::vector<double> params)> litouchCmdAction;

private:
    void init();
    Livz(){init();}
    static Livz& getInstance();

    ros::Timer tick_timer;
    ros::Timer cu_tick_timer;
    ros::Publisher litouch_terminal_msg_pub;
    ros::Subscriber litouch_cmd_sub;
    void LivzTick(const ros::TimerEvent& event);
    void LivzCusUpdaterTick(const ros::TimerEvent& event);

    template <typename T = visualization_msgs::Marker>
    static ros::Publisher& getPublisher(const std::string& topic_name);
    static ros::Publisher& getLTPublisher();
    static std::map<std::string, PublisherInfo>& getPublishers();
    static bool findPublisher(const std::string& topic_name, PublisherInfo& finded_publisher);

private:

    ros::NodeHandle nh_;
    std::map<std::string, PublisherInfo>    publishers_;
    std::vector<LAnimateTaskWrapper>        animate_tasks_;
    std::vector<LCustomUpdater>             customed_updaters_;
    static const std::string version_;

};

#endif // LROSVIS_LIB_H


