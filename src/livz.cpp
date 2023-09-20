// Copyright (c) [2023] [Lantern]
// 
// This file is part of [Livz]
// 
// This project is licensed under the MIT License.
// See LICENSE.txt for details.
#include "include/livz.hpp"

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */

#define PRINT_COLOR_TEXT(text, color) \
    std::cout << color << text << RESET << std::endl


const std::string Livz::version_ = "BETA 1.2";
std::function<void(const std::string&, const std::vector<Eigen::Vector3d>&, Eigen::Vector4d, double, double , std::string, int)> Livz::draw2DPolygon = Livz::draw2DPolygon_MARKER;
std::function<void(const int cmd_id, const std::vector<double> params)> Livz::litouchCmdAction = Livz::defaultLitouchCmdAction;

namespace CONFIG{
    std::string default_frame_id = "map";
}


void Livz::LivzTick(const ros::TimerEvent& event)
{
    auto it = animate_tasks_.begin();
    while (it != animate_tasks_.end()) {
        it->Update();
        if (it->Finished()) {
            it = animate_tasks_.erase(it); // erase会返回下一个有效的迭代器
        } else {
            ++it;
        }
    }
}


void Livz::LivzCusUpdaterTick(const ros::TimerEvent& event)
{
    auto it = customed_updaters_.begin();
    while (it != customed_updaters_.end()) {
        it->Update();
        if (it->Finished()) {
            it = customed_updaters_.erase(it); // erase会返回下一个有效的迭代器
        } else {
            ++it;
        }
    }
}

Livz& Livz::getInstance() {
    static bool first_call = true;
    if (first_call){
        int fake_argc = 1;
        char* fake_argv[] = {(char*)"Lantern", NULL};
        long long unique_id = (static_cast<long long>(std::time(nullptr)) << 32) | getpid();
        std::string node_name = "livz_node_" + std::to_string(unique_id);
        ros::init(fake_argc, fake_argv, "livz_node");
        first_call = false;
    }

    static Livz instance;
    return instance;
}


template <typename T = visualization_msgs::Marker>
ros::Publisher& Livz::getPublisher(const std::string& topic_name) {
    PublisherInfo &publisher_info = getInstance().publishers_[topic_name];
    ros::Publisher& publisher = publisher_info.publisher;
    if (publisher.getTopic().empty()) {
        publisher_info.publisher = getInstance().nh_.advertise<T>(topic_name, 10000);
        publisher_info.type      = typeid(T).name();
        double wait_time = 0.0;
        while (publisher.getNumSubscribers() < 1) {
            Livz::Delay(100);
            wait_time += 0.1;
            if(wait_time > 2.0){
                PRINT_COLOR_TEXT("[Livz] 话题 \"" << topic_name << "\" 似乎没有订阅者，请检查rviz配置。\n"
                 << "[Livz] Looks like Topic \"" << topic_name << "\" is not subscribed by any subscriber. Check rviz config.",  YELLOW);
                break;
            }
        } // 解决第一次发布看不到的bug
    }
    return publisher;
}

ros::Publisher& Livz::getLTPublisher() { 
    return getInstance().litouch_terminal_msg_pub; 
} 

std::map<std::string, PublisherInfo>& Livz::getPublishers()
{
    return getInstance().publishers_;
}

bool Livz::findPublisher(const std::string& topic_name, PublisherInfo& finded_publisher) {
    PublisherInfo& publisher_info = getInstance().publishers_[topic_name];
    ros::Publisher& publisher = publisher_info.publisher;
    if (publisher.getTopic().empty()) {
        return false;
    }
    finded_publisher = publisher_info;
    return true;
}



void Livz::Delay(const double ms)
{
    ros::Time start_time = ros::Time::now();
    ros::Duration delay_duration(ms / 1000.0);
    while (ros::Time::now() - start_time < delay_duration)
    {
        ros::spinOnce();
    }
}

void Livz::DelayMicroseconds(const double microseconds)
{
    ros::Time start_time = ros::Time::now();
    ros::Duration delay_duration(microseconds / 1000000.0);
    while (ros::Time::now() - start_time < delay_duration)
    {
        ros::spinOnce();
    }
}

void Livz::DelayAccordingtoAnimate(const LAnimateParam ani_param)
{
    Livz::Delay( 1000*ani_param.duration_ );
}

void Livz::init() {


    PRINT_COLOR_TEXT("Welcome to use Livz! ", YELLOW);
    PRINT_COLOR_TEXT("version： "<< version_ , YELLOW);
    draw2DPolygon                   = Livz::draw2DPolygon_MARKER;
    litouchCmdAction                = Livz::defaultLitouchCmdAction;
    tick_timer        = nh_.createTimer(ros::Duration(0.01),   [&](const ros::TimerEvent& event) { return LivzTick(event); });
    cu_tick_timer     = nh_.createTimer(ros::Duration(0.01),   [&](const ros::TimerEvent& event) { return LivzCusUpdaterTick(event); });
    litouch_cmd_sub   = nh_.subscribe("/litouch_cmd", 1000, Livz::litouchCmdCbk);
    litouch_terminal_msg_pub = nh_.advertise<std_msgs::String>("/litouch_terminal_msg", 1000);
}

void Livz::AddUpdater( const std::string& updater_name, std::function<bool(const double)> update_function ,const std::string& file, int line ) {

    for(LCustomUpdater updater : getInstance().customed_updaters_ ) {
        if( updater.updater_name_ == updater_name) {
            PRINT_COLOR_TEXT("[Livz] 名为 \"" << updater_name <<" \" 的updater已存在且尚未结束，本次操作将不会覆盖。\n"
                            <<"[Livz] The updater called \""<<updater_name<<" \" already exists and is still running, this operation will not overwrite it. \n"
                            <<"[Livz] "<<file<<" : "<<line ,  CYAN);
            return;
        }
    }
    getInstance().customed_updaters_.emplace_back( LCustomUpdater( updater_name, update_function ) );
}

void Livz::RemoveUpdater( const std::string& updater_name, const std::string& file, int line ) {

    auto& updaters = getInstance().customed_updaters_;
    auto it = std::remove_if(updaters.begin(), updaters.end(),
                             [&updater_name](const LCustomUpdater& updater) {
                                 return updater.updater_name_ == updater_name;
                             });

    if (it != updaters.end()) {
        updaters.erase(it, updaters.end());
    } else {
        PRINT_COLOR_TEXT("[Livz] 名为 \"" << updater_name <<" \" 的updater不存在，因此无法移除。\n"
                            <<"[Livz] The updater called \""<<updater_name<<" \" does not exist, and therefore cannot be removed. \n"
                            <<"[Livz] "<<file<<" : "<<line, CYAN);
    }
}


void Livz::setPolygonType(const int type)
{
    if(type == POLYGON_TYPE::TYPE_MARKER){
        draw2DPolygon = Livz::draw2DPolygon_MARKER;
        PRINT_COLOR_TEXT("[Livz] 多边形设置为Marker绘制。 \n"
                         <<"[Livz] Polygon type set to Marker. " , GREEN);
    }
    if(type == POLYGON_TYPE::TYPE_POLYGON){
        draw2DPolygon = Livz::draw2DPolygon_POLYGON;
        PRINT_COLOR_TEXT("[Livz] 多边形设置为Polygon绘制。 \n"
                         <<"[Livz] Polygon type set to Polygon. " , GREEN);
    }
}


void Livz::setDefaultFrameId(const std::string& default_frame_id)
{
    CONFIG::default_frame_id = default_frame_id;
}


void Livz::launchLiTouch(const std::string& config_path )
{
    getInstance(); //如果没有对象则强制创建对象
    std::string python_comand = "python";
    std::string versions[] = {"3.10","3.9","3.8","3.7","3.6","3.5","3.4",""};
    for (std::string version : versions){
        std::string command = "python" + version + " --version";
        bool res            = (std::system(command.c_str()) == 0);
        if (res) {
            python_comand += version;
            
            PRINT_COLOR_TEXT("[LiTouch] 使用"+python_comand+"启动。" , GREEN);
            break;
        }
    }

    pid_t pid = fork();
    if (pid == 0) {
        const std::string const_config_path = config_path;
        if (const_config_path.empty()) {
            execlp(python_comand.c_str(), python_comand.c_str(), LIVZ_TOUCH_PATH, nullptr);
        } else {
            execlp(python_comand.c_str(), python_comand.c_str(), LIVZ_TOUCH_PATH, const_config_path.c_str(), nullptr);
        }
    } else if (pid > 0) {
        // wait(NULL);  // 不等待子进程完成
    } else {
        // fork失败
    }
}

void Livz::litouchCmdCbk(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    const int size       = msg->data.size();
    const int param_size = size - 1;
    int cmd_id = -1;
    std::vector<double> params;
    if( size > 0 ) {
        cmd_id = round(msg->data[0]);
        if( param_size > 0 ) {
            for(size_t i = 0 ; i < param_size; i++ ) {
                params.push_back( msg->data[i + 1] );
            }
        }
        Livz::litouchCmdAction(cmd_id , params);
    }
}


void Livz::setLitouchAction(LitouchCmdActionFunc func)
{
    Livz::litouchCmdAction = func;
}

void Livz::defaultLitouchCmdAction(const int cmd_id, const std::vector<double> params)
{
    PRINT_COLOR_TEXT("[Livz] 接受到livztouch_cmd, id = "<<cmd_id<<" , 参数长度 = "<<params.size(), CYAN );
}

void Livz::rotatePointsSO1(std::vector<Eigen::Vector3d>& ori_points, const Eigen::Vector3d& rot_center, double angle_phi) {
    Eigen::Matrix2d rotation_matrix;
    double cphi = cos(angle_phi);
    double sphi = sin(angle_phi);
    double rc_x = rot_center(0);
    double rc_y = rot_center(1);

    rotation_matrix << cphi,  -sphi,
                       sphi,   cphi;

    for (Eigen::Vector3d& point : ori_points) {
        Eigen::Vector2d xy_point(point(0) - rc_x, point(1) - rc_y);
        xy_point = rotation_matrix * xy_point;
        point(0) = xy_point(0) + rc_x;
        point(1) = xy_point(1) + rc_y;
    }
}

void Livz::clearAll(const std::string& topic_name, const std::string& frame_id)
{
    PublisherInfo publisher_info;
    bool find = getInstance().findPublisher(topic_name, publisher_info);
    if(find == false){
        PRINT_COLOR_TEXT("[Livz] 尚未发布话题 \""<<topic_name<<"\"。\n"
                          <<"[Livz] Topic \""<<topic_name<<"\" has not been published. " , YELLOW);
        return ;
    }
    
    ros::Publisher& publisher  = publisher_info.publisher;
    std::string publisher_type = publisher_info.type;

    if (publisher_type == typeid(visualization_msgs::Marker).name()) {
        visualization_msgs::Marker deleteall_sig;
        deleteall_sig.action = visualization_msgs::Marker::DELETEALL;
        deleteall_sig.header.frame_id = frame_id;
        publisher.publish(deleteall_sig);
    }
    else if (publisher_type == typeid(visualization_msgs::MarkerArray).name())
    {
        visualization_msgs::MarkerArray deleteall_sig;
        visualization_msgs::Marker deleteall_marker;
        deleteall_marker.action = visualization_msgs::Marker::DELETEALL;
        deleteall_marker.header.frame_id = frame_id;
        deleteall_sig.markers.push_back(deleteall_marker );
        publisher.publish(deleteall_sig);
    }
    else if (publisher_type == typeid(geometry_msgs::PolygonStamped).name())
    {
        geometry_msgs::PolygonStamped empty_polygon;
        empty_polygon.header.frame_id = frame_id;
        publisher.publish(empty_polygon);
    }
    else if (publisher_type == typeid(sensor_msgs::PointCloud2).name())
    {
        pcl::PointCloud<pcl::PointXYZ> empty_pc;
        empty_pc.points.clear();
        empty_pc.points.push_back(pcl::PointXYZ(0,0,-100000));
        sensor_msgs::PointCloud2 empty_cloud;
        pcl::toROSMsg(empty_pc, empty_cloud);
        empty_cloud.header.frame_id = frame_id;
        publisher.publish(empty_cloud);
    }
    else {
        PRINT_COLOR_TEXT("[Livz] 话题 \"" << topic_name << "\" 的类型暂不支持clearAll。", YELLOW);
    }
}

void Livz::clearScreen(const std::string& frame_id)
{
    std::map<std::string, PublisherInfo>::iterator it;
    std::map<std::string, PublisherInfo>& publishers = getInstance().getPublishers();
    for(it = publishers.begin(); it != publishers.end(); ++it) {
        Livz::clearAll( it->first, frame_id );
    }
}


void Livz::drawOnePoint( const std::string& topic_name, 
                               const Eigen::Vector3d& position, 
                               double size, 
                               Eigen::Vector4d color, 
                               double opacity, 
                               std::string frame_id, 
                               int id) 
{
    ros::Publisher& publisher = getInstance().getPublisher(topic_name);

    visualization_msgs::Marker point;

    point.header.frame_id    = frame_id;
    point.header.stamp       = ros::Time::now();
    point.ns                 = "points";
    point.action             = visualization_msgs::Marker::ADD;
    point.pose.orientation.w = 1.0;
    point.id                 = id;
    point.type               = visualization_msgs::Marker::POINTS;
    point.scale.x            = size;
    point.scale.y            = size;
    point.scale.z            = size;

    geometry_msgs::Point p;
    p.x = position(0);
    p.y = position(1);
    p.z = position(2);
    point.points.push_back(p);

    // 处理颜色和不透明度
    std_msgs::ColorRGBA c;
    c.r = color(0);
    c.g = color(1);
    c.b = color(2);
    c.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
    c.a = (c.a >= 0.0 && c.a <= 0.1) ? 0 : c.a;
    point.colors.push_back(c);
    publisher.publish(point);
}

void Livz::drawPoints( const std::string& topic_name, 
                       const std::vector<Eigen::Vector3d>& positions, 
                       std::vector<double> sizes, 
                       std::vector<Eigen::Vector4d> colors, 
                       std::vector<double> opacities,
                       std::string frame_id, 
                       int id) 
{
    ros::Publisher& publisher = getInstance().getPublisher(topic_name);
    visualization_msgs::Marker points;

    points.header.frame_id    = frame_id;
    points.header.stamp       = ros::Time::now();
    points.ns                 = "points_array";
    points.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id                 = id;
    points.type               = visualization_msgs::Marker::POINTS;

    // 初始化默认值，使其长度与positions相同
    if (sizes.empty()) {
        sizes = std::vector<double>(positions.size(), 1.0);
    }
    if (colors.empty()) {
        colors = std::vector<Eigen::Vector4d>(positions.size(), V4D(1,1,0,1));
    }
    if (opacities.empty()) {
        opacities = std::vector<double>(positions.size(), -1.0);
    }

    for (size_t i = 0; i < positions.size(); ++i) {
        geometry_msgs::Point p;
        p.x = positions[i](0);
        p.y = positions[i](1);
        p.z = positions[i](2);
        points.points.push_back(p);

        std_msgs::ColorRGBA c;
        c.r = colors[i](0);
        c.g = colors[i](1);
        c.b = colors[i](2);
        c.a = (opacities[i] >= 0.0 && opacities[i] <= 1.0) ? opacities[i] : colors[i](3);
        c.a = (c.a >= 0.0 && c.a <= 0.1) ? 0 : c.a;
        points.colors.push_back(c);

        points.scale.x = sizes[i];
        points.scale.y = sizes[i];
        points.scale.z = sizes[i];
    }
    
    publisher.publish(points);
}


void Livz::drawOneSphere( const std::string& topic_name,
                                const Eigen::Vector3d& position,
                                double radius,
                                Eigen::Vector4d color,
                                double opacity,
                                std::string frame_id,
                                int id)
{
    ros::Publisher& publisher = getInstance().getPublisher(topic_name);

    visualization_msgs::Marker sphere;

    sphere.header.frame_id    = frame_id;
    sphere.header.stamp       = ros::Time::now();
    sphere.ns                 = "spheres";
    sphere.action             = visualization_msgs::Marker::ADD;
    sphere.pose.position.x    = position(0);
    sphere.pose.position.y    = position(1);
    sphere.pose.position.z    = position(2);
    sphere.pose.orientation.w = 1.0;
    sphere.id                 = id;
    sphere.type               = visualization_msgs::Marker::SPHERE;
    sphere.scale.x            = radius;
    sphere.scale.y            = radius;
    sphere.scale.z            = radius;

    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
    sphere.color.a = (sphere.color.a >= 0.0 && sphere.color.a <= 0.1) ? 0 : sphere.color.a;

    publisher.publish(sphere);
}

void Livz::drawOneCylinder(const std::string& topic_name,
                                Eigen::Vector3d position,
                                double radius,
                                double height,
                                Eigen::Vector4d color,
                                double opacity,
                                std::string frame_id,
                                int id)
{
    ros::Publisher& publisher = Livz::getInstance().getPublisher(topic_name);
    visualization_msgs::Marker cylinder;
    cylinder.header.frame_id = frame_id;
    cylinder.header.stamp = ros::Time::now();
    cylinder.ns = "cylinder";
    cylinder.id = id;
    cylinder.type = visualization_msgs::Marker::CYLINDER;
    cylinder.action = visualization_msgs::Marker::ADD;
    cylinder.pose.position.x = position[0];
    cylinder.pose.position.y = position[1];
    cylinder.pose.position.z = position[2];
    cylinder.scale.x = 2 * radius; // 圆柱体的直径
    cylinder.scale.y = 2 * radius; // 圆柱体的直径
    cylinder.scale.z = height; // 圆柱体的高度
    cylinder.color.r = color[0];
    cylinder.color.g = color[1];
    cylinder.color.b = color[2];
    cylinder.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
    cylinder.color.a = (cylinder.color.a >= 0.0 && cylinder.color.a <= 0.1) ? 0 : cylinder.color.a;
    publisher.publish(cylinder);
}

void Livz::drawOneCube(const std::string& topic_name,
                          Eigen::Vector3d position,
                          Eigen::Vector3d scale,
                          Eigen::Vector4d color,
                          double opacity,
                          std::string frame_id,
                          int id)
{
    ros::Publisher& publisher = Livz::getInstance().getPublisher(topic_name);
    visualization_msgs::Marker cube;
    cube.header.frame_id = frame_id;
    cube.header.stamp = ros::Time::now();
    cube.ns = "cube";
    cube.id = id;
    cube.type = visualization_msgs::Marker::CUBE;
    cube.action = visualization_msgs::Marker::ADD;
    cube.pose.position.x = position[0];
    cube.pose.position.y = position[1];
    cube.pose.position.z = position[2];
    cube.scale.x = scale[0]; // 立方体的长度
    cube.scale.y = scale[1]; // 立方体的宽度
    cube.scale.z = scale[2]; // 立方体的高度
    cube.color.r = color[0];
    cube.color.g = color[1];
    cube.color.b = color[2];
    cube.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
    cube.color.a = (cube.color.a >= 0.0 && cube.color.a <= 0.1) ? 0 : cube.color.a;
    publisher.publish(cube);
}

void Livz::drawOneArrow(const std::string& topic_name,
                              Eigen::Vector3d start_point,
                              Eigen::Vector3d end_point,
                              Eigen::Vector3d scale,
                              Eigen::Vector4d color,
                              double opacity,
                              std::string frame_id,
                              int id)
{
    ros::Publisher& publisher = Livz::getInstance().getPublisher(topic_name);
    visualization_msgs::Marker arrow;
    arrow.header.stamp = ros::Time::now();
    arrow.header.frame_id = frame_id;
    arrow.ns = "arrow";
    arrow.id = id;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point start, end;
    start.x = start_point[0];
    start.y = start_point[1];
    start.z = start_point[2];
    end.x = end_point[0];
    end.y = end_point[1];
    end.z = end_point[2];
    arrow.points.push_back(start);
    arrow.points.push_back(end);
    arrow.scale.x = scale[0]; // 箭头的长度
    arrow.scale.y = scale[1]; // 箭头的宽度
    arrow.scale.z = scale[2]; // 箭头的高度
    arrow.color.r = color[0];
    arrow.color.g = color[1];
    arrow.color.b = color[2];
    arrow.pose.orientation.w = 1.0; 
    arrow.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
    arrow.color.a = (arrow.color.a >= 0.0 && arrow.color.a <= 0.1) ? 0 : arrow.color.a;
    publisher.publish(arrow);
}

void Livz::drawArrows(const std::string& topic_name,
                      const std::vector<Eigen::Vector3d>& start_points,
                      const std::vector<Eigen::Vector3d>& end_points,
                      const std::vector<Eigen::Vector3d>& scales,
                      const Eigen::Vector4d& color,
                      double opacity,
                      const std::string& frame_id,
                      int id)
{
    if (start_points.size() != end_points.size() || start_points.size() != scales.size()) {
        PRINT_COLOR_TEXT("[Livz - ERROR] 开始点、结束点和尺度的数量必须相同。话题名为 \"" << topic_name << " \"\n"
                        <<"[Livz - ERROR] Start points, end points and colors must have the same number of elements. Topic name is \"" << topic_name<<" \"", RED);
        return;
    }

    ros::Publisher& publisher = getInstance().getPublisher<visualization_msgs::MarkerArray>(topic_name);
    visualization_msgs::MarkerArray marker_array;
    Eigen::Vector3d start_point, end_point;
    Eigen::Vector3d scale;
    for (size_t i = 0; i < start_points.size(); i ++ ) {
        
        start_point = start_points[i];
        end_point   = end_points[i];
        scale       = scales[i];

        visualization_msgs::Marker arrow;
        arrow.header.stamp = ros::Time::now();
        arrow.header.frame_id = frame_id;
        arrow.ns = "arrow";
        arrow.id = id + i;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point start, end;
        start.x = start_point[0];
        start.y = start_point[1];
        start.z = start_point[2];
        end.x = end_point[0];
        end.y = end_point[1];
        end.z = end_point[2];
        arrow.points.push_back(start);
        arrow.points.push_back(end);
        arrow.scale.x = scale[0]; // 箭头的长度
        arrow.scale.y = scale[1]; // 箭头的宽度
        arrow.scale.z = scale[2]; // 箭头的高度
        arrow.color.r = color[0];
        arrow.color.g = color[1];
        arrow.color.b = color[2];
        arrow.pose.orientation.w = 1.0; 
        arrow.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
        arrow.color.a = (arrow.color.a >= 0.0 && arrow.color.a <= 0.1) ? 0 : arrow.color.a;

        marker_array.markers.push_back(arrow);
    }
    publisher.publish(marker_array);
}

void Livz::drawArrowsWithColors(const std::string& topic_name,
                                const std::vector<Eigen::Vector3d>& start_points,
                                const std::vector<Eigen::Vector3d>& end_points,
                                const std::vector<Eigen::Vector3d>& scales,
                                const std::vector<Eigen::Vector4d>& colors,
                                double opacity,
                                const std::string& frame_id,
                                int id)
{
    if (start_points.size() != end_points.size() || start_points.size() != colors.size() || start_points.size() != scales.size()) {
        PRINT_COLOR_TEXT("[Livz - ERROR] 开始点、结束点、颜色和尺度的数量必须相同。话题名为 \"" << topic_name << " \"\n"
                        <<"[Livz - ERROR] Start points, end points, scales, and colors must have the same number of elements. Topic name is \"" << topic_name<<" \"", RED);
        return;
    }

    ros::Publisher& publisher = getInstance().getPublisher<visualization_msgs::MarkerArray>(topic_name);
    visualization_msgs::MarkerArray marker_array;
    Eigen::Vector3d start_point, end_point;
    Eigen::Vector4d color;
    Eigen::Vector3d scale;
    for (size_t i = 0; i < start_points.size(); i ++ ) {
        
        start_point = start_points[i];
        end_point   = end_points[i];
        color       = colors[i];
        scale       = scales[i];

        visualization_msgs::Marker arrow;
        arrow.header.stamp = ros::Time::now();
        arrow.header.frame_id = frame_id;
        arrow.ns = "arrow";
        arrow.id = id + i;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point start, end;
        start.x = start_point[0];
        start.y = start_point[1];
        start.z = start_point[2];
        end.x = end_point[0];
        end.y = end_point[1];
        end.z = end_point[2];
        arrow.points.push_back(start);
        arrow.points.push_back(end);
        arrow.scale.x = scale[0]; // 箭头的长度
        arrow.scale.y = scale[1]; // 箭头的宽度
        arrow.scale.z = scale[2]; // 箭头的高度
        arrow.color.r = color[0];
        arrow.color.g = color[1];
        arrow.color.b = color[2];
        arrow.pose.orientation.w = 1.0; 
        arrow.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
        arrow.color.a = (arrow.color.a >= 0.0 && arrow.color.a <= 0.1) ? 0 : arrow.color.a;

        marker_array.markers.push_back(arrow);
    }
    publisher.publish(marker_array);
}


void Livz::drawLineStrip(const std::string& topic_name,
                               const std::vector<Eigen::Vector3d>& points,
                               double width,
                               Eigen::Vector4d color,
                               double opacity,
                               std::string frame_id,
                               int id)
{
    ros::Publisher& publisher = getInstance().getPublisher(topic_name);

    visualization_msgs::Marker line_strip;

    line_strip.header.frame_id    = frame_id;
    line_strip.header.stamp       = ros::Time::now();
    line_strip.ns                 = "line_strip";
    line_strip.action             = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id                 = id;
    line_strip.type               = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x            = width;

    line_strip.color.r = color(0);
    line_strip.color.g = color(1);
    line_strip.color.b = color(2);
    line_strip.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
    line_strip.color.a = (line_strip.color.a >= 0.0 && line_strip.color.a <= 0.1) ? 0 : line_strip.color.a;

    for (const Eigen::Vector3d& p : points) {
        geometry_msgs::Point point;
        point.x = p(0);
        point.y = p(1);
        point.z = p(2);
        line_strip.points.push_back(point);
    }

    publisher.publish(line_strip);
}

void Livz::drawLineList( const std::string& topic_name,
                                 const std::vector<Eigen::Vector3d>& points,
                                 double width,
                                 Eigen::Vector4d color,
                                 double opacity,
                                 std::string frame_id ,
                                 int id)
{
    ros::Publisher& publisher = getInstance().getPublisher(topic_name);

    visualization_msgs::Marker line_list;

    line_list.header.frame_id    = frame_id;
    line_list.header.stamp       = ros::Time::now();
    line_list.ns                 = "line_list";
    line_list.action             = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id                 = id;
    line_list.type               = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x            = width;

    line_list.color.r = color(0);
    line_list.color.g = color(1);
    line_list.color.b = color(2);
    line_list.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
    line_list.color.a = (line_list.color.a >= 0.0 && line_list.color.a <= 0.1) ? 0 : line_list.color.a;

    for (const Eigen::Vector3d& p : points) {
        geometry_msgs::Point point;
        point.x = p(0);
        point.y = p(1);
        point.z = p(2);
        line_list.points.push_back(point);
    }

    publisher.publish(line_list);
}

void Livz::drawLineListWithColors(const std::string& topic_name,
                                  const std::vector<Eigen::Vector3d>& points,
                                  const std::vector<Eigen::Vector4d>& colors,
                                  double width,
                                  double opacity,
                                  const std::string& frame_id,
                                  int id)
{
    if (points.size() % 2 != 0) {
        PRINT_COLOR_TEXT("[Livz - ERROR] 点的数量必须为偶数。话题名为 " << topic_name, RED);
        return;
    }

    ros::Publisher& publisher = getInstance().getPublisher(topic_name);

    visualization_msgs::Marker line_list;

    line_list.header.frame_id    = frame_id;
    line_list.header.stamp       = ros::Time::now();
    line_list.ns                 = "line_list";
    line_list.action             = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id                 = id;
    line_list.type               = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x            = width;

    std_msgs::ColorRGBA line_color;
    line_list.color.r = colors[0](0);
    line_list.color.g = colors[0](1);
    line_list.color.b = colors[0](2);
    line_list.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : colors[0](3);
    line_list.color.a = (line_list.color.a >= 0.0 && line_list.color.a <= 0.1) ? 0 : line_list.color.a;

    for (size_t i = 0; i < points.size(); i += 2) {
        if( points[i].hasNaN() == true || points[i].allFinite() == false || points[i + 1].hasNaN() == true || points[i + 1].allFinite() == false ) {
            continue;
        }
        geometry_msgs::Point start_point, end_point;
        start_point.x = points[i](0);
        start_point.y = points[i](1);
        start_point.z = points[i](2);
        end_point.x = points[i + 1](0);
        end_point.y = points[i + 1](1);
        end_point.z = points[i + 1](2);
        line_list.points.push_back(start_point);
        line_list.points.push_back(end_point);

        line_color.r = colors[i / 2](0);
        line_color.g = colors[i / 2](1);
        line_color.b = colors[i / 2](2);
        line_color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : colors[i/2](3);
        line_color.a = (line_color.a >= 0.0 && line_color.a <= 0.1) ? 0 : line_color.a;
        line_list.colors.push_back(line_color);
        line_list.colors.push_back(line_color);
    }

    publisher.publish(line_list);
}



void Livz::draw2DPolygon_MARKER(const std::string& topic_name,
                                    const std::vector<Eigen::Vector3d>& vertices,
                                    Eigen::Vector4d color,
                                    double stroke_width,
                                    double opacity,
                                    std::string frame_id,
                                    int id)
{
    ros::Publisher& publisher = Livz::getInstance().getPublisher(topic_name);

    visualization_msgs::Marker polygon;

    polygon.header.frame_id    = frame_id;
    polygon.header.stamp       = ros::Time::now();
    polygon.ns                 = "polygons";
    polygon.action             = visualization_msgs::Marker::ADD;
    polygon.pose.orientation.w = 1.0;
    polygon.id                 = id;
    polygon.type               = visualization_msgs::Marker::LINE_STRIP;
    polygon.scale.x            = stroke_width;

    polygon.color.r = color(0);
    polygon.color.g = color(1);
    polygon.color.b = color(2);
    polygon.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
    polygon.color.a = (polygon.color.a >= 0.0 && polygon.color.a <= 0.1) ? 0 : polygon.color.a;

    for (const Eigen::Vector3d& vertex : vertices) {
        geometry_msgs::Point point;
        point.x = vertex(0);
        point.y = vertex(1);
        point.z = vertex(2); // 如果是2D多边形，此值通常为0
        polygon.points.push_back(point);
    }

    // 如果你想要闭合的多边形，可以再次添加第一个点
    if (!vertices.empty()) {
        geometry_msgs::Point first_point;
        first_point.x = vertices.front()(0);
        first_point.y = vertices.front()(1);
        first_point.z = vertices.front()(2);
        polygon.points.push_back(first_point);
    }

    publisher.publish(polygon);
}

void Livz::draw2DPolygon_POLYGON(const std::string& topic_name,
                               const std::vector<Eigen::Vector3d>& vertices,
                               Eigen::Vector4d color, // color不再使用
                               double stroke_width,   // stroke_width不再使用
                               double opacity,        // opacity不再使用
                               std::string frame_id,
                               int id)                // id不再使用
{
    ros::Publisher& publisher = Livz::getInstance().getPublisher<geometry_msgs::PolygonStamped>(topic_name);

    geometry_msgs::PolygonStamped polygon_stamped;

    polygon_stamped.header.frame_id = frame_id;
    polygon_stamped.header.stamp = ros::Time::now();

    for (const Eigen::Vector3d& vertex : vertices) {
        geometry_msgs::Point32 point;
        point.x = vertex(0);
        point.y = vertex(1);
        point.z = vertex(2); // 如果是2D多边形，此值通常为0
        polygon_stamped.polygon.points.push_back(point);
    }

    publisher.publish(polygon_stamped);

}

void Livz::draw2DRect(const std::string& topic_name,
                       const Eigen::Vector3d& top_left,
                       double width, double height, 
                       double rot_z,
                       Eigen::Vector4d color,
                        double stroke_width,
                       double opacity,
                       std::string frame_id,
                       int id) 
{
    const double z = top_left(2);
    std::vector<Eigen::Vector3d> vertices = {
        top_left,
        {top_left(0) + width, top_left(1), z},
        {top_left(0) + width, top_left(1) - height, z},
        {top_left(0), top_left(1) - height, z}
    };

    Eigen::Vector3d rot_center = top_left + Eigen::Vector3d( width/2 ,height/2 , 0);
    rotatePointsSO1(vertices, rot_center, rot_z);
    draw2DPolygon(topic_name, vertices, color, stroke_width, opacity, frame_id, id);
}

void Livz::draw2DCircle(const std::string& topic_name,
                         const Eigen::Vector3d& center,
                         double radius,
                         Eigen::Vector4d color,
                         double stroke_width,
                         double opacity,
                         std::string frame_id,
                         int id,
                         int samples) 
{
    const double z = center(2);
    std::vector<Eigen::Vector3d> vertices;
    for(int i = 0; i < samples; i++) {
        double theta = 2.0 * M_PI * i / samples;
        double x = center(0) + radius * cos(theta);
        double y = center(1) + radius * sin(theta);
        vertices.push_back({x, y, z});
    }
    draw2DPolygon(topic_name, vertices, color, stroke_width, opacity, frame_id, id);
}

void Livz::draw2DEllipse(const std::string& topic_name,
                          const Eigen::Vector3d& center,
                          double major_axis, double minor_axis,
                          double rot_z,
                          Eigen::Vector4d color,
                          double stroke_width,
                          double opacity,
                          std::string frame_id,
                          int id,
                          int samples) 
{
    const double z = center(2);
    std::vector<Eigen::Vector3d> vertices;
    for(int i = 0; i < samples; i++) {
        double theta = 2.0 * M_PI * i / samples;
        double x = center(0) + major_axis * cos(theta);
        double y = center(1) + minor_axis * sin(theta);
        vertices.push_back({x, y, z});
    }
    rotatePointsSO1(vertices,center, rot_z);
    draw2DPolygon(topic_name, vertices, color, stroke_width, opacity, frame_id, id);
}


void Livz::draw2DCone(const std::string& topic_name,
                          const Eigen::Vector3d& center,
                          double radius,
                          double fov,
                          double rot_z,
                          Eigen::Vector4d color,
                          double stroke_width ,
                          double opacity,
                          std::string frame_id,
                          int id,
                          int samples)
{
    const double z = center(2);
    const double fov_2 = fov/2;
    std::vector<Eigen::Vector3d> vertices;
    vertices.push_back(center);
    for(int i = 0; i < samples; i++) {
        double theta = fov * i / samples - fov_2;
        double x = center(0) + radius * cos(theta);
        double y = center(1) + radius * sin(theta);
        vertices.push_back({x, y, z});
    }
    vertices.push_back(center);
    rotatePointsSO1(vertices, center, rot_z);
    draw2DPolygon(topic_name, vertices, color, stroke_width, opacity, frame_id, id);
}

void Livz::drawPointcloud( const std::string& topic_name,
                                 pcl::PointCloud<pcl::PointXYZ>& cloud,
                                 Eigen::Vector4d color,
                                 std::string frame_id )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    uint8_t r = static_cast<uint8_t>(color(0) * 255.0);
    uint8_t g = static_cast<uint8_t>(color(1) * 255.0);
    uint8_t b = static_cast<uint8_t>(color(2) * 255.0);

    for (const auto& point : cloud.points) {
        pcl::PointXYZRGB p;
        p.x = point.x; p.y = point.y; p.z = point.z;
        p.r = r; p.g = g; p.b = b;
        coloredCloud->points.push_back(p);
    }

    coloredCloud->width    = cloud.width;
    coloredCloud->height   = cloud.height;
    coloredCloud->is_dense = cloud.is_dense;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*coloredCloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;

    ros::Publisher& publisher = Livz::getInstance().getPublisher<sensor_msgs::PointCloud2>(topic_name);
    publisher.publish(cloud_msg);

    if( cloud.empty() ){
        PRINT_COLOR_TEXT( "[Livz - WARN] 发送了空点云。话题名为 " << topic_name <<"\n "
                          << "[Livz - WARN] Published an empty pointcloud. Topic name is "<< topic_name , YELLOW );
    }
}


void Livz::drawPointcloudRGB(const std::string& topic_name,
                                pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                                std::string frame_id)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;

    ros::Publisher& publisher = Livz::getInstance().getPublisher<sensor_msgs::PointCloud2>(topic_name);
    publisher.publish(cloud_msg);

    if( cloud.empty() ){
        PRINT_COLOR_TEXT(  "[Livz - WARN] 发送了空点云。话题名为 "<< topic_name <<" \n"
                         <<"[Livz - WARN] Published an empty pointcloud. Topic name is "<< topic_name , YELLOW );
    }
}

void Livz::drawPointcloudI(const std::string& topic_name,
                                pcl::PointCloud<pcl::PointXYZI>& cloud,
                                std::string frame_id)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;

    ros::Publisher& publisher = Livz::getInstance().getPublisher<sensor_msgs::PointCloud2>(topic_name);
    publisher.publish(cloud_msg);

    if( cloud.empty() ){
        PRINT_COLOR_TEXT("[Livz - WARN] 发送了空点云。话题名为 " << topic_name << " \n"
                 << "[Livz - WARN] Published an empty pointcloud. Topic name is " << topic_name, YELLOW);
    }
}

void Livz::drawPointcloudWithShader(const std::string& topic_name,
                                          pcl::PointCloud<pcl::PointXYZ>& cloud,
                                          ShaderFunction shader,
                                          std::string frame_id)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    uint8_t r;
    uint8_t g;
    uint8_t b;

    for (const auto& point : cloud.points) {
        Eigen::Vector3d pos(point.x, point.y, point.z);
        Eigen::Vector3d color = shader(pos);
        r = static_cast<uint8_t>(color(0) * 255.0);
        g = static_cast<uint8_t>(color(1) * 255.0);
        b = static_cast<uint8_t>(color(2) * 255.0);
        pcl::PointXYZRGB p;
        p.x = point.x; p.y = point.y; p.z = point.z;
        p.r = r; p.g = g; p.b = b;
        coloredCloud->points.push_back(p);
    }

    coloredCloud->width    = cloud.width;
    coloredCloud->height   = cloud.height;
    coloredCloud->is_dense = cloud.is_dense;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*coloredCloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;

    ros::Publisher& publisher = Livz::getInstance().getPublisher<sensor_msgs::PointCloud2>(topic_name);
    publisher.publish(cloud_msg);

    if( cloud.empty() ){

        PRINT_COLOR_TEXT("[Livz - WARN] 发送了空点云。话题名为 " << topic_name << " \n"
                 << "[Livz - WARN] Published an empty pointcloud. Topic name is " << topic_name, YELLOW);
    }
}

void Livz::drawParametricCurve(const std::string& topic_name,
                                ParametricCurveFunction curve_function,
                                Eigen::Vector2d interval,
                                Eigen::Vector4d color,
                                double stroke_width,
                                double opacity,
                                std::string frame_id,
                                int id,
                                double sample_gap)
{
    ros::Publisher& publisher = getInstance().getPublisher(topic_name);

    visualization_msgs::Marker parametric_curve;

    parametric_curve.header.frame_id    = frame_id;
    parametric_curve.header.stamp       = ros::Time::now();
    parametric_curve.ns                 = "parametric_curve";
    parametric_curve.action             = visualization_msgs::Marker::ADD;
    parametric_curve.pose.orientation.w = 1.0;
    parametric_curve.id                 = id;
    parametric_curve.type               = visualization_msgs::Marker::LINE_STRIP;
    parametric_curve.scale.x            = stroke_width;

    parametric_curve.color.r = color(0);
    parametric_curve.color.g = color(1);
    parametric_curve.color.b = color(2);
    parametric_curve.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
    parametric_curve.color.a = (parametric_curve.color.a >= 0.0 && parametric_curve.color.a <= 0.1) ? 0 : parametric_curve.color.a;

    // 采样参数化曲线，构造线条
    double lower_bound = interval(0);
    double upper_bound = interval(1);

    if (upper_bound < lower_bound){
        PRINT_COLOR_TEXT("[Livz - WARN] 参数曲线绘制的区间为空集，检查interval参数。\n "
                         <<"[Livz - WARN] Parameter curve has an empty interval for drawing, check the interval parameter. ", YELLOW);
    }

    auto velocity_function = [&curve_function, lower_bound, upper_bound](double t, double delta) -> Eigen::Vector3d {
        if (t - delta < lower_bound) {
            // 左边界，使用前向差分
            Eigen::Vector3d forward = curve_function(t + delta);
            Eigen::Vector3d current = curve_function(t);
            return (forward - current) / delta;
        } else if (t + delta > upper_bound) {
            // 右边界，使用后向差分
            Eigen::Vector3d current = curve_function(t);
            Eigen::Vector3d backward = curve_function(t - delta);
            return (current - backward) / delta;
        } else {
            // 中间区域，使用中心差分
            Eigen::Vector3d forward = curve_function(t + delta);
            Eigen::Vector3d backward = curve_function(t - delta);
            return (forward - backward) / (2.0 * delta);
        }
    };

    double t = lower_bound;
    while (t <= upper_bound) {
        Eigen::Vector3d p = curve_function(t);
        Eigen::Vector3d v = velocity_function(t, 1e-5); 
        geometry_msgs::Point point;
        point.x = p(0);
        point.y = p(1);
        point.z = p(2);
        parametric_curve.points.push_back(point);

        double v_norm = v.norm();
        t += (v_norm < 1e-1) ? 0.01 : (sample_gap / v_norm);

        // std::cout<<"v_norm = "<<v_norm<<std::endl;
    }

    // 发布消息
    publisher.publish(parametric_curve);
}

void Livz::drawViewText(   const std::string& topic_name,
                                 const std::string& text,
                                 Eigen::Vector3d position,
                                 double size,
                                 Eigen::Vector4d color,
                                 double opacity,
                                 std::string frame_id ,
                                 int id)
{
    ros::Publisher& publisher = Livz::getInstance().getPublisher(topic_name);
    // 创建Marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "view_text";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];
    marker.pose.orientation.w = 1.0;

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
    marker.color.a = (marker.color.a >= 0.0 && marker.color.a <= 0.1) ? 0 : marker.color.a;
    // 设置尺寸
    marker.scale.z = size; // 文字的高度
    // 设置文本
    marker.text = text;
    // 发布Marker
    publisher.publish(marker);
}

void Livz::renderTrianglesWithColors(const std::string& topic_name, 
                                 Eigen::Matrix3Xd &mesh_vertexes, 
                                 const std::vector<Eigen::Vector4d>& colors, 
                                 double opacity,
                                 std::string frame_id,
                                 int id)
{
    int ptnum = mesh_vertexes.cols(); 
    if (colors.size() < floor(ptnum/3.0)) {
        PRINT_COLOR_TEXT("[Livz - ERROR] 颜色列表的数目小于三角面数。话题名为 " << topic_name, RED);
        return;
    }
    ros::Publisher& publisher = Livz::getInstance().getPublisher(topic_name);
    visualization_msgs::Marker triangles;
    triangles.header.stamp    = ros::Time::now();
    triangles.header.frame_id = frame_id;
    triangles.pose.orientation.w = 1.00;
    triangles.type = visualization_msgs::Marker::TRIANGLE_LIST;
    triangles.ns   = "triangles";

    triangles.scale.x = 1.00;
    triangles.scale.y = 1.00;
    triangles.scale.z = 1.00;

    triangles.pose.orientation.w = 1.0; 

    std_msgs::ColorRGBA triangle_color;
    triangles.color.r = colors[0](0);
    triangles.color.g = colors[0](1);
    triangles.color.b = colors[0](2);
    triangles.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : colors[0](3);
    triangles.color.a = (triangles.color.a >= 0.0 && triangles.color.a <= 0.1) ? 0 : triangles.color.a;

    geometry_msgs::Point point;
    for (size_t i = 0; i < ptnum; i++ ) {

        // if( mesh_vertexes.col(i).hasNaN() == true || mesh_vertexes.col(i).allFinite() == false ) {
        //     i += (2 - i%3) - 1;
        //     continue;
        // }
        point.x = mesh_vertexes(0, i);
        point.y = mesh_vertexes(1, i);
        point.z = mesh_vertexes(2, i);
        triangles.points.push_back(point);

        triangle_color.r = colors[floor(i / 3.0)](0);
        triangle_color.g = colors[floor(i / 3.0)](1);
        triangle_color.b = colors[floor(i / 3.0)](2);
        triangle_color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : colors[floor(i / 3.0)](3);
        triangle_color.a = (triangle_color.a >= 0.0 && triangle_color.a <= 0.1) ? 0 : triangle_color.a;
        triangles.colors.push_back(triangle_color);
    }
    publisher.publish(triangles);
}

void Livz::renderTriangleMesh(const std::string& topic_name, 
                                 Eigen::Matrix3Xd &mesh_vertexes, 
                                 Eigen::Vector4d color, 
                                 double opacity,
                                 std::string frame_id,
                                 int id)
{
    ros::Publisher& publisher = Livz::getInstance().getPublisher(topic_name);

    visualization_msgs::Marker mesh;
    mesh.header.stamp    = ros::Time::now();
    mesh.header.frame_id = frame_id;
    mesh.pose.orientation.w = 1.00;
    mesh.type = visualization_msgs::Marker::TRIANGLE_LIST;
    mesh.ns   = "mesh";

    mesh.scale.x = 1.00;
    mesh.scale.y = 1.00;
    mesh.scale.z = 1.00;

    mesh.pose.orientation.w = 1.0; 
    mesh.color.r = color[0];
    mesh.color.g = color[1];
    mesh.color.b = color[2];
    mesh.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
    mesh.color.a = (mesh.color.a >= 0.0 && mesh.color.a <= 0.1) ? 0 : mesh.color.a;

    geometry_msgs::Point point;
    int ptnum = mesh_vertexes.cols(); 
    for (int i = 0; i < ptnum; i++)
    {
        point.x = mesh_vertexes(0, i);
        point.y = mesh_vertexes(1, i);
        point.z = mesh_vertexes(2, i);
        mesh.points.push_back(point);
    }
    publisher.publish(mesh);
}
//唯一需要注意U，G matrix的格式
void Livz::renderTriangleMeshWithEdges( const std::string& meshtopic, 
                                        const std::string& edgetopic, 
                                            Eigen::MatrixXd &U, 
                                            Eigen::MatrixXi &G, 
                                            Eigen::Vector4d color, 
                                            double opacity,
                                            std::string frame_id,
                                            int id)
{
    ros::Publisher& publishermesh = Livz::getInstance().getPublisher(meshtopic);
    ros::Publisher& publisheredge = Livz::getInstance().getPublisher(edgetopic);

    visualization_msgs::Marker meshMarker, edgeMarker;

    meshMarker.header.stamp = ros::Time::now();
    meshMarker.header.frame_id = frame_id;
    meshMarker.pose.orientation.w = 1.00;
    meshMarker.action = visualization_msgs::Marker::ADD;
    meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    meshMarker.ns = "mesh";
    meshMarker.color.r = color[0];
    meshMarker.color.g = color[1];
    meshMarker.color.b = color[2];
    meshMarker.color.a = (opacity >= 0.0 && opacity <= 1.0) ? opacity : color(3);
    meshMarker.color.a = (meshMarker.color.a >= 0.0 && meshMarker.color.a <= 0.1) ? 0 : meshMarker.color.a;

    meshMarker.scale.x = 1.00;
    meshMarker.scale.y = 1.00;
    meshMarker.scale.z = 1.00;

    edgeMarker = meshMarker;
    edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = 0.00;
    edgeMarker.color.g = 1.00;
    edgeMarker.color.b = 0.00;
    edgeMarker.color.a = 1.00;
    edgeMarker.scale.x = 0.005 * 0.1;
    edgeMarker.scale.y = 0.005 * 0.1;
    edgeMarker.scale.z = 0.005 * 0.1;

    geometry_msgs::Point point;

    int faces = G.rows();
    for (int i = 0; i < faces; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            point.x = U.row(G.row(i)[j])[0];
            point.y = U.row(G.row(i)[j])[1];
            point.z = U.row(G.row(i)[j])[2];
            meshMarker.points.push_back(point);

            edgeMarker.points.push_back(point);
            point.x = U.row(G.row(i)[(j + 1) % 3])[0];
            point.y = U.row(G.row(i)[(j + 1) % 3])[1];
            point.z = U.row(G.row(i)[(j + 1) % 3])[2];
            edgeMarker.points.push_back(point);
        }
    }
    publishermesh.publish(edgeMarker);
    publisheredge.publish(meshMarker);
}

void Livz::renderTriangleMeshEdges( const std::string& topic_name, 
                                        Eigen::Matrix3Xd &mesh_vertexes, 
                                        Eigen::Vector4d color,
                                        double stroke_width,
                                        double opacity,
                                        std::string frame_id,
                                        int id)
{
    std::vector<Eigen::Vector3d> edge_vertexes;
    int ptnum = mesh_vertexes.cols(); // ptnum/3
    Eigen::Vector3d point;
    for (int i = 0; i < ptnum / 3; i++) // mesh.cols() 12
    {
        for (int j = 0; j < 3; j++)
        {
            point(0) = mesh_vertexes(0, 3 * i + j);
            point(1) = mesh_vertexes(1, 3 * i + j);
            point(2) = mesh_vertexes(2, 3 * i + j);
            edge_vertexes.push_back(point);
            point(0) = mesh_vertexes(0, 3 * i + (j + 1) % 3);
            point(1) = mesh_vertexes(1, 3 * i + (j + 1) % 3);
            point(2) = mesh_vertexes(2, 3 * i + (j + 1) % 3);
            edge_vertexes.push_back(point);
        }
    }
    Livz::drawLineList(topic_name, edge_vertexes, stroke_width, color, opacity, frame_id, id);
}

void Livz::renderSurface(const std::string& topic_name, 
                               ParametricSurfaceFunction surf_function, 
                               Eigen::Vector2d u_range,
                               Eigen::Vector2d v_range,
                               double u_res,
                               double v_res,
                               Eigen::Vector4d color, 
                               double opacity,
                               std::string frame_id,
                               int id)
{

    int u_spcount = floor(u_range[1] - u_range[0]) / u_res;
    int v_spcount = floor(v_range[1] - v_range[0]) / v_res;
    
    u_res = (u_range[1] - u_range[0]) / u_spcount;
    v_res = (v_range[1] - v_range[0]) / v_spcount;
    Eigen::Matrix3Xd mesh_vertexes(3, u_spcount * v_spcount * 6); // 每个方块2个三角形，每个三角形3个顶点

    double u_start = u_range[0];
    double u_end   = u_range(1);
    double v_start = v_range[0];
    double v_end   = v_range(1);
    int index = 0;

    for (int i = 0; i < u_spcount; ++i) {
        for (int j = 0; j < v_spcount; ++j) {
            double u = u_start + i * u_res;
            double v = v_start + j * v_res;
            double z1 = surf_function(u, v);
            double z2 = surf_function(u + u_res, v);
            double z3 = surf_function(u, v + v_res);
            double z4 = surf_function(u + u_res, v + v_res);

            // 第一个三角形
            mesh_vertexes.col(index++) << u, v, z1;
            mesh_vertexes.col(index++) << u + u_res, v, z2;
            mesh_vertexes.col(index++) << u, v + v_res, z3;

            // 第二个三角形
            mesh_vertexes.col(index++) << u + u_res, v, z2;
            mesh_vertexes.col(index++) << u + u_res, v + v_res, z4;
            mesh_vertexes.col(index++) << u, v + v_res, z3;
        }
    }
    Livz::renderTriangleMesh(topic_name, mesh_vertexes, color, opacity, frame_id, id);
}

void Livz::renderVectorField(  const std::string& topic_name, 
                                    VectorField field_function,
                                    Eigen::Vector3d box_min ,
                                    Eigen::Vector3d box_max ,
                                    Eigen::Vector3d resolution,
                                    const bool only_2d,
                                    Eigen::Vector4d color_min, 
                                    Eigen::Vector4d color_max, 
                                    double opacity ,
                                    std::string frame_id,
                                    int id )
{
    // 检查box_min和box_max定义的范围是否体积为0
    if (box_min[0] >= box_max[0] || box_min[1] >= box_max[1] || box_min[2] >= box_max[2]) {
        PRINT_COLOR_TEXT("[Livz - WARN] 向量场绘制的区间为空集，检查box参数。\n "
                       <<"[Livz - WARN] Vector Field has an empty interval for drawing, check the box parameter. ", YELLOW);
        return;
    }

    // 根据resolution生成采样点并放入vector
    std::vector<Eigen::Vector3d> sample_points;
    std::vector<Eigen::Vector3d> vector_results;
    std::vector<Eigen::Vector3d> sample_ends;
    std::vector<Eigen::Vector3d> arrow_sizes;
    std::vector<Eigen::Vector4d> arrow_colors;

    double min_vector_norm =  1e8;
    double max_vector_norm = -1e8;

    Eigen::Vector3d sample_point;
    Eigen::Vector3d vector_result;
    resolution[0] = (box_max[0] - box_min[0]) / floor ( (box_max[0] - box_min[0]) / resolution[0] );
    resolution[1] = (box_max[1] - box_min[1]) / floor ( (box_max[1] - box_min[1]) / resolution[1] );
    resolution[2] = (box_max[2] - box_min[2]) / floor ( (box_max[2] - box_min[2]) / resolution[2] );

    double vector_norm = 0.0;
    for (double x = box_min[0]; x <= box_max[0]; x += resolution[0]) {
        for (double y = box_min[1]; y <= box_max[1]; y += resolution[1]) {
            for (double z = box_min[2]; only_2d ? z <= box_min[2] : z <= box_max[2]; z += resolution[2]) {
                sample_point = Eigen::Vector3d(x,y,z);
                sample_points.emplace_back(sample_point);
                vector_result = field_function(sample_point);
                if(only_2d){vector_result[2] = 0;}
                vector_results.emplace_back(vector_result);

                vector_norm = vector_result.norm();
                if(vector_norm > max_vector_norm){ max_vector_norm = vector_norm; }
                if(vector_norm < min_vector_norm){ min_vector_norm = vector_norm; }
            }
        }
    }

    double k = 0.4 * resolution.minCoeff() / (max_vector_norm - min_vector_norm + 0.01);
    double b = 0.5 * resolution.minCoeff() - k * min_vector_norm;
    double arrow_len, arrow_width, arrow_height, vector_len;
    Eigen::Vector4d arrow_color;
    double st;
    for(size_t i = 0; i < sample_points.size(); i++)
    {
        Eigen::Vector3d res = vector_results[i];
        vector_norm = res.norm();
        st = (vector_norm - min_vector_norm) / (max_vector_norm - min_vector_norm + 0.01);
        vector_len   = k * vector_norm + b;
        //  vector_len   = 0.8 * resolution.minCoeff();
        arrow_len    = 0.1 * vector_len;
        arrow_width  = 3 * arrow_len;
        arrow_height = arrow_width;
        sample_ends.emplace_back(sample_points[i] + vector_len*res.normalized());
        arrow_sizes.emplace_back( Eigen::Vector3d(arrow_len, arrow_width, arrow_height) );
        INTERPOLATION::interpolation( color_min, color_max, arrow_color, st );
        arrow_colors.emplace_back(arrow_color );
    }
    Livz::drawArrowsWithColors(topic_name, sample_points, sample_ends ,arrow_sizes, arrow_colors,opacity, frame_id, id );
}

void Livz::renderStreamLines(  const std::string& topic_name, 
                                    VectorField field_function,
                                    Eigen::Vector3d box_min ,
                                    Eigen::Vector3d box_max ,
                                    Eigen::Vector3d resolution,
                                    int start_step ,
                                    int end_step,
                                    const bool only_2d,
                                    Eigen::Vector4d color_min, 
                                    Eigen::Vector4d color_max, 
                                    double opacity ,
                                    std::string frame_id,
                                    int id )
{
    // 检查box_min和box_max定义的范围是否体积为0
    if (box_min[0] >= box_max[0] || box_min[1] >= box_max[1] || box_min[2] >= box_max[2]) {
        PRINT_COLOR_TEXT("[Livz - WARN] 向量场绘制的区间为空集，检查box参数。\n "
                       <<"[Livz - WARN] Vector Field has an empty interval for drawing, check the box parameter. ", YELLOW);
        return;
    }

    // 根据resolution生成采样点并放入vector
    std::vector<Eigen::Vector3d> sample_points;
    std::vector<Eigen::Vector3d> render_points;
    std::vector<Eigen::Vector4d> render_colors;

    Eigen::Matrix3Xd render_arrows;
    std::vector<Eigen::Vector4d> arrow_colors;

    double min_vector_norm =  1e8;
    double max_vector_norm = -1e8;

    Eigen::Vector3d sample_point;
    Eigen::Vector3d next_point;
    Eigen::Vector3d vector_result;
    resolution[0] = (box_max[0] - box_min[0]) / floor ( (box_max[0] - box_min[0]) / resolution[0] );
    resolution[1] = (box_max[1] - box_min[1]) / floor ( (box_max[1] - box_min[1]) / resolution[1] );
    resolution[2] = (box_max[2] - box_min[2]) / floor ( (box_max[2] - box_min[2]) / resolution[2] );
    if(only_2d){resolution[2] = 1e9;}

    double vector_norm = 0.0;
    for (double x = box_min[0]; x <= box_max[0]; x += resolution[0]) {
        for (double y = box_min[1]; y <= box_max[1]; y += resolution[1]) {
            for (double z = box_min[2]; only_2d ? (z <= box_min[2]) : (z <= box_max[2]); z += resolution[2]) {
                sample_point = Eigen::Vector3d(x,y,z);
                sample_points.emplace_back(sample_point);
                vector_result = field_function(sample_point);
                if(only_2d){vector_result[2] = 0;}
                vector_norm = vector_result.norm();
                if(vector_norm > max_vector_norm){ max_vector_norm = vector_norm; }
                if(vector_norm < min_vector_norm){ min_vector_norm = vector_norm; }
            }
        }
    }

    Eigen::Vector4d render_color;
    double st;
    double min_res = resolution.minCoeff();

    render_arrows.resize(3, 3*sample_points.size());
    for(size_t i = 0; i < sample_points.size(); i++)
    {
        sample_point = sample_points[i];
        for(int exp_times = 0; exp_times <= end_step; exp_times ++){
            vector_result = field_function(sample_point);
            if(only_2d){vector_result[2] = 0;}
            vector_norm = vector_result.norm();
            st = (vector_norm - min_vector_norm) / (max_vector_norm - min_vector_norm + 0.01);

            next_point = sample_point + 0.1*vector_result/vector_norm;

            if ( exp_times >= start_step ){
                render_points.emplace_back(sample_point);
                render_points.emplace_back(next_point);

                INTERPOLATION::interpolation( color_min, color_max, render_color, st );
                render_colors.emplace_back(render_color);
            }

            if(exp_times == ceil( (end_step + start_step) / 2 ))
            {
                Eigen::Vector3d p1,p2,p3;
                Eigen::Vector3d d,s,r;
                d  = next_point - sample_point;

                if( d(2) == 0 ){ r[0] = -d(1); r[1] = d(0); r[2] = 0;}
                else if( d(0) == 0 ){ r[0] = 0.0; r[1] = -d(2); r[2] = d(1);}
                else if( d(1) == 0 ){ r[0] = -d(2); r[1] = 0;    r[2] = d(0);}
                else {r[0] = -d(0)*d(2) ; r[1] = -d(1)*d(2); r[2] = d(0)*d(0) + d(1)*d(1); }
                p1 = next_point;

                const double arrow_s = 0.6 * min_res;
                s  = p1 -  2*arrow_s * d;
                p2 = s + arrow_s * r;
                p3 = s - arrow_s * r;

                render_arrows.col(3*i) = p1;
                render_arrows.col(3*i +1 ) = p2;
                render_arrows.col(3*i +2 ) = p3;

                arrow_colors.emplace_back(render_color);
            }

            sample_point = next_point;
        }
    }
    Livz::drawLineListWithColors(topic_name, render_points, render_colors, 0.03 * min_res, opacity, frame_id, id);
    Livz::renderTrianglesWithColors(topic_name, render_arrows, arrow_colors, opacity, frame_id, id+1);
}
