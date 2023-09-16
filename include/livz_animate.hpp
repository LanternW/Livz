// Copyright (c) [2023] [Lantern]
// 
// This file is part of [Livz]
// 
// This project is licensed under the MIT License.
// See LICENSE.txt for details.
#ifndef LROSVIS_ANIMATE_H
#define LROSVIS_ANIMATE_H

#include <functional>
#include <map>
#include <cmath>
#include <any>
#include <memory>
#include <functional>
#include <iostream>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>




namespace RATE_FUNC{

    double linear(double t);

    double quadratic(double t);

    double cubic(double t);

    double sine(double t);

    double circular(double t);

    double elastic(double t);

    double bounce(double t);
}

namespace INTERPOLATION{
    void interpolation(double begin, double end, double& result, double tick);
    void interpolation(int begin, int end, int& result, double tick); 
    void interpolation(Eigen::Vector2d begin, Eigen::Vector2d end,  Eigen::Vector2d& result, double tick);
    void interpolation(Eigen::Vector3d begin, Eigen::Vector3d end,  Eigen::Vector3d& result, double tick);
    void interpolation(Eigen::Vector4d begin, Eigen::Vector4d end,  Eigen::Vector4d& result, double tick);
    void interpolation(std::vector<Eigen::Vector3d> begin, std::vector<Eigen::Vector3d> end,  std::vector<Eigen::Vector3d>& result, double tick);
    void interpolation(Eigen::Matrix3Xd begin, Eigen::Matrix3Xd end,  Eigen::Matrix3Xd& result, double tick);

    void interpolation(pcl::PointCloud<pcl::PointXYZ> begin, pcl::PointCloud<pcl::PointXYZ> end, pcl::PointCloud<pcl::PointXYZ>& result, double tick);
    void interpolation(pcl::PointCloud<pcl::PointXYZI> begin, pcl::PointCloud<pcl::PointXYZI> end,  pcl::PointCloud<pcl::PointXYZI>& result, double tick);

    template <typename R, typename... Args>
    void interpolation(std::function<R(Args...)> begin, std::function<R(Args...)> end, std::function<R(Args...)>& result, double tick)
    {
        result = [begin, end, tick](Args... args) -> R {
                    R begin_value = begin(args...);
                    R end_value   = end(args...);
                    decltype(begin_value) result_value;
                    interpolation(begin_value, end_value, result_value ,tick);
                    return result_value;
                };
    }

    // 通用模板版本
    template <typename T>
    void interpolation(T begin, T end, T& result, double tick) {
        result = begin;
    }
}

class LAnimateParam 
{
public:
    typedef std::function<double(const double)> Rate_function;
public:
    LAnimateParam(){}
    LAnimateParam(const std::string topic_name, double dur = 3.0, Rate_function rate_function = RATE_FUNC::linear, bool loop = false)
    {
        topic_name_ = topic_name;
        duration_   = dur;
        rate_func_  = rate_function;
        loop_       = loop;
    }
    bool   loop_              = false; //是否循环
    double duration_          = 1.0;   //持续时间
    Rate_function rate_func_  = RATE_FUNC::linear; //插值函数
    std::string topic_name_;
};


template<typename Func, typename Params>
class LAnimateTask
{
public:
    LAnimateTask(LAnimateParam param, Func func, Params params_begin, Params params_end)
        : function_{func}
        , params_begin_{params_begin}
        , params_end_{params_end}
        , finished_{false}
        , tick_{0.0}
    { 
        param_      = param;
        topic_name_ = param.topic_name_;
        begin_tick_ = ros::Time::now();
    }
    ~LAnimateTask(){
        // std::cout<<"DELETE!!!!!"<<std::endl;
    }

    void update()
    {
        double real_tick_ = (ros::Time::now() - begin_tick_).toSec() / param_.duration_;
        int floor_sec     = floor(real_tick_);
        tick_             = real_tick_ - floor_sec;
        nonlinear_tick_   = param_.rate_func_(tick_);

        decltype(params_begin_) interpolated_params;
        interpolate_params(params_begin_, params_end_, interpolated_params, nonlinear_tick_);

        if(real_tick_ > 1.0){
            if(param_.loop_ == false){ finished_ = true; }
        }
        

        if(finished_ == false) {
            std::apply(function_, interpolated_params);
        }
    }
    inline bool finished(){return finished_;}

private:
    LAnimateParam param_;
    std::string  topic_name_;
    bool finished_;
    double tick_;
    double nonlinear_tick_;
    ros::Time begin_tick_;

    Func function_;
    Params params_begin_; 
    Params params_end_;  

    template <typename T, std::size_t... Is>
    void detail_interpolate_tuples(const T& begin, const T& end, T& result, double tick, std::index_sequence<Is...>) {
        ((INTERPOLATION::interpolation(std::get<Is>(begin), std::get<Is>(end), std::get<Is>(result), tick),  ...));
    }

    template <typename... Types>
    void interpolate_params(const std::tuple<Types...>& begin, const std::tuple<Types...>& end, std::tuple<Types...>& result, double tick) {
        detail_interpolate_tuples(begin, end, result, tick, std::index_sequence_for<Types...>{});

    }

};


class LAnimateTaskWrapper {
    std::function<void()> update_accessor;
    std::function<bool()> finished_accessor;
    std::shared_ptr<void> task_; // 使用void类型的shared_ptr来保存指针,不保存的话，咱们C++的智能指针可就自做主张给对象扬了

public:
    
    template<typename Func, typename Params>
    LAnimateTaskWrapper(std::shared_ptr<LAnimateTask<Func, Params>> task)
        : task_(task) {
        update_accessor = [capturedTask = task]() {
            capturedTask -> update();
        };
        finished_accessor = [capturedTask = task]() {
            return capturedTask->finished();
        };
    }
    
    void Update() {
        update_accessor();
    }
    bool Finished() {
        return finished_accessor();
    }

    
};

class LCustomUpdater {
public:
    LCustomUpdater(const std::string& name, std::function<bool(const double)> func) :updater_name_(name), function_(func) {
        finished_   = false;
        begin_tick_ = ros::Time::now();
    }

    void Update() {
        double real_tick_ = (ros::Time::now() - begin_tick_).toSec();
        finished_ = function_( real_tick_ );
    }

    inline bool Finished() { return finished_; }

    std::string updater_name_;
private:
    bool finished_;
    double tick_;
    ros::Time begin_tick_;
    std::function<bool(const double)> function_;

};


#endif
