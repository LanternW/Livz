// Copyright (c) [2023] [Lantern]
// 
// This file is part of [Livz]
// 
// This project is licensed under the MIT License.
// See LICENSE.txt for details.
#include "include/livz_animate.hpp"

namespace RATE_FUNC{

    double linear(double t){ 
        return std::max(0.0, std::min(t, 1.0)); 
    }

    double quadratic(double t){
        t = std::max(0.0, std::min(t, 1.0));  
        return t * t;
    }

    double cubic(double t){
        t = std::max(0.0, std::min(t, 1.0)); 
        return t * t * t;
    }

    double sine(double t){
        t = std::max(0.0, std::min(t, 1.0)); 
        return sin(t * M_PI_2);
    }

    double circular(double t){
        t = std::max(0.0, std::min(t, 1.0)); 
        return 1 - sqrt(1 - t * t);
    }

    double elastic(double t){
        t = std::max(0.0, std::min(t, 1.0)); 
        return -pow(2, 10 * (t - 1)) * sin((t - 1.1) * 5 * M_PI);
    }

    double bounce(double t){
        t = std::max(0.0, std::min(t, 1.0)); 
        if (t < 4 / 11.0) {
            return (121 * t * t) / 16.0;
        } else if (t < 8 / 11.0) {
            return (363 / 40.0 * t * t) - (99 / 10.0 * t) + 17 / 5.0;
        } else if (t < 9 / 10.0) {
            return (4356 / 361.0 * t * t) - (35442 / 1805.0 * t) + 16061 / 1805.0;
        } else {
            return (54 / 5.0 * t * t) - (513 / 25.0 * t) + 268 / 25.0;
        }
    }
}

namespace INTERPOLATION {
    void interpolation(double begin, double end, double& result, double tick) {
        result = begin + (end - begin) * tick;
    }
    void interpolation(int begin, int end, int& result, double tick){
        result = static_cast<int>(begin + (end - begin) * tick);
    }
    void interpolation(Eigen::Vector2d begin, Eigen::Vector2d end,  Eigen::Vector2d& result, double tick) {
        result = begin + (end - begin) * tick;
    }
    void interpolation(Eigen::Vector3d begin, Eigen::Vector3d end,  Eigen::Vector3d& result, double tick) {
        result = begin + (end - begin) * tick;
    }
    void interpolation(Eigen::Vector4d begin, Eigen::Vector4d end,  Eigen::Vector4d& result, double tick) {
        result = begin + (end - begin) * tick;
    }

    void interpolation(Eigen::Matrix3Xd begin, Eigen::Matrix3Xd end,  Eigen::Matrix3Xd& result, double tick) {
        const int N1   = begin.cols();
        const int N2   = end.cols();
        const int MAX_N = std::max(N1,N2);
        const int MIN_N = std::min(N1,N2);

        double step;
        Eigen::Vector3d position;
        if (N1 != N2) {
            result.resize(3, MAX_N );
            step =  double(MIN_N)/ double(MAX_N);
            Eigen::Matrix3Xd& more_one = (N1 > N2) ? begin : end;
            Eigen::Matrix3Xd& less_one = (N1 > N2) ? end : begin;
            for( size_t i = 0 ; i < MAX_N; i++){
                int index = int(i * step);
                if(N1 > N2){ interpolation(more_one.col(i) ,less_one.col(index)  , position, tick ); }
                else       { interpolation(less_one.col(index) , more_one.col(i) , position, tick ); }
                
                result.col(i) = position;
            }
        }
        else {
            result.resize(3, N1);
            for( size_t i = 0 ; i < N1; i++){
                interpolation(end.col(i) , begin.col(i) , position, tick );
                result.col(i) = position;
            }
        }

    }

    void interpolation(std::vector<Eigen::Vector3d> begin, std::vector<Eigen::Vector3d> end,  std::vector<Eigen::Vector3d>& result, double tick) {
        const int N1   = begin.size();
        const int N2   = end.size();
        const int MAX_N = std::max(N1,N2);
        const int MIN_N = std::min(N1,N2);

        double step ;
        if (N1 != N2) {
            result.resize( MAX_N );
            step =  double(MIN_N)/double(MAX_N);
            std::vector<Eigen::Vector3d>& more_one = (N1 > N2) ? begin : end;
            std::vector<Eigen::Vector3d>& less_one = (N1 > N2) ? end : begin;
            for( size_t i = 0 ; i < MAX_N; i++){
                int index = int(i * step);
                if(N1 > N2){interpolation(more_one[i] , less_one[index] , result[i] , tick );}
                else       {interpolation(less_one[index] , more_one[i] , result[i] , tick );}
            }
        }
        else {
            result.resize(N1);
            for( size_t i = 0 ; i < N1; i++){
                interpolation(end[i] , begin[i] , result[i] , tick );
            }
        }

    }

    void interpolation(pcl::PointCloud<pcl::PointXYZ> begin, pcl::PointCloud<pcl::PointXYZ> end,  pcl::PointCloud<pcl::PointXYZ>& result, double tick)
    {
        const int N1   = begin . size();
        const int N2   = end   . size();
        const int MAX_N = std::max(N1,N2);
        const int MIN_N = std::min(N1,N2);

        double step ;
        pcl::PointXYZ point;
        double tx,ty,tz;
        if (N1 != N2) {
            result.points.resize( MAX_N);
            step =  double(MIN_N)/double(MAX_N);
            pcl::PointCloud<pcl::PointXYZ>& more_one = (N1 > N2) ? begin : end;
            pcl::PointCloud<pcl::PointXYZ>& less_one = (N1 > N2) ? end : begin;
            for( size_t i = 0 ; i < MAX_N; i++){
                int index = int(i * step);
                if(N1 > N2) {
                    interpolation(less_one.points[index].x ,  more_one.points[i].x , tx , tick);
                    interpolation(less_one.points[index].y ,  more_one.points[i].y , ty , tick);
                    interpolation(less_one.points[index].z ,  more_one.points[i].z , tz , tick);
                }
                else {
                    interpolation(more_one.points[i].x , less_one.points[index].x , tx , tick);
                    interpolation(more_one.points[i].y , less_one.points[index].y , ty , tick);
                    interpolation(more_one.points[i].z , less_one.points[index].z , tz , tick);
                }
                point.x = tx ; point.y = ty; point.z = tz;
                result.points[i] = point;
            }
        }
        else {
            result.points.resize(N1);
            for( size_t i = 0 ; i < N1; i++){
                interpolation(begin.points[i].x ,  end.points[i].x , tx , tick);
                interpolation(begin.points[i].y ,  end.points[i].y , ty , tick);
                interpolation(begin.points[i].z ,  end.points[i].z , tz , tick);
                point.x = tx ; point.y = ty; point.z = tz;
                result.points[i] = point;
            }
        }
    }

    void interpolation(pcl::PointCloud<pcl::PointXYZI> begin, pcl::PointCloud<pcl::PointXYZI> end,  pcl::PointCloud<pcl::PointXYZI>& result, double tick)
    {
        const int N1   = begin . size();
        const int N2   = end   . size();
        const int MAX_N = std::max(N1,N2);
        const int MIN_N = std::min(N1,N2);

        double step ;
        pcl::PointXYZI point;
        double tx,ty,tz,ti;
        if (N1 != N2) {
            result.points.resize( MAX_N);
            step =  double(MIN_N)/double(MAX_N);
            pcl::PointCloud<pcl::PointXYZI>& more_one = (N1 > N2) ? begin : end;
            pcl::PointCloud<pcl::PointXYZI>& less_one = (N1 > N2) ? end : begin;
            for( size_t i = 0 ; i < MAX_N; i++){
                int index = int(i * step);
                if(N1 > N2) {
                    interpolation(less_one.points[index].x ,  more_one.points[i].x , tx , tick);
                    interpolation(less_one.points[index].y ,  more_one.points[i].y , ty , tick);
                    interpolation(less_one.points[index].z ,  more_one.points[i].z , tz , tick);
                    interpolation(less_one.points[index].intensity ,  more_one.points[i].intensity , ti , tick);
                }
                else {
                    interpolation( more_one.points[i].x ,less_one.points[index].x ,  tx , tick);
                    interpolation( more_one.points[i].y ,less_one.points[index].y ,  ty , tick);
                    interpolation( more_one.points[i].z ,less_one.points[index].z ,  tz , tick);
                    interpolation( more_one.points[i].intensity ,less_one.points[index].intensity ,  ti , tick);
                }
                point.x = tx ; point.y = ty; point.z = tz; point.intensity = ti;
                result.points[i] = point;
            }
        }
        else {
            result.points.resize(N1);
            for( size_t i = 0 ; i < N1; i++){
                interpolation(begin.points[i].x ,  end.points[i].x , tx , tick);
                interpolation(begin.points[i].y ,  end.points[i].y , ty , tick);
                interpolation(begin.points[i].z ,  end.points[i].z , tz , tick);
                interpolation(begin.points[i].intensity ,  end.points[i].intensity , ti , tick);
                point.x = tx ; point.y = ty; point.z = tz; point.intensity = ti;
                result.points[i] = point;
            }
        }
    }

}
