#include <ros/ros.h>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <livz/livz.hpp>

// 漩涡
Eigen::Vector3d field(Eigen::Vector3d pos){
    double x = pos.x();
    double y = pos.y();
    double z = pos.z();

    Eigen::Vector3d originalComponent(x * y, y * z, z * x);
    Eigen::Vector3d vortexCenter(0, 0, 0);
    Eigen::Vector3d relativePos = pos - vortexCenter;

    double vortexStrength           = 1.0;
    Eigen::Vector3d vortexComponent = vortexStrength * Eigen::Vector3d(-relativePos.y(), relativePos.x(), 0);
    Eigen::Vector3d result          = originalComponent + vortexComponent;

    return result;
}

// 漩涡
Eigen::Vector3d field2(Eigen::Vector3d pos){
    double x = pos.x();
    double y = pos.y();
    double z = pos.z();

    Eigen::Vector3d originalComponent(x * y, y * z, z * x);
    Eigen::Vector3d vortexCenter(0, 2, 0);
    Eigen::Vector3d relativePos = pos - vortexCenter;

    double vortexStrength           = 1.0;
    Eigen::Vector3d vortexComponent = vortexStrength * Eigen::Vector3d(-relativePos.y(), relativePos.x(), 0);
    Eigen::Vector3d result          = originalComponent + vortexComponent;

    return result;
}

void demoField()
{
    Livz::renderVectorField("field", field, Eigen::Vector3d(-4,-4,0), Eigen::Vector3d(4,4,1), Eigen::Vector3d(0.5,0.5,0.5), false);
    Livz::Delay(2000);
    Livz::clearAll("field");
    Livz::renderStreamLines("streamline", field, Eigen::Vector3d(-4,-4,0), Eigen::Vector3d(4,4,1), Eigen::Vector3d(0.1,0.1,0.1), 0 ,19 , true);
    Livz::Delay(4000);

    Livz::clearAll("streamline");

    LAnimateParam ani_param("streamline", 5.0, RATE_FUNC::linear, false); 
    Livz::createAnimate(ani_param, Livz::renderStreamLines,
                        LPARAMS( field, Eigen::Vector3d(-4,-4,0), Eigen::Vector3d(4,4,1), Eigen::Vector3d(0.2,0.2,0.1), 0 ,4   , true, LCOLOR::RED, LCOLOR::YELLOW, -1.0, "map", 1),
                        LPARAMS( field, Eigen::Vector3d(-4,-4,0), Eigen::Vector3d(4,4,1), Eigen::Vector3d(0.2,0.2,0.1), 36 ,40 , true, LCOLOR::RED, LCOLOR::YELLOW, -1.0, "map", 1) );
    
    Livz::DelayAccordingtoAnimate( ani_param );
    Livz::clearAll("streamline");
    // 使用VECTOR_FIELD将函数转换为智能指针
    Livz::createAnimate(ani_param, Livz::renderStreamLines,
                        LPARAMS( VECTOR_FIELD( field  ), Eigen::Vector3d(-4,-4,0), Eigen::Vector3d(4,4,0.1), Eigen::Vector3d(0.2,0.2,0.3), 0 ,10   , false, LCOLOR::RED, LCOLOR::YELLOW, -1.0, "map", 1),
                        LPARAMS( VECTOR_FIELD( field2 ), Eigen::Vector3d(-4,-4,0), Eigen::Vector3d(4,4,0.1), Eigen::Vector3d(0.2,0.2,0.3), 0 ,10   , false, LCOLOR::RED, LCOLOR::YELLOW, -1.0, "map", 1) );

}