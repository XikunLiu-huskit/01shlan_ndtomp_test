//
// Created by xkhuskit on 24.01.21.
//

#ifndef INC_01_LIDAR_ODOMETRY_NDT_DK_H
#define INC_01_LIDAR_ODOMETRY_NDT_DK_H

#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/registration/pclomp/ndt_omp.h"


namespace lidar_localization{
class NDTDKRegistration: public RegistrationInterface {
    public:
        NDTDKRegistration(const YAML::Node& node);
        NDTDKRegistration(float res, std::string search_method);

        bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
        bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                       const Eigen::Matrix4f& predict_pose,
                       CloudData::CLOUD_PTR& result_cloud_ptr,
                       Eigen::Matrix4f& result_pose) override;

    private:
        bool SetRegistrationParam(float res, std::string search_method);

    private:
        pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndtdk_ptr_;

};
}



#endif //INC_01_LIDAR_ODOMETRY_NDT_DK_H
