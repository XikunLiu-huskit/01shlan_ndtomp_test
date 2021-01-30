//
// Created by xkhuskit on 25.01.21.
//

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "lidar_localization/models/registration/gicp_dk.h"
#include "glog/logging.h"

namespace lidar_localization {
    GICPDKRegistration::GICPDKRegistration(const YAML::Node& node)
    :gicpdk_ptr_(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>()) {

    }

    /*GICPDKRegistration::GICPDKRegistration(float res, std::string search_method)
            :ndtdk_ptr_(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>()) {

        SetRegistrationParam(res, search_method);
    }

    bool GICPDKRegistration::SetRegistrationParam(float res, std::string search_method) {
        int num_threads = omp_get_max_threads();
        ndtdk_ptr_->setResolution(res);
        pclomp::NeighborSearchMethod search_mth = pclomp::KDTREE;
        if (search_method == "pclomp_DIRECT7"){
            search_mth = pclomp::DIRECT7;
        }
        ndtdk_ptr_->setNeighborhoodSearchMethod(search_mth);
        ndtdk_ptr_->setNumThreads(num_threads);
        LOG(INFO) << "NDT_DK params:" << std::endl
                  << "res: " << res << ", "
                  << "search_method: " << search_method
                  << std::endl << std::endl;

        return true;
    }*/

    bool GICPDKRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
        gicpdk_ptr_->setInputTarget(input_target);

        return true;
    }

    bool GICPDKRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source,
                                    const Eigen::Matrix4f& predict_pose,
                                    CloudData::CLOUD_PTR& result_cloud_ptr,
                                    Eigen::Matrix4f& result_pose) {
        gicpdk_ptr_->setInputSource(input_source);
        gicpdk_ptr_->align(*result_cloud_ptr, predict_pose);
        result_pose = gicpdk_ptr_->getFinalTransformation();

        return true;
    }

}
