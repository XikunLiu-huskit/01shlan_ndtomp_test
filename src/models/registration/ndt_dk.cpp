//
// Created by xkhuskit on 25.01.21.
//

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "lidar_localization/models/registration/ndt_dk.h"
#include "glog/logging.h"

namespace lidar_localization {
    NDTDKRegistration::NDTDKRegistration(const YAML::Node& node)
    :ndtdk_ptr_(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>()) {
        float res = node["res"].as<float>();
        std::string search_method = node["search_method"].as<std::string>();

        SetRegistrationParam(res, search_method);
    }

    NDTDKRegistration::NDTDKRegistration(float res, std::string search_method)
            :ndtdk_ptr_(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>()) {

        SetRegistrationParam(res, search_method);
    }

    bool NDTDKRegistration::SetRegistrationParam(float res, std::string search_method) {
        int num_threads = omp_get_max_threads();
        ndtdk_ptr_->setResolution(res);
        pclomp::NeighborSearchMethod search_mth = pclomp::KDTREE;
        if (search_method == "pclomp_DIRECT7"){
            search_mth = pclomp::DIRECT7;
        }
        else {
            LOG(ERROR) << "Search method " << search_method << " NOT FOUND!";
            return false;
        }
        ndtdk_ptr_->setNeighborhoodSearchMethod(search_mth);
        ndtdk_ptr_->setNumThreads(num_threads);
        LOG(INFO) << "NDT_DK params:" << std::endl
                  << "res: " << res << ", "
                  << "search_method: " << search_method
                  << std::endl << std::endl;

        return true;
    }

    bool NDTDKRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
        ndtdk_ptr_->setInputTarget(input_target);

        return true;
    }

    bool NDTDKRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source,
                                    const Eigen::Matrix4f& predict_pose,
                                    CloudData::CLOUD_PTR& result_cloud_ptr,
                                    Eigen::Matrix4f& result_pose) {
        ndtdk_ptr_->setInputSource(input_source);
        ndtdk_ptr_->align(*result_cloud_ptr, predict_pose);
        result_pose = ndtdk_ptr_->getFinalTransformation();

        return true;
    }

}
