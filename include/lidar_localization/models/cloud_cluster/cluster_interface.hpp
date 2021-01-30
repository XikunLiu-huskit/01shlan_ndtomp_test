//
// Created by xkhuskit on 27.01.21.
//

#ifndef LIDAR_LOCALIZATION_CLUSTER_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_CLUSTER_INTERFACE_HPP_


#include <yaml-cpp/yaml.h>
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class ClusterInterface {
    public:
        virtual ~ClusterInterface() = default;
        virtual bool Cluster(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr) = 0;
    };
}

#endif //LIDAR_LOCALIZATION_CLUSTER_INTERFACE_HPP_
