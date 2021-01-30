//
// Created by xkhuskit on 27.01.21.
//

#ifndef LIDAR_LOCALIZATION_EUCLIDEAN_CLUSTER_HPP_
#define LIDAR_LOCALIZATION_EUCLIDEAN_CLUSTER_HPP_


#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include "lidar_localization/models/cloud_cluster/cluster_interface.hpp"
#include <pcl_conversions/pcl_conversions.h>


namespace lidar_localization {
class EuclideanCluster: public ClusterInterface {
  public:
      EuclideanCluster(const YAML::Node& node);
      EuclideanCluster(float z_min,
                       float z_max,
                       int cluster_min_num,
                       int cluster_max_num,
                       int circular_region_num,
                       std::vector<float> region,
                       float tolerance);

      bool Cluster(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr) override;

  private:
      bool SetClusterParam(float z_min,
                           float z_max,
                           int cluster_min_num,
                           int cluster_max_num,
                           int circular_region_num,
                           std::vector<float> region,
                           float tolerance);

  private:
      pcl::EuclideanClusterExtraction<CloudData::POINT> ec_ptr_;
      pcl::PassThrough<CloudData::POINT> pt_ptr_;
      std::vector<float> region_ = {};
      float tolerance_ = 0.0;
      float tolerance_res_ = 0.0;
      int cluster_min_num_ = 0;
      int circular_region_num_ = 0;


};
}


#endif //LIDAR_LOCALIZATION_EUCLIDEAN_CLUSTER_HPP_
