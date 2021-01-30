//
// Created by xkhuskit on 27.01.21.
//

#include "lidar_localization/models/cloud_cluster/euclidean_cluster.hpp"

#include "glog/logging.h"


namespace lidar_localization {

    EuclideanCluster::EuclideanCluster(const YAML::Node& node) {
        float z_min = node["z_min"].as<float>();
        float z_max = node["z_max"].as<float>();
        int cluster_min_num = node["cluster_min_num"].as<int>();
        int cluster_max_num = node["cluster_max_num"].as<int>();
        int circular_region_num = node["circular_region"]["HDL_64E"]["circular_region_num"].as<int>();
        std::vector<float> region = node["circular_region"]["HDL_64E"]["region"].as<std::vector<float>>();
        float tolerance = node["tolerance"].as<float>();

        SetClusterParam(z_min, z_max, cluster_min_num, cluster_max_num, circular_region_num, region, tolerance);
    }

    EuclideanCluster::EuclideanCluster(float z_min,
                                       float z_max,
                                       int cluster_min_num,
                                       int cluster_max_num,
                                       int circular_region_num,
                                       std::vector<float> region,
                                       float tolerance) {

        SetClusterParam(z_min, z_max, cluster_min_num, cluster_max_num, circular_region_num, region, tolerance);
    }

    bool EuclideanCluster::SetClusterParam(float z_min,
                                           float z_max,
                                           int cluster_min_num,
                                           int cluster_max_num,
                                           int circular_region_num,
                                           std::vector<float> region,
                                           float tolerance){

        pt_ptr_.setFilterFieldName("z");
        pt_ptr_.setFilterLimits(z_min, z_max);
        ec_ptr_.setMinClusterSize(cluster_min_num);
        ec_ptr_.setMaxClusterSize(cluster_max_num);
        region_ = region;
        circular_region_num_ = circular_region_num;
        tolerance_ = tolerance;
        cluster_min_num_ = cluster_min_num;
        tolerance_res_ = 0;


        LOG(INFO) << "Euclidean Cluster parameters:" << std::endl
                  << "Filter Z axel between: " << z_min << "and" << z_max << std::endl
                  << "Cluster size is between: " << cluster_min_num << "and" << std::endl;

        return true;
    }


    bool EuclideanCluster::Cluster(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
        pcl::IndicesPtr pc_idptr_(new std::vector<int>);
        pt_ptr_.setInputCloud(input_cloud_ptr);
        pt_ptr_.filter(*pc_idptr_);
        boost::array<std::vector<int>, 7> indices_array_;
        for (const auto& it : *pc_idptr_) {
            float range = 0.0;
            for (int i=0; i<circular_region_num_; i++) {
                float d = input_cloud_ptr->points[it].x * input_cloud_ptr->points[it].x +
                          input_cloud_ptr->points[it].y * input_cloud_ptr->points[it].y +
                          input_cloud_ptr->points[it].z * input_cloud_ptr->points[it].z;
                if (d > range * range && d <= (range+region_[i]) * (range+region_[i])) {
                    indices_array_[i].push_back(it);
                    break;
                }
                range += region_[i];
            }
        }

        for (int i=0; i<circular_region_num_; i++) {
            tolerance_res_ += tolerance_;
            if (indices_array_[i].size()>cluster_min_num_) {
                boost::shared_ptr<std::vector<int>> indices_array_ptr_(new std::vector<int>(indices_array_[i]));
                pcl::search::KdTree<CloudData::POINT>::Ptr tree_(new pcl::search::KdTree<CloudData::POINT>);
                tree_->setInputCloud(input_cloud_ptr, indices_array_ptr_);

                std::vector<pcl::PointIndices> cluster_indices;
                ec_ptr_.setClusterTolerance(tolerance_res_);
                ec_ptr_.setSearchMethod(tree_);
                ec_ptr_.setInputCloud(input_cloud_ptr);
                ec_ptr_.setIndices(indices_array_ptr_);
                ec_ptr_.extract(cluster_indices);

                for (const auto& cluster_indice : cluster_indices) {
                    /*LOG(INFO) << "Cluster size: " << cluster_indice.indices.size() << std::endl;*/
                    if (cluster_indice.indices.size() < cluster_min_num_) {
                        LOG(INFO) << "filter out small cluster" << std::endl;
                        continue;
                    }
                    for (const int indice : cluster_indice.indices) {
                        filtered_cloud_ptr->points.push_back(input_cloud_ptr->points[indice]);
                    }

                }

            }
        }

        return true;
    }

}