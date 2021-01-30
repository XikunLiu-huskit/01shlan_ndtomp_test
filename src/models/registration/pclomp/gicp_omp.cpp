#include "lidar_localization/models/registration/pclomp/gicp_omp.h"
#include "lidar_localization/models/registration/pclomp/gicp_omp_impl.hpp"

template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>;

