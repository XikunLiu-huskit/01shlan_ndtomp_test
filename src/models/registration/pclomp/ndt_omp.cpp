#include <lidar_localization/models/registration/pclomp/ndt_omp.h>
#include <lidar_localization/models/registration/pclomp/ndt_omp_impl.hpp>

template class pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
