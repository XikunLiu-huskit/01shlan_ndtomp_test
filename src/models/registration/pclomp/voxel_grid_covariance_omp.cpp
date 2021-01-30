#include <lidar_localization/models/registration/pclomp/voxel_grid_covariance_omp.h>
#include <lidar_localization/models/registration/pclomp/voxel_grid_covariance_omp_impl.hpp>

template class pclomp::VoxelGridCovariance<pcl::PointXYZ>;
template class pclomp::VoxelGridCovariance<pcl::PointXYZI>;
