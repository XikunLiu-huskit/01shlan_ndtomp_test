find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_library(ndt_omp
        src/models/registration/pclomp/voxel_grid_covariance_omp.cpp
        src/models/registration/pclomp/ndt_omp.cpp
        src/models/registration/pclomp/gicp_omp.cpp
        )

add_library(cluster
        src/models/cloud_cluster/euclidean_cluster.cpp
        )