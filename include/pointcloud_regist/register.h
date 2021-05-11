//
// Created by vickylzy on 2021/5/11.
//

#ifndef SRC_REGISTRATION_REGISTER_H
#define SRC_REGISTRATION_REGISTER_H

#include <Eigen/Dense>
#include <pclomp/ndt_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>


namespace stereo_vision {


    class Register {
    public:
        explicit Register(const std::string &pcd_file_path);

        int compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_stereovision, Eigen::Matrix4f &init_motion, Eigen::Matrix4f &result_motion);

    private:
        const float leaf_size = 0.04f;
        const float TransformationEpsilon = 0.01f;
        const float ndtResolution = 1.0f;
        pcl::PointCloud<pcl::PointXYZ>::Ptr localmap_cloud;
        pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
        pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration_ptr;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    };


}


#endif //SRC_REGISTRATION_REGISTER_H
