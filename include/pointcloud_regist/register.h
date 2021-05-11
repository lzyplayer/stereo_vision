//
// Created by vickylzy on 2021/5/11.
//

#ifndef SRC_REGISTRATION_REGISTER_H
#define SRC_REGISTRATION_REGISTER_H

#include <Eigen/Dense>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>


namespace stereo_vision {


    class Register {
    public:
        Register(const std::string &pcd_file_path, float leafSize = 0.04f);

        int compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_stereovision, Eigen::Matrix4f &init_motion, Eigen::Matrix4f &result_motion);

    private:
        float leaf_size ;

        pcl::PointCloud<pcl::PointXYZ>::Ptr localmap_cloud;
        pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    };


}


#endif //SRC_REGISTRATION_REGISTER_H
