//
// Created by vickylzy on 2021/5/11.
//

#include "pointcloud_regist/register.h"


namespace stereo_vision {

    Register::Register(const std::string &pcd_file_path) {
        //
        voxelGrid.setLeafSize(leaf_size, leaf_size, leaf_size);
        // load and clear
        pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr clear_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        localmap_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

        if (!pcl::io::loadPCDFile(pcd_file_path, *loaded_cloud))
            perror("cannnot load map_cloud");
        std::vector<int> indices_index;
        pcl::removeNaNFromPointCloud(*loaded_cloud, *clear_cloud, indices_index);
        voxelGrid.setInputCloud(clear_cloud);
        voxelGrid.filter(*localmap_cloud);
        // register
        pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(
                new pclomp::NormalDistributionsTransform<pcl::PointXYZ,
                        pcl::PointXYZ>());
        ndt->setTransformationEpsilon(TransformationEpsilon);
        ndt->setResolution(ndtResolution);
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        registration_ptr = ndt;
    };

    int Register::compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_stereovision, Eigen::Matrix4f &init_motion, Eigen::Matrix4f &result_motion) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr clear_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        std::vector<int> indices_index;
        pcl::removeNaNFromPointCloud(*pc_stereovision, *clear_cloud, indices_index);
        voxelGrid.setInputCloud(clear_cloud);
        voxelGrid.filter(*filtered_cloud);

        registration_ptr->setInputTarget(localmap_cloud);
        registration_ptr->setInputSource(filtered_cloud);
        pcl::PointCloud<pcl::PointXYZ> result_cloud;
        registration_ptr->align(result_cloud, init_motion); //,curr_pose
        // TODO: post process needed on result @pengyu
        double fitness_score = registration_ptr->getFitnessScore();
        std::cout << "fitness score:  " << fitness_score << std::endl;
        result_motion = registration_ptr->getFinalTransformation();
        return 0;
    };

}