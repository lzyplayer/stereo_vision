//
// Created by vickylzy on 2021/5/11.
//

#include "pointcloud_regist/register.h"


namespace stereo_vision {

    Register::Register(const std::string &pcd_file_path, float leafSize) : leaf_size(leafSize) {
        voxelGrid.setLeafSize(leaf_size, leaf_size, leaf_size);
        // load and clear
        pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr clear_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        localmap_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

        if (pcl::io::loadPCDFile(pcd_file_path, *loaded_cloud) != 0)
            perror("cannnot load map_cloud");
        std::vector<int> indices_index;
        pcl::removeNaNFromPointCloud(*loaded_cloud, *clear_cloud, indices_index);
        voxelGrid.setInputCloud(clear_cloud);
        voxelGrid.filter(*localmap_cloud);
        kdtree.setInputCloud(localmap_cloud);
        icp.setMaxCorrespondenceDistance(0.2);
        icp.setRANSACOutlierRejectionThreshold(0.02);
        icp.setMaximumIterations(50);
        // register
    };

    int Register::compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_stereovision,
                          Eigen::Matrix4f &init_motion,
                          Eigen::Matrix4f &result_motion,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_source_down,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_trans_source_down,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_local_down) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr clear_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        std::vector<int> indices_index;
        pcl::removeNaNFromPointCloud(*pc_stereovision, *clear_cloud, indices_index);
        voxelGrid.setInputCloud(clear_cloud);
        voxelGrid.filter(*filtered_cloud);

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloest_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointXYZ search_point(init_motion(0,3),init_motion(1,3),init_motion(2,3));
        if ( kdtree.radiusSearch (search_point, 3, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            pcl::PointXYZ p;
            for (int i : pointIdxRadiusSearch) {
                p  = localmap_cloud->points[i];
                cloest_cloud->push_back(p);
            }
        }else{
            perror("closest_points nothing left");
        }


        icp.setInputTarget(cloest_cloud);
        icp.setInputSource(filtered_cloud);
        pcl::PointCloud<pcl::PointXYZ> result_cloud;
        icp.align(result_cloud, init_motion); //,curr_pose

        // out param
        pc_source_down = filtered_cloud;
        pc_trans_source_down = result_cloud.makeShared();
        pc_local_down = cloest_cloud;

        double fitness_score = icp.getFitnessScore();
        result_motion = icp.getFinalTransformation();
        std::cout << "fitness score:  " << fitness_score << std::endl;
        std::cout << "init_motion:  " << init_motion << std::endl;
        std::cout << "result_motion:  " << result_motion << std::endl;
        std::cout << "fitness score:  " << fitness_score << std::endl;

        Eigen::Matrix4f delta_T = init_motion.inverse() * result_motion;
        float dist = sqrt(delta_T(0, 3) * delta_T(0, 3) + delta_T(1, 3) * delta_T(1, 3) + delta_T(2, 3) * delta_T(2, 3));
        if (dist > 0.2){
            std::cout << "Registration Failed and Initial value will be used:) with dist:" << dist << std::endl;
            result_motion = init_motion;
        } else {
            std::cout << "Registration Successful:) with dist: "  << dist << std::endl;
            result_motion = result_motion;
        }


        return 0;
    };

}