//
// Created by vickylzy on 2021/5/9.
//

#include <iostream>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/filter.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/registration/icp.h>
//#include <boost/shared_ptr.hpp>

#include "stereo_match/stereo_constructor.h"
#include "stereo_match/param_loader.h"
#include "station_prarm_reader.hpp"
#include "stereo_match/transform_utility.hpp"

using namespace std;


int main() {
    std::cout << "Hello, World!" << std::endl;
// init
    stereo_vision::ParamLoader paramLoader_l("../camera_info/camera_stereo_1.yaml");
    stereo_vision::ParamLoader paramLoader_r("../camera_info/camera_stereo_2.yaml");
    paramLoader_l.show_info();
    paramLoader_r.show_info();
    stereo_vision::StereoConstructor stereoConstructor_a1(paramLoader_l);
    stereo_vision::StereoConstructor stereoConstructor_b2(paramLoader_r);

    int block_size = 9;
    int minDisparity = 128;
    int numberOfDisparities = 32;

    stereoConstructor_a1.onInit(block_size, minDisparity, numberOfDisparities);
    stereoConstructor_b2.onInit(block_size, minDisparity, numberOfDisparities);
    stereo_vision::StereoConstructor* stereoConstructor_ptr;

    // TODO: tell backend what image you need

    // endless loop !
    while (true){
        // read realtime param
        std::vector<std::string> station_param = stereo_vision::getArgList(ARG_FETCH_BLOCK, "/dev/shm/c590/cacheA/", "data.txt");
        Eigen::Matrix4f arm_trans = stereo_vision::stationParam2rot(station_param);
        // choose arm end
        if (station_param[14]=="55")
            stereoConstructor_ptr = &stereoConstructor_a1;
        else if (station_param[14]=="AA")
            stereoConstructor_ptr = &stereoConstructor_b2;
        else
            continue;
        //  get eigen transform
        //  .........init transform
        //

        // TODO: read im backend provided
        string img1_filename = "/home/vickylzy/Documents/space_station_arm/test_image/left_back.png";
        string img2_filename = "/home/vickylzy/Documents/space_station_arm/test_image/right_back.png";

        cv::Mat im1 = cv::imread(img1_filename,cv::IMREAD_GRAYSCALE);
        cv::Mat im2 = cv::imread(img2_filename,cv::IMREAD_GRAYSCALE);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource(new pcl::PointCloud<pcl::PointXYZ>);// uninitialized or initialized
        stereoConstructor_ptr->compute_match(im1, im2, cloudSource);
    }

    /**
     *  registration here
     *  init transform: compute needed
     *  data pointcloud: cloudSource
     */

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRef(new pcl::PointCloud<pcl::PointXYZ>);
////    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRef_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//
//
//    pcl::io::loadPCDFile("/home/shaoyi/ICP/pt1_single.pcd", *cloudRef);
////    pcl::io::loadPCDFile("/home/shaoyi/ICP/pt2_single.pcd", *cloudSource);
//    Eigen::Matrix4f m_InitTF;
//    m_InitTF.setIdentity();
//    m_InitTF << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1;
//
//    pcl::VoxelGrid<pcl::PointXYZ> sor;
//    sor.setInputCloud(cloudRef);
//    sor.setLeafSize(100.0f, 100.0f, 100.0f);
//    sor.filter(*cloudRef_filtered);
//
//    sor.setInputCloud(cloudSource);
//    sor.setLeafSize(100.0f, 100.0f, 100.0f);
//    sor.filter(*cloudSource_filtered);
//
//    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//    icp.setInputSource(cloudSource_filtered);
//    icp.setInputTarget(cloudRef_filtered);
//
//    pcl::PointCloud<pcl::PointXYZ> ptcloud_transformed;
//    icp.align(ptcloud_transformed, m_InitTF);
//
//    //pcl::visualization::CloudViewer viewer("Cloud Viewer");
//
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloudRef_filtered, 0, 255, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_color(ptcloud_transformed.makeShared(), 255, 0, 0);
//
//
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//    viewer->addPointCloud(cloudRef_filtered, target_color, "cloud1");
//    viewer->addPointCloud(ptcloud_transformed.makeShared(), final_color, "cloud2");
//
//    std::cout << icp.getFinalTransformation() << std::endl;
//
//    while (!viewer->wasStopped())
//    {
//        viewer->spinOnce(100);
//        usleep(100000);
////        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }
/***
 *  registration end here
 */
//  processing online
//    {
//        while (1){
//            string filepath =  getPathName("fifo") ;
//            // read image
//            stereoConstructor.compute_match(im1,im2,cloudTrimmed);
//            // point cloud registration
//        }
//    }

    return 0;
}

//while (1)
//std::cout << getPathName("fifo") << std::endl;
//return 0;