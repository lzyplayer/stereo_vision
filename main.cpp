#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/filter.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/registration/icp.h>
#include <boost/shared_ptr.hpp>

#include "stereo_match/stereo_constructor.h"
#include "stereo_match/param_loader.h"

using namespace std;


int main() {
    std::cout << "Hello, World!" << std::endl;
//
//    stereo_vision::ParamLoader paramLoader("/home/vickylzy/CLionProjects/stereo_vision/camera_info/camera_stereo_merged_1.yaml");
    stereo_vision::ParamLoader paramLoader("/home/vickylzy/CLionProjects/stereo_vision/camera_info/camera_stereo_2.yaml");
    paramLoader.show_info();
    stereo_vision::StereoConstructor stereoConstructor(paramLoader);

    int block_size = 13;
    int minDisparity = 128;
    int numberOfDisparities = 32;

    stereoConstructor.onInit(block_size, minDisparity, numberOfDisparities);

//    string img1_filename = "/home/vickylzy/Documents/space_station_arm/test_image/left_back.png";
//    string img2_filename = "/home/vickylzy/Documents/space_station_arm/test_image/right_back.png";
    string img1_filename = "/home/vickylzy/Pictures/20201214机械臂腕部相机（正样）力学试验后外参第二次测试/腕部301b/L1100.bmp";
    string img2_filename = "/home/vickylzy/Pictures/20201214机械臂腕部相机（正样）力学试验后外参第二次测试/腕部301b/R1100.bmp";
//
//    string img1_filename = "/home/vickylzy/CLionProjects/stereo_vision/data/imgL.jpg";
//    string img2_filename = "/home/vickylzy/CLionProjects/stereo_vision/data/imgR.jpg";

//    cv::Mat im1 = cv::imread(img1_filename,cv::IMREAD_COLOR);
//    cv::Mat im2 = cv::imread(img2_filename,cv::IMREAD_COLOR);

    cv::Mat im1 = cv::imread(img1_filename, cv::IMREAD_GRAYSCALE);
    cv::Mat im2 = cv::imread(img2_filename, cv::IMREAD_GRAYSCALE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource(new pcl::PointCloud<pcl::PointXYZ>);// uninitialized or initialized
    stereoConstructor.compute_match(im1, im2, cloudSource);


    /**
     *  registration here
     */

//
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



    return 0;
}

//while (1)
//std::cout << getPathName("fifo") << std::endl;
//return 0;