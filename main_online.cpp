//
// Created by vickylzy on 2021/5/9.
//
#include <csignal>
#include <iostream>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/registration/icp.h>
//#include <boost/shared_ptr.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/eigen.hpp>
#include "stereo_match/stereo_constructor.h"
#include "stereo_match/param_loader.h"
#include "station_prarm_reader.hpp"
#include "stereo_match/transform_utility.hpp"
#include "stereo_match/transfer_info.hpp"
#include "pointcloud_regist/register.h"

using namespace std;

namespace {

    volatile std::sig_atomic_t camera_l_signal;
    volatile std::sig_atomic_t camera_r_signal;

}  // namespace



int main() {
    //camera signal declare

    std::signal(SIGRTMIN + 1, [](int s) { camera_l_signal = s; });
    std::signal(SIGRTMIN + 2, [](int s) { camera_r_signal = s; });
    tran_info::clear_pidtxt();
    tran_info::save_pidtxt();
// init
    stereo_vision::ParamLoader paramLoader_1("../camera_info/camera_stereo_1.yaml");
    stereo_vision::ParamLoader paramLoader_2("../camera_info/camera_stereo_2.yaml");
//    paramLoader_1.show_info();
//    paramLoader_2.show_info();
    stereo_vision::StereoConstructor stereoConstructor_a1(paramLoader_1);
    stereo_vision::StereoConstructor stereoConstructor_b2(paramLoader_2);

    int block_size = 9;
    int minDisparity = 128;
    int numberOfDisparities = 64;

    stereoConstructor_a1.onInit(block_size, minDisparity, numberOfDisparities);
    stereoConstructor_b2.onInit(block_size, minDisparity, numberOfDisparities);
    stereo_vision::Register pc_register("../data/point_cloud_body_real.pcd");
    stereo_vision::StereoConstructor *stereoConstructor_ptr;
    stereo_vision::ParamLoader *paramLoader_ptr;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // endless loop !
    while (true) {
        // TODO: tell backend what image you need
        tran_info::write_reqlist();
        while (!camera_l_signal | !camera_r_signal) {
            // wait image arrived
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        // read realtime param
        std::vector<std::string> station_param = stereo_vision::getArgList(ARG_FETCH_BLOCK, tran_info::trans_dir_path, "data.txt");
        // choose arm end
        if (stoi(station_param[15]) == 85) {
            // arm lock end a1, use cam b2
            stereoConstructor_ptr = &stereoConstructor_b2;
            paramLoader_ptr = &paramLoader_2;
        } else if (stoi(station_param[15]) == 170) {
            // arm lock end b2, use cam a1
            stereoConstructor_ptr = &stereoConstructor_a1;
            paramLoader_ptr = &paramLoader_1;
        } else {
            perror("unknown arm end");
            continue;
        }
        Eigen::Matrix4f M_w_ee = stereo_vision::stationParam2rot(station_param);
        Eigen::Matrix4f M_ee_lc = Eigen::Matrix4f::Zero();
        cv::cv2eigen(paramLoader_ptr->getMEeLcam(), M_ee_lc);
        Eigen::Matrix4f M_w_lc = M_w_ee * M_ee_lc;

        string img_left_filename(tran_info::im_cam0_path);
        string img_right_filename(tran_info::im_cam1_path);

//        cv::Mat im1 = cv::imread(img_left_filename, cv::IMREAD_GRAYSCALE);
//        cv::Mat im2 = cv::imread(img_right_filename, cv::IMREAD_GRAYSCALE);
        cv::Mat im1 = cv::imread(img_left_filename, cv::IMREAD_COLOR);
        cv::Mat im2 = cv::imread(img_right_filename, cv::IMREAD_COLOR);


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource(new pcl::PointCloud<pcl::PointXYZ>);
        stereoConstructor_ptr->compute_match(im1, im2, cloudSource);

        /**
         *  registration here
         *  init transform: M_w_lc
         *  data pointcloud: cloudSource
         */
        Eigen::Matrix4f result_motion;
        pcl::PointCloud<pcl::PointXYZ>::Ptr downtransSource(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downlocalmap(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downSource(new pcl::PointCloud<pcl::PointXYZ>);
        pc_register.compute(cloudSource, M_w_lc, result_motion,downSource,downtransSource,downlocalmap);

        /***
         *  registration end here
         */
        // visual down
        pcl::PointCloud<pcl::PointXYZ>::Ptr ori_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*downSource,*ori_cloud,M_w_lc);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(pc_register.localmap_cloud, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(downlocalmap, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_color(downtransSource, 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> initial_value_color(downSource, 0, 0, 255);


//        viewer->addPointCloud(pc_register.localmap_cloud, target_color, "cloud1");
        viewer->removeAllPointClouds();
        viewer->addPointCloud(downlocalmap, target_color, "cloud1");
        viewer->addPointCloud(downtransSource, final_color, "cloud2");
        viewer->addPointCloud(ori_cloud, initial_value_color, "cloud3");

//        double pos_x, pos_y, pos_z;
//        pos_x = point_search.x;
//        if (point_search.y < 0) {
//            pos_y = point_search.y - 1;
//        }
//        else {
//            pos_y = point_search.y + 1;
//        }
//
//        if (point_search.z < 0) {
//            pos_y = point_search.z - 1;
//        }
//        else {
//            pos_y = point_search.z + 1;
//        }
//
//        viewer->initCameraParameters();
//        viewer->setCameraPosition (pos_x, pos_y, pos_z,
//        0, 0, 0, 0, 0, -0.1);

//        viewer->addCoordinateSystem(1.0);

        while (!viewer->wasStopped())
        {
            viewer->spinOnce();
            //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

    }


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