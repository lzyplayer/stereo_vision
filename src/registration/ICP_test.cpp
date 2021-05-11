#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/filters/filter.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <boost/shared_ptr.hpp>

//#include <conio.h>


int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRef(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRef_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource_filtered(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::io::loadPCDFile("/home/shaoyi/ICP/pt1_single.pcd", *cloudRef);
    pcl::io::loadPCDFile("/home/shaoyi/ICP/pt2_single.pcd", *cloudSource);
    Eigen::Matrix4f m_InitTF;
    m_InitTF.setIdentity();
    m_InitTF << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloudRef);
    sor.setLeafSize(100.0f, 100.0f, 100.0f);
    sor.filter(*cloudRef_filtered);

    sor.setInputCloud(cloudSource);
    sor.setLeafSize(100.0f, 100.0f, 100.0f);
    sor.filter(*cloudSource_filtered);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudSource_filtered);
    icp.setInputTarget(cloudRef_filtered);

    pcl::PointCloud<pcl::PointXYZ> ptcloud_transformed;
    icp.align(ptcloud_transformed, m_InitTF);

    //pcl::visualization::CloudViewer viewer("Cloud Viewer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloudRef_filtered, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_color(ptcloud_transformed.makeShared(), 255, 0, 0);


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud(cloudRef_filtered, target_color, "cloud1");
    viewer->addPointCloud(ptcloud_transformed.makeShared(), final_color, "cloud2");

    std::cout << icp.getFinalTransformation() << std::endl;

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }




    //_getch();
    //while (!viewer.wasStopped())
    //{
    //	std::cout << cloudRef_filtered->width << endl;
    //	std::cout << cloudRef_filtered->height << endl;
    //	//you can also do cool processing here
    //	//FIXME: Note that this is running in a separate thread from viewerPsycho
    //	//and you should guard against race conditions yourself...
    //	user_data++;
    //}
    return 0;
}
