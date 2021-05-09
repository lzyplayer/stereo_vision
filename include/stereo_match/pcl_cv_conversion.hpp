
#ifndef INCLUDE_PCL_CV_CONVERSION_HPP
#define INCLUDE_PCL_CV_CONVERSION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/utility.hpp>


namespace stereo_vision{


pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(const cv::Mat &OpencVPointCloud, float scale)
{
    /*
    *  Function: Get from a Mat to pcl pointcloud datatype
    *  In: cv::Mat
    *  Out: pcl::PointCloud
    */

    //char pr=100, pg=100, pb=100;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);//(new pcl::pointcloud<pcl::pointXYZ>);

    for(int i=0;i<OpencVPointCloud.cols;i++)
    {
        //std::cout<<i<<endl;

        pcl::PointXYZ point;
        point.x = OpencVPointCloud.at<float>(0,i)*scale;
        point.y = OpencVPointCloud.at<float>(1,i)*scale;
        point.z = OpencVPointCloud.at<float>(2,i)*scale;

        // when color needs to be added:
        //uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
        //point.rgb = *reinterpret_cast<float*>(&rgb);

        point_cloud_ptr -> points.push_back(point);


    }
    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

    return point_cloud_ptr;

}
    cv::Mat  PoinXYZToMat(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud_ptr){

        cv::Mat OpenCVPointCloud(3, point_cloud_ptr->points.size(), CV_64FC1);
        for(int i=0; i < point_cloud_ptr->points.size();i++){
            OpenCVPointCloud.at<double>(0,i) = point_cloud_ptr->points.at(i).x;
            OpenCVPointCloud.at<double>(1,i) = point_cloud_ptr->points.at(i).y;
            OpenCVPointCloud.at<double>(2,i) = point_cloud_ptr->points.at(i).z;
        }

        return OpenCVPointCloud;
    }
}


#endif //INCLUDE_PCL_CV_CONVERSION_HPP
