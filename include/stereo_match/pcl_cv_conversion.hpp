
#ifndef INCLUDE_PCL_CV_CONVERSION_HPP
#define INCLUDE_PCL_CV_CONVERSION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/utility.hpp>


namespace stereo_vision {


    pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(const cv::Mat &OpencVPointCloud, float scale) {
        const double max_z = 1.0e5;
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);//(new pcl::pointcloud<pcl::pointXYZ>);
        for (int y = 0; y < OpencVPointCloud.rows; y++) {
            for (int x = 0; x < OpencVPointCloud.cols; x++) {
                Vec3f point = OpencVPointCloud.at<Vec3f>(y, x);
                if (fabs(point[2]*scale - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
                if (point[2]*scale>9) continue;
                pcl::PointXYZ point_pcl;
                point_pcl.x = point[0] * scale;
                point_pcl.y = point[1] * scale;
                point_pcl.z = point[2] * scale;
                point_cloud_ptr->points.push_back(point_pcl);
            }
        }
        point_cloud_ptr->width = static_cast<size_t >(point_cloud_ptr->points.size());
        point_cloud_ptr->height = 1;

        return point_cloud_ptr;

    }

    cv::Mat PoinXYZToMat(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud_ptr) {

        cv::Mat OpenCVPointCloud(3, point_cloud_ptr->points.size(), CV_64FC1);
        for (int i = 0; i < point_cloud_ptr->points.size(); i++) {
            OpenCVPointCloud.at<double>(0, i) = point_cloud_ptr->points.at(i).x;
            OpenCVPointCloud.at<double>(1, i) = point_cloud_ptr->points.at(i).y;
            OpenCVPointCloud.at<double>(2, i) = point_cloud_ptr->points.at(i).z;
        }

        return OpenCVPointCloud;
    }
}


#endif //INCLUDE_PCL_CV_CONVERSION_HPP
