#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "stereo_constructor.h"
#include "param_loader.h"

using namespace std;
std::string getPathName(const char *fifopath)
{
    FILE *f = fopen(fifopath, "rb");
    char b[FILENAME_MAX];
    fread(b, FILENAME_MAX, 1, f);
    fclose(f);
    return std::string(b);
}



int main() {
    std::cout << "Hello, World!" << std::endl;
//
//    stereo_vision::ParamLoader paramLoader("/home/vickylzy/CLionProjects/stereo_vision/camera_info/camera_stereo_merged_1.yaml");
    stereo_vision::ParamLoader paramLoader("/home/vickylzy/CLionProjects/stereo_vision/camera_info/camera_stereo_1_matlab.yaml");
    paramLoader.show_info();
    stereo_vision::StereoConstructor stereoConstructor(paramLoader);
    stereoConstructor.onInit(13,128,32);

//    string img1_filename = "/home/vickylzy/Pictures/imgL.jpg";
//    string img2_filename = "/home/vickylzy/Pictures/imgR.jpg";

    string img1_filename = "/home/vickylzy/Documents/space_station_arm/test_image/left_back.png";
    string img2_filename = "/home/vickylzy/Documents/space_station_arm/test_image/right_back.png";

//    string img1_filename = "/home/vickylzy/CLionProjects/stereo_vision/data/imgL.jpg";
//    string img2_filename = "/home/vickylzy/CLionProjects/stereo_vision/data/imgR.jpg";

//    cv::Mat im1 = cv::imread(img1_filename,cv::IMREAD_COLOR);
//    cv::Mat im2 = cv::imread(img2_filename,cv::IMREAD_COLOR);

    cv::Mat im1 = cv::imread(img1_filename,cv::IMREAD_GRAYSCALE);
    cv::Mat im2 = cv::imread(img2_filename,cv::IMREAD_GRAYSCALE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTrimmed;// uninitialized or initialized
    stereoConstructor.compute_match(im1,im2,cloudTrimmed);

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