//
// Created by vickylzy on 2021/5/3.
//

#ifndef SRC_STEREOCONSTRUCTOR_H
#define SRC_STEREOCONSTRUCTOR_H

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include "param_loader.h"



namespace stereo_vision {



    using namespace cv;

    class StereoConstructor {
    public:
        void onInit();
        int compute_match(const Mat& im_l,const Mat& im_r);
        explicit StereoConstructor(const ParamLoader &param);

    private:
        ParamLoader param;
        cv::Ptr<cv::StereoSGBM> sgbm;
    };

}
#endif //SRC_STEREOCONSTRUCTOR_H
