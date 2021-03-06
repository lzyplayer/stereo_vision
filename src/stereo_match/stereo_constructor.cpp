//
// Created by vickylzy on 2021/5/3.
//

#include "stereo_match/stereo_constructor.h"
#include "stereo_match/showHorizion.hpp"
#include "stereo_match/pcl_cv_conversion.hpp"
#include <utility>

namespace stereo_vision {


    StereoConstructor::StereoConstructor(ParamLoader param) : param(std::move(param)) {}

    void StereoConstructor::onInit(int SADWindowSize, int minDisparity, int numberOfDisparities) {

        sgbm = StereoSGBM::create(0, 16, 3);

        if (numberOfDisparities < 1 || numberOfDisparities % 16 != 0) {
            printf(" The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
            throw;
        }
        if (SADWindowSize < 1 || SADWindowSize % 2 != 1) {
            printf(" parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
            throw;
        }
        //para set
        sgbm->setPreFilterCap(1);
        sgbm->setBlockSize(SADWindowSize);
        sgbm->setMinDisparity(minDisparity);
        sgbm->setNumDisparities(numberOfDisparities);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(5);
        sgbm->setUniquenessRatio(20);
        sgbm->setMode(StereoSGBM::MODE_HH);
//        sgbm->setMode(StereoSGBM::MODE_SGBM);

    }

    void saveXYZ(const char *filename, const Mat &mat,float scale) {
        const double max_z = 1.0e5;
        FILE *fp = fopen(filename, "wt");
        for (int y = 0; y < mat.rows; y++) {
            for (int x = 0; x < mat.cols; x++) {
                Vec3f point = mat.at<Vec3f>(y, x);
                if (fabs(point[2]*scale - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
                if (point[2]*scale>9) continue;
                fprintf(fp, "%f %f %f\n", point[0]*scale, point[1]*scale, point[2]*scale);
            }
        }
        fclose(fp);
    }

    int StereoConstructor::compute_match(const Mat &im_l, const Mat &im_r, pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_stereovision) {

        int64 t = getTickCount();
        int cn = im_l.channels(); // 3 channels
        if (cn == 3)
            --cn;
        int sgbmWinSize = sgbm->getBlockSize();
        sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);
        sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);

        Size img_size = im_l.size();
        Rect roi1, roi2;
        Mat Q;
        Mat R1, P1, R2, P2;
        // stereo im rectify
        stereoRectify(param.getInsML(), param.getDL(), param.getInsMR(), param.getDR(), img_size, param.getERrl(), param.getETrl(), R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, img_size, &roi1, &roi2);

        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(param.getInsML(), param.getDL(), R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(param.getInsMR(), param.getDR(), R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r;
        remap(im_l, img1r, map11, map12, INTER_LINEAR);
        remap(im_r, img2r, map21, map22, INTER_LINEAR);

//        imwrite("left_image_recitied.png",img1r);
//        imwrite("right_image_recitied.png",img2r);

        //compute
        Mat disp, disp8;
        sgbm->compute(img1r, img2r, disp);
//        showRecitifyResult(img1r,img2r);
//        sgbm->compute(im_l, im_r, disp);
//        showRecitifyResult(im_l,im_r);

//
        t = getTickCount() - t;
        printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());

        Mat normalized_im;
#if  (CV_MAJOR_VERSION == 4)
        normalize(disp, normalized_im, 0, 255, NORM_MINMAX, CV_8UC1);
#else
        normalize(disp, normalized_im, 0, 255, CV_MINMAX, CV_8UC1);
#endif
        imwrite("disparity_im.png", normalized_im);

        //save pointcloud
        Mat xyz;
        Mat floatDisp;
        float disparity_multiplier = 1.0f;
        if (disp.type() == CV_16S)
            disparity_multiplier = 16.0f;

        disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);
        reprojectImageTo3D(floatDisp, xyz, Q, true);
        float scale_m_mm = 1e-3;

        saveXYZ("cloud.xyz", xyz,scale_m_mm);
        //  precision  milimter 2 meter
        pc_stereovision = MatToPoinXYZ(xyz, scale_m_mm);


        return 0;
    }


}
