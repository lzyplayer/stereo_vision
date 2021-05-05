//
// Created by vickylzy on 2021/5/3.
//

#include "stereo_constructor.h"

namespace stereo_vision {

    StereoConstructor::StereoConstructor(const ParamLoader &param) : param(param) {}

    void StereoConstructor::onInit() {

        int SADWindowSize = 5;  //need modify!!! block size
        int numberOfDisparities = 16; //need modify!!! max-disparity

        if ( numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
        {
            printf(" The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
            throw;
        }
        int width =1920;
        numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((width/8) + 15) & -16;

        if (SADWindowSize < 1 || SADWindowSize % 2 != 1)
        {
            printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
            throw;
        }

        sgbm = StereoSGBM::create(0,16,3);
        sgbm->setPreFilterCap(63);
        int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
        sgbm->setBlockSize(sgbmWinSize);
        int cn = 1; // 3 channels
        sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setMinDisparity(0);
        sgbm->setNumDisparities(numberOfDisparities);
        sgbm->setUniquenessRatio(10);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(1);

//        sgbm->setMode(StereoSGBM::MODE_HH);
        sgbm->setMode(StereoSGBM::MODE_SGBM);

    }

    void saveXYZ(const char* filename, const Mat& mat)
    {
        const double max_z = 1.0e4;
        FILE* fp = fopen(filename, "wt");
        for(int y = 0; y < mat.rows; y++)
        {
            for(int x = 0; x < mat.cols; x++)
            {
                Vec3f point = mat.at<Vec3f>(y, x);
                if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
                fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
            }
        }
        fclose(fp);
    }
    int StereoConstructor::compute_match(const Mat &im_l, const Mat &im_r) {

        int64 t = getTickCount();

        //para set
        int numberOfDisparities = 16; //need modify!!! max-disparity
        numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((im_l.cols/8) + 15) & -16;
        sgbm->setNumDisparities(numberOfDisparities);


        Size img_size = im_l.size();
        Rect roi1, roi2;
        Mat Q;
        Mat R1, P1, R2, P2;
        // stereo im rectify
        stereoRectify( param.getInsML(), param.getDL(), param.getInsMR(), param.getDR(), img_size, param.getERrl(), param.getETrl(), R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, img_size, &roi1, &roi2 );

        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(param.getInsML(), param.getDL(), R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(param.getInsMR(), param.getDR(), R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r;
        remap(im_l, img1r, map11, map12, INTER_LINEAR);
        remap(im_r, img2r, map21, map22, INTER_LINEAR);

        imwrite("left_image_recitied.png",img1r);
        imwrite("right_image_recitied.png",img2r);

        //compute
        Mat disp, disp8;
//        sgbm->compute(img1r, img2r, disp);
        sgbm->compute(im_l, im_r, disp);
//

        t = getTickCount() - t;
        printf("Time elapsed: %fms\n", t*1000/getTickFrequency());


        disp.convertTo(disp8, CV_8U);


        imwrite("disparity_im.png",  disp8 );

        //save pointcloud
        Mat xyz;
        Mat floatDisp;
        float disparity_multiplier = 1.0f;
        if (disp.type() == CV_16S)
            disparity_multiplier = 16.0f;

        disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);
        reprojectImageTo3D(floatDisp, xyz, Q, false);
        saveXYZ("cloud.xyz", xyz);



    return 0;
    }



}
