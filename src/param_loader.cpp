//
// Created by vickylzy on 2021/5/3.
//

#include <iostream>
#include "param_loader.h"

namespace stereo_vision{

    ParamLoader::ParamLoader(const std::string &filePath) : file_path(filePath) {
        cv::FileStorage fs(filePath, cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", filePath.c_str());
            throw std::runtime_error("file not found!");
        }
        cv::FileNode fn = fs["Mapping"];
        double ins_m[3][3];
        // left inst
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                ins_m[i][j] = static_cast<double>(fn["camera_matrix_left"]["data"][3*i+j]);
            }
        }
        ins_M_l = cv::Mat(3, 3, CV_64F, ins_m).clone();
        //right inst
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                ins_m[i][j] = static_cast<double>(fn["camera_matrix_right"]["data"][3*i+j]);
            }
        }
        ins_M_r = cv::Mat(3, 3, CV_64F, ins_m).clone();
        //left_disro
        double disort[5];
        for (int i = 0; i < 5; ++i) {
            disort[i] = static_cast<double>(fn["distortion_coefficients_left"]["data"][i]);
        }
        D_l = cv::Mat(5, 1, CV_64F, disort).clone();
        //right_distro
        for (int i = 0; i < 5; ++i) {
            disort[i] = static_cast<double>(fn["distortion_coefficients_right"]["data"][i]);
        }
        D_r = cv::Mat(5, 1, CV_64F, disort).clone();
        //exter R
        double exter_r[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                exter_r[i][j] = static_cast<double>(fn["extrinsic_matrix_Mrl"]["data"][4*i+j]);
            }
        }
        E_Rrl = cv::Mat(3, 3, CV_64F, exter_r).clone();
        //exter T
        double exter_t[3];
        int j = 3;
        for (int i = 0; i < 3; ++i) {
            exter_t[i] = static_cast<double>(fn["extrinsic_matrix_Mrl"]["data"][4*i+j]);
        }
        E_Trl = cv::Mat(3, 1, CV_64F, exter_t).clone();

//        fs["camera_matrix_left"]["data"]>>ins_M_l;
//        fn["camera_matrix_right"]>>ins_M_r;
//        fn["distortion_coefficients_left"]>>D_l;
//        fn["distortion_coefficients_right"]>>D_r;
//        fn["extrinsic_matrix_Mrl"]>>E_Mrl;

    }

    void ParamLoader::show_info() {

        std::cout<<"ins_M_l:\n"<<ins_M_l<<"\n"<<"D_l:\n"<<D_l<<"\n"<<"ins_M_r:\n"<<ins_M_r<<"\n"<<"D_r:\n"<<D_r<<"\n"<<"E_Mrl:\n"<<E_Rrl<<"\n"<<E_Trl<<std::endl;
    }

    const cv::Mat &ParamLoader::getInsML() const {
        return ins_M_l;
    }

    const cv::Mat &ParamLoader::getInsMR() const {
        return ins_M_r;
    }

    const cv::Mat &ParamLoader::getDL() const {
        return D_l;
    }

    const cv::Mat &ParamLoader::getDR() const {
        return D_r;
    }

    const cv::Mat &ParamLoader::getERrl() const {
        return E_Rrl;
    }

    const cv::Mat &ParamLoader::getETrl() const {
        return E_Trl;
    }


}