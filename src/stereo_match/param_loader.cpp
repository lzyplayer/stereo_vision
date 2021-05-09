//
// Created by vickylzy on 2021/5/3.
//

#include <iostream>
#include "stereo_match/param_loader.h"

namespace stereo_vision{

    ParamLoader::ParamLoader(const std::string &filePath) : file_path(filePath) {
        cv::FileStorage fs(filePath, cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", filePath.c_str());
            throw std::runtime_error("file not found!");
        }

        cv::Mat result;

        fs["camera_matrix_left"] >> ins_M_l;
        fs["camera_matrix_right"]>>ins_M_r ;
        fs["distortion_coefficients_left"]>>D_l ;
        fs["distortion_coefficients_right"]>>D_r ;
        fs["extrinsic_R_Mrl"]>>E_Rrl ;
        fs["extrinsic_T_Mrl"]>>E_Trl ;


    }

    void ParamLoader::show_info() {

        std::cout<<"ins_M_l:\n"<<ins_M_l<<"\n"<<"D_l:\n"<<D_l<<"\n"<<"ins_M_r:\n"<<ins_M_r<<"\n"<<"D_r:\n"<<D_r<<"\n"<<"E_Rrl:\n"<<E_Rrl<<"\n"<<"E_trl:\n"<<E_Trl<<std::endl;
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