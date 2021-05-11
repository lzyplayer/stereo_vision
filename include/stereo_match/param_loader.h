//
// Created by vickylzy on 2021/5/3.
//

#ifndef SRC_PARAM_LOADER_H
#define SRC_PARAM_LOADER_H

#include <string>
#include <opencv2/core/utility.hpp>


namespace stereo_vision {
    class ParamLoader {
    public:
        explicit ParamLoader(const std::string &filePath);

        void show_info();

        const cv::Mat &getInsML() const;

        const cv::Mat &getInsMR() const;

        const cv::Mat &getDL() const;

        const cv::Mat &getDR() const;

        const cv::Mat &getERrl() const;

        const cv::Mat &getETrl() const;

    private:
        std::string file_path;
        cv::Mat ins_M_l, ins_M_r, D_l, D_r, E_Rrl, E_Trl, M_ee_lcam;
    public:
        const cv::Mat &getMEeLcam() const;
        // M_ee_lcam = M_ee_pointcloud

    };
}


#endif //SRC_PARAM_LOADER_H
