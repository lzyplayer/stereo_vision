//
// Created by vickylzy on 2021/5/9.
//

#ifndef INCLUDE_STEREO_MATCH_TRANSFER_INFO_HPP
#define INCLUDE_STEREO_MATCH_TRANSFER_INFO_HPP

#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>
#include <chrono>
#include <thread>

namespace tran_info {

    const char *trans_dir_path = "/dev/shm/c590/cacheA/";
    const char *pid_txt_path = "/dev/shm/c590/cacheA/pid.txt";
    const char *req_list_path = "/dev/shm/c590/cacheA/req.list";
    const char *im_cam0_path = "/dev/shm/c590/cacheA/cam0.jepg";
    const char *im_cam1_path = "/dev/shm/c590/cacheA/cam1.jpeg";

    inline bool check_exist(const std::string &name) {
        std::ifstream f(name.c_str());
        return f.good();
    }

    int clear_pidtxt() {
        if (remove(tran_info::pid_txt_path) != 0)
            perror("Error deleting file");
        else
            puts("Orginal pid.txt successfully deleted");
        return 0;
    }

    int save_pidtxt() {
        std::ofstream outfile(tran_info::pid_txt_path);
        outfile << getpid() << std::endl;
        outfile.close();
        return 0;
    }

    // write reqlist and wait if already exists
    int write_reqlist() {
        while (check_exist(req_list_path)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::ofstream outfile(req_list_path);
        outfile << "cam1\n" << "cam2" << std::endl;
        outfile << "+-fin-+" << std::endl;
        outfile.close();
        return 0;
    }


}


#endif //INCLUDE_STEREO_MATCH_TRANSFER_INFO_HPP
