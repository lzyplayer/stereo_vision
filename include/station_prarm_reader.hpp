#ifndef INCLUDE_STEREO_MATCH_STATION_PRARM_READER_HPP
#define INCLUDE_STEREO_MATCH_STATION_PRARM_READER_HPP


#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <stdio.h>

#define ARG_FETCH_BLOCK    1
#define ARG_FETCH_NONBLOCK 0
#define CHK_FILE_KEYWORD "+-fin-+"
#define ARG_LIST_LEN_MAX 8192

namespace stereo_vision{

    static char *checkFile(const char *pathname,
                           const char *filename, int wait)
    {
        FILE *f;
        char filepath[FILENAME_MAX];
        char *chk;
        char *buf = (char*)malloc(ARG_LIST_LEN_MAX);
        sprintf(filepath, "%s/%s", pathname, filename);
        CHK_FILE_LABEL:
        while (true) {
            f = fopen(filepath, "r");
            if (f) {
                break;
            } else if (!wait) {
                free(buf);
                return NULL;
            }
            usleep(1000 * 20); /* Wait 20ms */
        }
        fread(buf, ARG_LIST_LEN_MAX, 1, f);
        fclose(f);
        chk = strstr(buf, CHK_FILE_KEYWORD);
        if (!chk) {
            usleep(1000 * 20); /* Wait 20ms */
            if (wait) {
                goto CHK_FILE_LABEL;
            } else {
                free(buf);
                return NULL;
            }
        }
        *chk = '\0';
        return buf;
    }

    /**
     * station_param
     * [0]: arm joint 1 theta
     * [1]: arm joint 2 theta
     * [2]: arm joint 3 theta
     * [3]: arm joint 4 theta
     * [4]: arm joint 5 theta
     * [5]: arm joint 6 theta
     * [6]: arm joint 7 theta
     * [7]: Translation x M_world_arm
     * [8]: Translation y M_world_arm
     * [9]: Translation z M_world_arm
     * [10]: Rotation z M_world_arm
     * [11]: Rotation y M_world_arm
     * [12]: Rotation x M_world_arm
     * [13]: base footprint ID
     * [14]: currentEndCam "55"-1a  "AA"-2b
     */
    std::vector<std::string> getArgList(int wait,
                                        const char *pathname, const char *filename)
    {
        std::string file;
        std::vector<std::string> ret;
        char *s, *csr, *findch;
        s = checkFile(pathname, filename, wait);
        if (!s)
            return ret;
        csr = s;
        while (true) {
            findch = strchr(csr, '\n');
            if (!findch)
                break;
            *findch = '\0';
            ret.emplace_back(csr);
            csr = findch + 1;
        }
        free(s);
        //remove file after read
        std::string combined_name = std::string(pathname) + filename;
        if( remove( combined_name.c_str() ) != 0 )
            perror( "Error deleting file" );
        else
            puts( "File successfully deleted" );
        //
        return ret;
    }



//    int main()
//    {
//        std::vector<std::string> v = getArgList(ARG_FETCH_BLOCK, "./", "data.txt");
//        for (auto i : v) {
//            std::cout << i << "\n";
//        }
//        return 0;
//    }

}




#endif //INCLUDE_STEREO_MATCH_STATION_PRARM_READER_HPP
