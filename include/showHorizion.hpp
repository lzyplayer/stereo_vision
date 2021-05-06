//
// Created by vickylzy on 2021/5/5.
//

#ifndef INCLUDE_SHOWHORIZION_HPP
#define INCLUDE_SHOWHORIZION_HPP

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

namespace stereo_vision{
   int showRecitifyResult(const Mat& im1, const Mat& im2){
       namedWindow("recitify show", CV_WINDOW_AUTOSIZE);
       Mat img_hor_con;
       hconcat(im1,im2,img_hor_con);
       int width = img_hor_con.cols;
       for (int i = 50; i < im1.rows; i=i+50) {
           line(img_hor_con,Point(0,i),Point(width-1,i),Scalar(255,0,0));
       }

       imshow("recitify show", img_hor_con); // Show our image inside the created window.

       while(1)
       {
           if(waitKey() == 27) //ESC (prevents closing on actions like taking screenshots)
               break;
       }
       destroyWindow("recitify show"); //destroy the created window

       return 0;
   }
}

#endif //INCLUDE_SHOWHORIZION_HPP
