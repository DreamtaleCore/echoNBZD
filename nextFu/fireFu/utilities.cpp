#include "utilities.h"
#define HsvType int

Mat bgr2Hsv(Mat img, HsvType hsvtype){
    Mat hsv_img, hsv_img_h, hsv_img_s, hsv_img_v;
    cvtColor(img, hsv_img, COLOR_BGR2HSV);
    vector<Mat> channels;
    split(hsv_img, channels);
    hsv_img_h = channels[0];
    hsv_img_s = channels[1];
    hsv_img_v = channels[2];
    switch (hsvtype){
    case 0:
        return hsv_img;
    case 1:
        return hsv_img_h;
    case 2:
        return hsv_img_s;
    case 3:
        return hsv_img_v;
    }
    return hsv_img;
}
