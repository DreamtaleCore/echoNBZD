#ifndef UTILITIES_H
#define UTILITIES_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
using namespace cv;
#define HsvType int
#define HSV_H 0
#define HSV_H_CHANGLE 1
#define HSV_S_CHANGLE 2
#define HSV_V_CHANGLE 3
Mat bgr2Hsv(Mat img, HsvType hsvtype);

#endif // UTILITIES_H
