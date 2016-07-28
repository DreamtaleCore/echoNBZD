#ifndef CV4STAB_H
#define CV4STAB_H

// include opencv libraries
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/legacy/compat.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>

// include extra std libraries
#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>

using namespace std;
using namespace cv;

#define SEGSIZE 30        //每次处理帧数
#define WINDOWSIZE 30       //计算帧匹配代价窗口
#define GAPSIZE 5         //首关键帧与尾关键帧分别在段前GAPSIZE帧和后GAPSIZE帧中
#define TAUS 200
#define TAUA 200
#define VELOCITY 30
//是否去抖振幅判断阈值
#define SHIFT_SD 0.4
#define ROTATE_SD 0.017
#define SCALE_SD 0.1

// get the video from a file then deal with it and
// then storage it into a file, just show it in the
// point window
//int videoStab(string inputFile, string outputFile, string windowName);

#endif // CV4STAB_H
