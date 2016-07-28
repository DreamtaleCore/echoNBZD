#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

RNG rng(12345);

int main()
{
    VideoCapture cap;
    stringstream ss;

    int postFix = rng.uniform(1, 2000);

    ss << "/tmp/videoGet" << postFix << ".avi";

    for(int i = 0; i < 10; i ++)
    {
        cap.open(i);
        if(cap.isOpened())
            break;
    }

    Mat frame;

    cap >> frame;


    VideoWriter vw = VideoWriter(ss.str(), CV_FOURCC('M', 'J', 'P', 'G'), 10, frame.size());

    while (cap.isOpened())
    {
        cap >> frame;
        vw << frame;

        imshow("test", frame);

        if(waitKey(100) == 'q')
            break;
    }


    return 0;
}