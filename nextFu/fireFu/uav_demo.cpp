#include "utilities.h"

int main()
{
    VideoCapture capture(0);
    if (!capture.isOpened()) return 0;
    while (1)
    {
        Mat src_frame;
        capture >> src_frame;
        Mat hsv_img_h;
        //utilities test;
        hsv_img_h=bgr2Hsv(src_frame, 0);
        imshow("test", hsv_img_h);
        waitKey(10);
    }
    return 0;
}
