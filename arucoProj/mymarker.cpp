#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src; Mat src_gray;
int thresh = 100;
int max_thresh = 255;

/// Function header
void thresh_callback(int, void* );

/** @function main */
int main( int argc, char** argv )
{
    VideoCapture cap;
    cap.open("/home/nbzd/ws/Projs/stabelVideo.avi");

    while (cap.isOpened())
    {

        /// Load source image and convert it to gray
        //cap >> src;
        src = imread( "/home/nbzd/Pictures/now.png", 1 );

        if(src.empty())
            return 0;

        /// Convert image to gray and blur it
        cvtColor( src, src_gray, CV_BGR2GRAY );
        Mat hsv;
        cvtColor(src, hsv, CV_BGR2HSV);
        vector<Mat> imgSp;
        split(hsv, imgSp);

        Mat hsvV = imgSp[2], thr1;

        imshow("hsv_v", hsvV);

        threshold(hsvV, thr1, 85, 255, 0);

        blur( thr1, src_gray, Size(3,3));

        imshow("Src_Gray", src_gray);

        /// Create Window
        char* source_window = "Source";
        namedWindow( source_window, CV_WINDOW_AUTOSIZE );
        imshow( source_window, src );

        createTrackbar( " Canny thresh:", "Source", &thresh, max_thresh, thresh_callback );
        thresh_callback( 0, 0 );

        if(waitKey(1) == 27)
            break;
    }
    return(0);
}

/** @function thresh_callback */
void thresh_callback(int, void* )
{
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using canny
    Canny( src_gray, canny_output, thresh, thresh*2, 3 );

    imshow("canny", canny_output);
    /// Find contours
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Draw contours
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    vector<Point > approxCurve;

    for( int i = 0; i< contours.size(); i++ )
    {
        approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * 0.05, true);
        if(approxCurve.size() == 4 && contourArea(contours[i]) > 40)
        {
            vector<vector<Point> > rect;
            rect.push_back(approxCurve);
            Scalar color = Scalar( 255, 0, 255);
            drawContours( drawing, rect, 0, color, 2, 8, hierarchy, 0, Point() );
        }
    }

    /// Show in a window
    imshow( "Contours", drawing );
}

